"""
Tiago
=====
High-level agent for a PAL Tiago mobile manipulator.

Accepts a list of (pick_object, place_object) name pairs and executes them
sequentially.  Each object name is a Gazebo model name; world-frame positions
are resolved automatically via ObjectToRobot (gz-ros2-bridge).

Pose source
-----------
The robot's pose is read live from Gazebo Fortress via the gz-ros2 bridge topic
``/world/<world_name>/pose/info`` — this is the ground-truth world pose, not
wheel odometry.  No spawn-pose parameter is required regardless of where the
robot is placed in the world.  The nav loop compares world-frame goals directly
against the live world pose, so there is no odometry drift.

Prerequisite: the gz-ros2 bridge must be running::

    ros2 run ros_gz_bridge parameter_bridge \\
      /world/<world_name>/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
      /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

Architecture
------------
                    ┌─────────────────────────────────────┐
                    │  Tiago (Agent subclass)              │
                    │                                     │
                    │  ObjectToRobot ──► world_pose XYZ   │
                    │       │  (same /pose/info topic)    │
                    │       ▼                             │
                    │  TiagoAdapter  (world-frame + GT)   │
                    │    ├─ TiagoPickPlacePlanner          │
                    │    │    └─ nav pose computation      │
                    │    ├─ P-controller navigation        │
                    │    │    └─ ground-truth feedback     │
                    │    └─ TiagoGripperAdapter            │
                    │  TiagoNavigator (standalone nav)     │
                    └─────────────────────────────────────┘

ObjectToRobot shares the adapter's ROS2 node so all subscriptions live on
the same node and are served by the same MultiThreadedExecutor.

Usage
-----
    tiago = Tiago(
        robot_name  = "tiago_robot1",
        world_name  = "backyard",
        skill_weights = ...,
        contexts    = ...,
        role        = ...,
        teamsize    = 1,
    )
    # Navigate by location name (from the locations dict):
    tiago.navigate_to("DiningTable")

    # Navigate by Gazebo model name (resolved via OTR):
    tiago.navigate_to("prep_table_1")

    # Navigate by explicit world-frame XYZ:
    tiago.navigate_to((-0.77, -1.54, 0.0))

    # Name-based (resolved via Gazebo / ObjectToRobot):
    tiago.pick_and_place_objects([
        ("tomato_1", "bowl_1"),
        ("meat_1",   "plate_1"),
    ])

    # Coordinate-based (world-frame XYZ tuples):
    tiago.pick_and_place_objects([
        ((0.5, 1.2, 0.85), (0.5, 2.0, 0.85)),
    ])

    # Mixed — pick by name, place by coordinate (or vice-versa):
    tiago.pick_and_place_objects([
        ("tomato_1", (0.5, 2.0, 0.85)),
        ((0.5, 1.2, 0.85), "bowl_1"),
    ])
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple, Union

import rclpy
import rclpy.parameter
from rclpy.executors import MultiThreadedExecutor

import tf2_ros

from src.entities.Agent import Agent
from tiago_adapters.TiagoAdapter import TiagoAdapter, RobotMode
from tiago_navigator import TiagoNavigator
from robot_common.object_world_to_robot import ObjectToRobot
from robot_common.sdf_surface_resolver import SdfSurfaceResolver
from utils import ROSUtils

from locations import get_locations, get_scene

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ        = Tuple[float, float, float]
NameOrXYZ  = Union[str, XYZ]           # object name  OR  explicit (x,y,z)
ObjectPair = Tuple[NameOrXYZ, NameOrXYZ]  # (pick, place) — each can be a name or XYZ

CONST_SCALE = 10

TIME_PICK_THE_DISHES = 600/CONST_SCALE  
TIME_DOING_THE_DISHES = 600/CONST_SCALE    # seconds to "do the dishes" (simulate with sleep)
TIME_PUTTING_AWAY_DISHES = 300/CONST_SCALE    # seconds to "put away dishes" (simulate with sleep)

TIME_PICKING_RICE = 330/CONST_SCALE     # seconds to "pick" the rice (simulate with sleep)
TIME_COOKING_RICE = 2400/CONST_SCALE    # seconds to "cook" the rice (simulate with sleep)
TIME_SERVE_MAIN_DISH = 120/CONST_SCALE

TIME_WELCOMING_GUESTS = 5#15.0  # seconds to "welcome guests" (simulate with sleep)

TIME_PICK_FOOD_GRILL = 300/CONST_SCALE
TIME_GRILL_FOOD = 5000/CONST_SCALE    # seconds
TIME_PICK_GRILLED_FOOD = 300/CONST_SCALE
TIME_SERVE_GRILLED_FOOD = 120/CONST_SCALE

TIME_PICK_VEGETABLE = 300/CONST_SCALE
TIME_CHOP_VEGETABLES = 3000/CONST_SCALE    # seconds
TIME_PICK_CHOPPED_VEGETABLE = 300/CONST_SCALE
TIME_SERVE_SIDES = 120/CONST_SCALE

TIME_PICK_SALAD = 300/CONST_SCALE
TIME_CHOP_SALAD_INGREDIENTS = 2000/CONST_SCALE    # seconds
TIME_PICK_CHOPPED_SALAD = 300/CONST_SCALE
TIME_SERVE_SALAD = 120/CONST_SCALE

#TIME_SERVING_DRINKS = 0

#TIME_HOST = 0

TIME_SAYING_GOODBYE = 15.0

# ---------------------------------------------------------------------------
# TF-based arm height lookup
# ---------------------------------------------------------------------------

_ARM_BASE_Z_FALLBACK = 0.83   # metres — used only if TF lookup fails

def _lookup_arm_base_z(robot_name: str, timeout: float = 10.0) -> float:
    """
    Look up the actual height of arm_1_link above the floor by reading the
    TF tree.  This is always more accurate than a hardcoded constant because
    it reflects the torso lift joint position at the time of the call.

    Creates a temporary ROS2 node, waits for the TF tree to populate, reads
    the transform, then destroys the node.  Falls back to
    _ARM_BASE_Z_FALLBACK (0.83 m) if the transform is unavailable.
    """
    import time as _time

    arm_link  = f"{robot_name}/arm_1_link"
    base_link = f"{robot_name}/base_footprint"

    try:
        tmp_node   = rclpy.create_node(f"_arm_z_probe_{robot_name}")
        tf_buffer  = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, tmp_node)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(tmp_node)

        deadline = _time.time() + timeout
        z = None
        while _time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
            try:
                tf = tf_buffer.lookup_transform(
                    base_link, arm_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                z = tf.transform.translation.z
                break
            except Exception:
                pass

        executor.shutdown()
        tmp_node.destroy_node()

        if z is not None:
            print(f"[Tiago] arm_1_link height from TF: {z:.4f} m")
            return float(z)

    except Exception as exc:
        print(f"[Tiago] TF arm_base_z lookup failed: {exc}")

    print(f"[Tiago] WARNING: using fallback arm_base_z = {_ARM_BASE_Z_FALLBACK} m")
    return _ARM_BASE_Z_FALLBACK


_GRIPPER_Z_OFFSET_FALLBACK = 0.143  # metres — standard PAL Tiago grasping_frame offset

# NOTE: gripper_z_offset is handled internally by TiagoPickPlacePlanner._load_chain.
# It reads the arm-only URDF to detect whether gripper_grasping_frame is in the chain.
# No external TF lookup is needed.

class Tiago(Agent):
    """
    Parameters
    ----------
    robot_name : str
        Agent identifier (e.g. "tiago_robot1").
        Gazebo / ROS2 namespace for this robot (e.g. "tiago_robot1").
    world_name : str
        Gazebo world name that matches the gz-ros2-bridge command, e.g. "backyard".
        Must match <world name="…"> in the .sdf AND the bridge topic
        /world/<world_name>/pose/info.
    skill_weights, contexts, role, teamsize :
        Passed through to the Agent base class.
    arm_base_z : float or None
        Height of arm_1_link above the floor (m).
        When None (default), the value is read live from the TF tree at
        startup — this is always more accurate than a hardcoded constant
        because it reflects the actual torso lift position.
        Only pass an explicit float if TF is unavailable.
    sdf_path : str, optional
        Absolute path to the world .sdf file.  When provided:
          - Top-surface z offsets are computed automatically from the SDF
            geometry (no manual place_z_offset needed).
          - Navigation uses table-aware standoff poses: the robot stops at
            ``preferred_reach`` from the object, projected perpendicular to
            the nearest table face, with a minimum of ``table_standoff`` metres
            clearance from the face edge.
        If omitted, raw model origin z is used and the old preferred_reach
        nav behaviour applies.
    models_base_dir : str, optional
        Root directory that contains ``<model_name>/model.sdf`` files
        (e.g. ``/ros2_ws/src/my_pkg/simulation/models``).
        Used by the table resolver to parse external model footprints
        referenced by ``<include>`` tags in the world SDF.
        Falls back to GAZEBO_MODEL_PATH if omitted.
    table_standoff : float
        Minimum metres of clearance between the robot base and the table
        face edge (default 0.10). The prep table has no side-wall collision
        geometry, so the base can safely approach within 0.10 m of the slab
        edge. The actual arm reach is auto-computed to reach_pref=0.55 m.
    pose_timeout : float
        Seconds to wait for the first pose/info message on startup. Default 15.
    nav_timeout : float
        Seconds to wait for a navigation goal to complete. Default 120.
    """

    def __init__(
        self,
        robot_name:     str,
        world_name:     str,
        constraints:    dict            = None,
        skill_weights:  dict            = None,
        contexts:       List[str]       = None,
        role:           str             = "member",
        teamsize:       int             = 1,
        run_id:         str             = "test",
        use_sim:        bool            = True,
        workspace:      List[str]       = None,
        party_duration: float           = 3600.0, #1h default
        guests:         int             = 0,
        arm_base_z:     Optional[float] = None,
        pose_timeout:   float           = 15.0,
        table_standoff                  = 0.10,
        nav_timeout:    float           = 120.0,
    ) -> None:
        
        super().__init__(
            robot_name, 
            constraints or {},
            workspace or [], 
            skill_weights or {}, 
            contexts or [], 
            role, 
            teamsize, 
            party_duration, 
            guests, 
            run_id
        )

        if use_sim:
            mode = RobotMode.SIMULATION
        else:
            mode = RobotMode.REAL_WORLD

        self._robot_name = robot_name
        self._world_name = world_name
        self._locations = get_locations(world_name)

        # ── Auto-resolve SDF path and models dir ──────────────────────
        # Mirror the test script (tiago_pick_and_place.py): use ROSUtils to
        # find the world SDF and models directory automatically when not
        # supplied by the caller — this is what makes SdfSurfaceResolver work.
        try:
            sdf_path = ROSUtils._get_sdf_path(world_name)
            print(f"[Tiago] Auto-resolved SDF path: {sdf_path}")
        except FileNotFoundError as exc:
            print(f"[Tiago] WARNING: SDF not found ({exc}) — surface offsets unavailable.")
        
        try:
            models_base_dir = ROSUtils._get_models_dir()
            print(f"[Tiago] Auto-resolved models dir: {models_base_dir}")
        except FileNotFoundError as exc:
            print(f"[Tiago] WARNING: models dir not found ({exc}).")

        # Event + storage for adapter completion signal
        self._goal_done_event = threading.Event()
        self._last_result: Tuple[bool, str] = (False, "No goal sent yet.")
        self._nav_timeout = nav_timeout

        # ── ROS2 initialisation ───────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        # TiagoAdapter in world-frame mode:
        # receives world-frame XYZ, computes nav poses internally, drives base.
        # arm_base_z is looked up from TF if not provided explicitly.
        resolved_arm_base_z = arm_base_z if arm_base_z is not None \
            else _lookup_arm_base_z(robot_name)

        # TiagoAdapter in world-frame mode with Gazebo ground-truth pose feedback.
        # Passing world_name activates the /world/<world_name>/pose/info subscription
        # inside the adapter: the nav loop reads the robot's actual Gazebo world pose
        # every cycle instead of integrating wheel odometry.  This eliminates the
        # odom drift that accumulates over longer navigation distances and means
        # spawn_world_pose is never needed — no manual coordinate is required.
        self._adapter = TiagoAdapter(
            robot_name      = robot_name,
            mode            = mode,
            result_callback = self._on_result,
            robot_frame     = False,
            arm_base_z      = resolved_arm_base_z,
            world_sdf_path  = sdf_path,
            models_base_dir = models_base_dir,
            table_standoff  = table_standoff,
            world_name      = world_name,   # activates ground-truth pose; no spawn param needed
        )

        # ObjectToRobot shares the adapter node so all subscriptions live on
        # the same executor.  Both OTR and the adapter subscribe to
        # /world/<world_name>/pose/info; ROS2 delivers the messages to each
        # callback independently — no conflict.
        self._otr = ObjectToRobot(
            node         = self._adapter,
            robot_name   = robot_name,
            world_name   = world_name,
            pose_timeout = pose_timeout,
        )

        # ── Executor: spins adapter node + gripper node ───────────────
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._adapter)
        self._executor.add_node(self._adapter._gripper)

        self._spin_thread = threading.Thread(
            target = self._executor.spin,
            daemon = True,
            name   = f"tiago_{robot_name}_spin",
        )
        self._spin_thread.start()

        # ── TiagoNavigator (standalone navigation) ────────────────────
        # Shares the same world_name / SDF / models_dir so it uses the
        # same ground-truth pose source and table-aware standoff logic.
        self._navigator = TiagoNavigator(
            robot_name      = robot_name,
            world_sdf_path  = sdf_path,
            models_base_dir = models_base_dir,
            table_standoff  = table_standoff,
            world_name      = world_name,
        )
        self._executor.add_node(self._navigator)
        self._nav_done_event = threading.Event()
        self._nav_result: Tuple[bool, str] = (False, "No nav goal sent yet.")

        # ── Ground-truth pose confirmation ────────────────────────────
        # Wait for the first GT message so we know the bridge is live before
        # attempting navigation.  The adapter's _pose_ready event fires on
        # the first /world/<world_name>/pose/info message (or first odom if
        # the bridge is slow — the nav loop handles both gracefully).
        print(f"[Tiago] Waiting for ground-truth pose from "
              f"/world/{world_name}/pose/info ...")
        if self._adapter._pose_ready.wait(timeout=pose_timeout):
            if self._adapter._use_ground_truth:
                wx, wy, wth = self._adapter._get_robot_pose()
                print(
                    f"[Tiago] Ground-truth pose active: "
                    f"({wx:.3f}, {wy:.3f}, {math.degrees(wth):.1f}°) "
                    f"[no spawn parameter needed]"
                )
            else:
                print("[Tiago] WARNING: GT not yet received — odometry active. "
                      "Check the gz-ros2 bridge is running for "
                      f"world '{world_name}'.")
        else:
            print("[Tiago] WARNING: No pose received within "
                  f"{pose_timeout:.0f} s. Navigation may fail.")

        # SdfSurfaceResolver: parse SDF once to get top-surface z offsets automatically.
        self._surface = SdfSurfaceResolver(sdf_path, verbose=False) if sdf_path else None

        self._adapter.get_logger().info(
            f"[Tiago] Agent '{robot_name}' ready. "
            f"robot='{robot_name}'  world='{world_name}'"
        )

        # Tiago can get it automatically, but it gets the center of the objects.
        # Here I define some specific places just to look better in the video.
        # Selected from DEFAULT_SCENES (locations.py) by world_name — falls
        # back to the "backyard" layout if world_name is unset or unrecognized.
        self.scene = get_scene(world_name)
    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick_and_place_objects(
        self,
        object_pairs:   List[ObjectPair],
        place_z_offset: float = 0.0,
    ) -> Tuple[bool, str]:
        """
        Execute a sequence of pick-and-place operations sequentially.

        Parameters
        ----------
        object_pairs : list of (pick, place)
            Each element is a 2-tuple where every item can be **either**:

            * a ``str``  — Gazebo model name, resolved automatically via
              ObjectToRobot (requires the gz-ros2 bridge to be running).
            * an ``(x, y, z)`` tuple — explicit world-frame coordinates used
              directly, bypassing Gazebo lookup.

            Mixed pairs are supported, e.g.::

                [
                    ("tomato_1", "bowl_1"),            # both by name
                    ("meat_1",   (0.5, 2.0, 0.85)),    # pick by name, place by XYZ
                    ((0.5, 1.2, 0.85), "plate_1"),     # pick by XYZ, place by name
                    ((0.5, 1.2, 0.85), (0.5, 2.0, 0.85)), # both by XYZ
                ]

        place_z_offset : float
            Manual z offset added to place targets that are resolved **by
            name** when no ``sdf_path`` was provided at init.  Ignored when
            coordinates are given directly or when SdfSurfaceResolver is
            active.

        Returns when all pairs have been attempted.  Per-pair success/failure
        is logged; the method does not raise on individual failures.
        """
        results = []

        for idx, (pick, place) in enumerate(object_pairs):
            pick_label  = pick  if isinstance(pick,  str) else _fmt(pick)
            place_label = place if isinstance(place, str) else _fmt(place)
            self._adapter.get_logger().info(
                f"[Tiago] Pair {idx+1}/{len(object_pairs)}: "
                f"pick={pick_label!r}  place={place_label!r}"
            )

            # ── Resolve pick world pose ──────────────────────────────
            if isinstance(pick, str):
                # Re-query per pair: pick object may have moved.
                pick_xyz = self._resolve_world_xyz(pick)
                if pick_xyz is None:
                    self._adapter.get_logger().error(
                        f"[Tiago] Skipping pair {idx+1}: "
                        f"cannot find '{pick}' in Gazebo."
                    )
                    continue
            else:
                pick_xyz = tuple(pick)   # explicit world-frame XYZ

            pick_name = pick if isinstance(pick, str) else None

            # ── Resolve place world pose ─────────────────────────────
            if isinstance(place, str):
                # Use SdfSurfaceResolver.place_xyz() when available — it returns
                # (x, y, world_pose.z + top_surface_offset).  Fall back to
                # manual place_z_offset when the resolver is absent.
                if self._surface is not None:
                    result = self._otr.get_pose(place)
                    if "error" in result:
                        self._adapter.get_logger().error(
                            f"[Tiago] Skipping pair {idx+1}: "
                            f"cannot find '{place}' in Gazebo."
                        )
                        continue
                    place_xyz = self._surface.place_xyz(result, place)
                    if place_xyz is None:
                        pos = result["world_pose"].position
                        place_xyz = (pos.x, pos.y, pos.z + place_z_offset)
                else:
                    place_xyz = self._resolve_surface_xyz(place, place_z_offset)
                if place_xyz is None:
                    self._adapter.get_logger().error(
                        f"[Tiago] Skipping pair {idx+1}: "
                        f"cannot find '{place}' in Gazebo."
                    )
                    continue
            else:
                place_xyz = tuple(place)   # explicit world-frame XYZ

            self._adapter.get_logger().info(
                f"[Tiago] Executing: pick={_fmt(pick_xyz)}  place={_fmt(place_xyz)}"
            )

            # ── Execute pick-and-place ───────────────────────────────
            self._goal_done_event.clear()

            accepted = self._adapter.pick_and_place(
                pick_xyz         = pick_xyz,
                place_xyz        = place_xyz,
                pick_object_name = pick_name,
            )

            if not accepted:
                # Should not happen (we wait for completion before next pair),
                # but guard anyway.
                self._adapter.get_logger().error(
                    f"[Tiago] Adapter busy — skipping pair {idx+1}."
                )
                continue

            # Block until adapter signals completion
            self._goal_done_event.wait()
            success, message = self._last_result
            results.append((success, message))

            if success:
                self._adapter.get_logger().info(
                    f"[Tiago] ✓ Pair {idx+1} complete: {message}"
                )
            else:
                self._adapter.get_logger().error(
                    f"[Tiago] ✗ Pair {idx+1} failed: {message}"
                )

        # Return overall success (True only if all pairs succeeded)
        all_ok = all(r[0] for r in results)
        summary = "; ".join(r[1] for r in results)
        
        return all_ok, summary  
    
    # ------------------------------------------------------------------
    # Navigation API
    # ------------------------------------------------------------------

    def navigate_to(
        self,
        target: Union[str, XYZ],
        timeout: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Drive the robot to a named location or explicit world-frame coordinates.
 
        Parameters
        ----------
        target : str or (x, y, z)
            * ``str`` — key in the ``locations`` dict (e.g. ``"DiningTable"``),
              or a Gazebo model name resolved via ObjectToRobot.
              Named locations in the dict are tried first; if not found there
              the name is looked up in Gazebo via OTR.
            * ``(x, y, z)`` tuple — explicit world-frame target.  The z value
              is passed to the navigator as ``target_z`` (used for approach
              pose height; navigation is planar).
 
        timeout : float, optional
            Override the instance-level ``nav_timeout``.
 
        Returns
        -------
        (success, message)
        """
        # ── Resolve target XYZ ──────────────────────────────────────────
        if isinstance(target, str):
            if target in self._locations:
                loc = self._locations[target]
                target_xyz = (loc["x"], loc["y"], 0.0)
                self._adapter.get_logger().info(
                    f"[Tiago] navigate_to: '{target}' → "
                    f"({loc['x']:.3f}, {loc['y']:.3f}) [locations dict]"
                )
            else:
                # Fall back to Gazebo pose lookup
                result = self._otr.get_pose(target)
                if "error" in result:
                    return False, f"Cannot resolve navigate target '{target}': {result['error']}"
                pos = result["world_pose"].position
                target_xyz = (pos.x, pos.y, pos.z)
                self._adapter.get_logger().info(
                    f"[Tiago] navigate_to: '{target}' → "
                    f"({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) [Gazebo OTR]"
                )
        else:
            target_xyz = tuple(target)
            self._adapter.get_logger().info(
                f"[Tiago] navigate_to: XYZ={_fmt(target_xyz)}"
            )
 
        tx, ty, tz = target_xyz
 
        # ── Dispatch via TiagoNavigator ──────────────────────────────────
        self._nav_done_event.clear()

        def _on_nav_done(success: bool, message: str) -> None:
            self._nav_result = (success, message)
            self._nav_done_event.set()

        accepted = self._navigator.navigate_to(tx, ty, _on_nav_done, target_z=tz)
        if not accepted:
            msg = "[Tiago] Navigator rejected goal (already busy)."
            self._adapter.get_logger().error(msg)
            return False, msg

        deadline = timeout if timeout is not None else self._nav_timeout
        finished = self._nav_done_event.wait(timeout=deadline)
        if not finished:
            self._navigator.cancel()
            msg = f"[Tiago] Navigation timed out after {deadline:.0f} s."
            self._adapter.get_logger().error(msg)
            return False, msg

        success, message = self._nav_result
        logger = self._adapter.get_logger()
        if success:
            logger.info(f"[Tiago] navigate_to result: {message}")
        else:
            logger.error(f"[Tiago] navigate_to result: {message}")

        return success, message

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def create_tasks_sequence(self, task_type: str, group: str, target_place: str, use_names: bool = False) -> List[dict]:
        sequence = []
        if task_type == "pick_place":
            for food, placements in self.scene[group]["placements"].items():
                if use_names:
                    task = {
                        "action":      "pick_and_place",
                        "pick_name":   food,
                        "place_name":  target_place,
                    }
                else:
                    task = {
                        "action":     "pick_and_place",
                        "pick_name":  food,
                        "place_xyz":  placements[target_place],
                    }
                sequence.append(task)
        else:
            print(f"Task type '{task_type}' not recognized.")
        return sequence

    def define_subtask(self, action: str, use_names: bool = False) -> List[dict]:
        sequence = None

        if action == "ServeDrinks":
            place = "table_right" if self._world_name == "backyard" else "table_right_1"
            sequence = self.create_tasks_sequence(
                "pick_place", "drinks",
                "place_guests" if not use_names else place,
                use_names=use_names,
            )
            sequence.append({"action": "navigate_to", "target_xyz": (0, 0, 0)})
            print("[Tiago] Serve drinks sequence created.")
        elif action == "NavigateTo":
            # Generic navigate: caller must pass the target as the task string
            # or use navigate_to() directly.  This branch handles dict tasks of
            # the form {"action": "NavigateTo", "target": "House"}.
            print("[Tiago] NavigateTo: use navigate_to() directly or pass a dict task.")
            sequence = []
        elif action == "PickDishes":
            # Drive to the House location first, then simulate collecting dishes.
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_THE_DISHES})
            print("[Tiago] Pick dishes sequence created.")
        elif action == "DoTheDishes":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_DOING_THE_DISHES})
            print("[Tiago] Do the dishes sequence created.")
        elif action == "PickRice":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICKING_RICE})
            print("[Tiago] Pick rice sequence created.")
        elif action == "CookRice":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_COOKING_RICE})
            print("[Tiago] Cook rice sequence created.")
        elif action == "ServeMainDish":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": 5})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 5})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 5})
            print("[Tiago] Serve main dish sequence created.")
        elif action == "ServeSides":
            target = "VegetablePrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": 5})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 5})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 5})
            print("[Tiago] Serve sides sequence created.")
        elif action == "ServeSalad":
            target = "SaladPrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": 5})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 5})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 5})
            print("[Tiago] Serve salad sequence created.")
        elif action == "ServeGrilledFood":
            target = "FoodGrillPrepTable" if self._world_name == "backyard_b" else "GrillPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": 5})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 5})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 5})
            print("[Tiago] Serve grilled food sequence created.")

        elif action == "ServeGrilledSeafood":
            sequence = [
                {"action": "navigate_to", "target_name": "FishGrillPrepTable"}
            ]
            sequence.append({"action": "sleep", "time": 5})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 5})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 5})
            print("[Tiago] Serve grilled seafood sequence created.")

        elif action == "WelcomeGuests":
            sequence = [
                {"action": "navigate_to", "target_name": "MainEntrance"}
            ]
            sequence.append({"action": "sleep", "time": 120})
            sequence.append({"action": "navigate_to", "target_name": "GroupOfGuests_1"})
            sequence.append({"action": "sleep", "time": 15})
            sequence.append({"action": "navigate_to", "target_name": "GroupOfGuests_2"})
            sequence.append({"action": "sleep", "time": 15})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 5})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 5})
            print("[Tiago] Welcome guests sequence created.")
        elif action == "PickDishes":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_THE_DISHES})
            print("[Tiago] Pick dishes sequence created.")
        elif action == "DoTheDishes":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_DOING_THE_DISHES})
            print("[Tiago] Do the dishes sequence created.")
        elif action == "PutTheDishesAway":
            sequence = [
                {"action": "navigate_to", "target_name": "House"}
            ]
            sequence.append({"action": "sleep", "time": TIME_PUTTING_AWAY_DISHES})
            print("[Tiago] Put the dishes away sequence created.")
        elif action == "Host":
            sequence = [{"action": "sleep", "time": 20}]
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "GrillPrepTable"})
                sequence.append({"action": "sleep", "time": 25})
                sequence.append({"action": "navigate_to", "target_name": "MainPrepTable"})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "FoodGrillPrepTable"})
                sequence.append({"action": "sleep", "time": 25})
                sequence.append({"action": "navigate_to", "target_name": "SaladPrepTable"})
                sequence.append({"action": "sleep", "time": 15})
                sequence.append({"action": "navigate_to", "target_name": "VegetablePrepTable"})
            sequence.append({"action": "sleep", "time": 20})
            sequence.append({"action": "navigate_to", "target_name": "GroupOfGuests_2"})
            sequence.append({"action": "sleep", "time": 120})
            sequence.append({"action": "navigate_to", "target_name": "DrinksTable"})
            sequence.append({"action": "sleep", "time": 20})
            sequence.append({"action": "navigate_to", "target_name": "GroupOfGuests_1"})
            sequence.append({"action": "sleep", "time": 120})
            if self._world_name == "backyard":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable"})
                sequence.append({"action": "sleep", "time": 60})
            elif self._world_name == "backyard_b":
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_1"})
                sequence.append({"action": "sleep", "time": 30})
                sequence.append({"action": "navigate_to", "target_name": "DiningTable_2"})
                sequence.append({"action": "sleep", "time": 30})
            
            print("[Tiago] Host sequence created.")

        # Vegetables
        elif action == "PickVegetables":
            target = "VegetablePrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_VEGETABLE})
            print("[Tiago] Pick vegetables sequence created.")
        elif action == "ChopVegetables":
            target = "VegetablePrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_CHOP_VEGETABLES})
            print("[Tiago] Chopping vegetables... (not implemented)")
        elif action == "PickChoppedVegetables":
            target = "VegetablePrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_CHOPPED_VEGETABLE})
            print("[Tiago] Pick chopped vegetables sequence created.")

        # Salad
        elif action == "PickSaladIngredients":
            target = "SaladPrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_SALAD})
            print("[Tiago] Pick salad ingredients sequence created.")
        elif action == "ChopSaladIngredients":
            target = "SaladPrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_CHOP_SALAD_INGREDIENTS})
            print("[Tiago] Chopping salad ingredients... (not implemented)")
        elif action == "PickChoppedSaladIngredients":
            target = "SaladPrepTable" if self._world_name == "backyard_b" else "MainPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_CHOPPED_SALAD})
            print("[Tiago] Pick chopped salad ingredients sequence created.")

        # Meat + Garlic Bread
        elif action == "PickFoodIngredientsGrill":
            target = "FoodGrillPrepTable" if self._world_name == "backyard_b" else "GrillPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_FOOD_GRILL})
            print("[Tiago] Pick food ingredients for grill sequence created.")
        elif action == "GrillFood":
            target = "FoodGrillPrepTable" if self._world_name == "backyard_b" else "GrillPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_GRILL_FOOD})
            print("[Tiago] Grilling food... (not implemented)")
        elif action == "PickGrilledFood":
            target = "FoodGrillPrepTable" if self._world_name == "backyard_b" else "GrillPrepTable"
            sequence = [
                {"action": "navigate_to", "target_name": target}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_GRILLED_FOOD})
            print("[Tiago] Pick grilled food sequence created.")
        # Fish + Squid
        elif action == "PickSeafoodIngredientsGrill": 
            sequence = [
                {"action": "navigate_to", "target_name": "FishGrillPrepTable"}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_FOOD_GRILL})
            print("[Tiago] Pick seafood ingredients for grill sequence created.")
        elif action == "GrillSeafood":
            sequence = [
                {"action": "navigate_to", "target_name": "FishGrillPrepTable"}
            ]
            sequence.append({"action": "sleep", "time": TIME_GRILL_FOOD})
            print("[Tiago] Grilling seafood... (not implemented)")
        elif action == "PickGrilledSeafood":
            sequence = [
                {"action": "navigate_to", "target_name": "FishGrillPrepTable"}
            ]
            sequence.append({"action": "sleep", "time": TIME_PICK_GRILLED_FOOD})
            print("[Tiago] Pick grilled seafood sequence created.")
        elif action == "Reception":
            print("[Tiago] Checking subgoal Reception task completeness... (not implemented)")
            sequence = [{"action": "sleep", "time": 25}]
        elif action == "PrepareDrinks":
            print("[Tiago] Checking subgoal Preparing drinks completeness... (not implemented)")
            sequence = [{"action": "sleep", "time": 25}]
        elif action == "SuperviseParty":
            print("[Tiago] Checking subgoal Supervising party completeness... (not implemented)")
            sequence = [{"action": "sleep", "time": 25}]
        elif action == "Clean":
            print("[Tiago] Checking subgoal Cleaning completeness... (not implemented)")
            sequence = [{"action": "sleep", "time": 25}]
        elif action == "PrepareFood":
            print("[Tiago] Checking subgoal Preparing food completeness... (not implemented)")
            sequence = [{"action": "sleep", "time": 25}]
        else:
            print(f"[Tiago] Action '{action}' not recognized.")

        return sequence
    
    # ------------------------------------------------------------------
    # Agent interface (called by base class task dispatch)
    # ------------------------------------------------------------------

    def _execute_task_specific(self, task: dict) -> None:
        sequence = self.define_subtask(task)
        if sequence is None:
            print(f"[Tiago] No sequence defined for action: '{task}'")
            return
    
        for subtask in sequence:
            action = subtask["action"]
            if action == "pick_and_place":
                # Each side is either a name (str) or an explicit XYZ tuple.
                pick  = subtask.get("pick_name")  or subtask.get("pick_xyz")
                place = subtask.get("place_name") or subtask.get("place_xyz")
                ok, msg = self.pick_and_place_objects([(pick, place)])
                log = self._adapter.get_logger() if self._adapter else None
                if log:
                    if ok:
                        log.info(f"[Tiago] task result: {msg}")
                    else:
                        log.error(f"[Tiago] task result: {msg}")
            elif action == "navigate_to":
                # target may be a location name (str), Gazebo model name, or XYZ tuple.
                target  = subtask.get("target_name") or subtask.get("target_xyz")
                timeout = subtask.get("timeout")      # optional per-subtask override
                if target is None:
                    print("[Tiago] navigate_to subtask missing 'target_name' or 'target_xyz'.")
                    continue
                ok, msg = self.navigate_to(target, timeout=timeout)
                log = self._adapter.get_logger() if self._adapter else None
                if log:
                    if ok:
                        log.info(f"[Tiago] navigate result: {msg}")
                    else:
                        log.error(f"[Tiago] navigate result: {msg}")
            elif action == "sleep":
                time.sleep(subtask.get("time"))
            else:
                print(f"[Tiago] Unknown task action: '{action}'")

    def shutdown(self) -> None:
        """Cleanly shut down ROS2 executor and nodes."""
        self._adapter.get_logger().info("[Tiago] Shutting down ...")
        self._executor.shutdown(timeout_sec=3.0)
        for node in (self._adapter._gripper, self._adapter, self._navigator):
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
        self._adapter.get_logger().info("[Tiago] Shutdown complete.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _resolve_world_xyz(
        self,
        model_name: str,
        z_offset:   float = 0.0,
    ) -> Optional[XYZ]:
        """
        Return world-frame (x, y, z) of a Gazebo model for pick targeting.

        Uses OTR world_pose.z directly — the object's geometric centre height.
        This is the correct pick target: the gripper descends to centre height
        so the fingers close around the widest part of the object.

        Do NOT subtract top_offset here. The bottom of the object is at the
        table surface (z=0.850 for the prep table) — the gripper cannot
        physically reach that z without hitting the table.
        """
        result = self._otr.get_pose(model_name)
        if "error" in result:
            return None
        pos = result["world_pose"].position
        return (pos.x, pos.y, pos.z + z_offset)

    def _resolve_surface_xyz(
        self,
        model_name:    str,
        manual_offset: float = 0.0,
    ) -> Optional[XYZ]:
        """
        Return world-frame (x, y, top_surface_z) for a Gazebo model.

        If an SdfSurfaceResolver was loaded at init, the z offset to the top
        surface is computed automatically from the SDF geometry.
        Otherwise, manual_offset is added to the raw world_pose.z.
        """
        result = self._otr.get_pose(model_name)
        if "error" in result:
            return None
        pos = result["world_pose"].position
        if self._surface is not None:
            offset = self._surface.top_surface_offset(model_name)
        else:
            offset = manual_offset
        return (pos.x, pos.y, pos.z + offset)

    def _on_result(self, success: bool, message: str) -> None:
        """Called by TiagoAdapter when a pick-and-place cycle finishes."""
        self._last_result = (success, message)
        self._goal_done_event.set()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fmt(xyz: XYZ) -> str:
    return f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"