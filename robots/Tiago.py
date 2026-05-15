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
    tiago.pick_and_place_objects([
        ("tomato_1", "bowl_1"),
        ("meat_1",   "plate_1"),
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
from robot_common.object_world_to_robot import ObjectToRobot
from robot_common.sdf_surface_resolver import SdfSurfaceResolver
from utils import ROSUtils

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ        = Tuple[float, float, float]
ObjectPair = Tuple[str, str]   # (pick_model_name, place_model_name)


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
    """

    def __init__(
        self,
        robot_name:   str,
        world_name:   str,
        skill_weights,
        contexts,
        role,
        teamsize:     int,
        use_sim: bool,
        arm_base_z:   Optional[float] = None,
        pose_timeout: float           = 15.0,
        table_standoff  = 0.10,
    ) -> None:
        constraints = {
            "can_move":               True,
            "can_manipulate":         True,
            "max_size_object_cm":     10,
            "max_payload_kg":         3,
            "max_reach_cm":           85,
            "can_transport_objects":  True,
            "workspace":              "global",
        }
        super().__init__(robot_name, constraints, skill_weights, contexts, role, teamsize)

        if use_sim:
            mode = RobotMode.SIMULATION
        else:
            mode = RobotMode.REAL_WORLD

        self._robot_name = robot_name
        self._world_name = world_name

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
        self._surface = SdfSurfaceResolver(sdf_path, verbose=True) if sdf_path else None

        self._adapter.get_logger().info(
            f"[Tiago] Agent '{robot_name}' ready. "
            f"robot='{robot_name}'  world='{world_name}'"
        )

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
        object_pairs : list of (pick_name, place_name)
            Gazebo model names.  E.g.:
                [("tomato_1", "bowl_1"), ("meat_1", "plate_1")]
            Each pair is executed in order; the method blocks until all
            pairs are done or a fatal error occurs.

        place_z_offset : float
            Manual z offset added to every place target.  Only used when
            sdf_path was not provided at init.  When sdf_path was provided,
            offsets are computed automatically per model and this is ignored.

        Returns when all pairs have been attempted.  Per-pair success/failure
        is logged; the method does not raise on individual failures.
        """
        results = []

        for idx, (pick_name, place_name) in enumerate(object_pairs):
            self._adapter.get_logger().info(
                f"[Tiago] Pair {idx+1}/{len(object_pairs)}: "
                f"pick='{pick_name}'  place='{place_name}'"
            )

            # ── Resolve pick world pose ──────────────────────────────
            # Re-query per pair: the pick object may have moved (e.g. a
            # previous robot already picked it) and static place objects
            # don't change, so per-pair queries are always safe.
            pick_xyz = self._resolve_world_xyz(pick_name)
            if pick_xyz is None:
                self._adapter.get_logger().error(
                    f"[Tiago] Skipping pair {idx+1}: "
                    f"cannot find '{pick_name}' in Gazebo."
                )
                continue

            # ── Resolve place world pose ─────────────────────────────
            # Use SdfSurfaceResolver.place_xyz() directly — same as the test
            # script.  This returns (x, y, world_pose.z + top_surface_offset),
            # i.e. the z at which a placed object rests on the destination surface.
            # Fall back to manual place_z_offset when the resolver is absent.
            if self._surface is not None:
                result = self._otr.get_pose(place_name)
                if "error" in result:
                    self._adapter.get_logger().error(
                        f"[Tiago] Skipping pair {idx+1}: "
                        f"cannot find '{place_name}' in Gazebo."
                    )
                    continue
                place_xyz = self._surface.place_xyz(result, place_name)
                if place_xyz is None:
                    pos = result["world_pose"].position
                    place_xyz = (pos.x, pos.y, pos.z + place_z_offset)
            else:
                place_xyz = self._resolve_surface_xyz(place_name, place_z_offset)
            if place_xyz is None:
                self._adapter.get_logger().error(
                    f"[Tiago] Skipping pair {idx+1}: "
                    f"cannot find '{place_name}' in Gazebo."
                )
                continue

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
    # Agent interface (called by base class task dispatch)
    # ------------------------------------------------------------------

    def _execute_task_specific(self, task: dict) -> None:
        """
        Entry point for task-dispatch from the Agent base class.

        Expected task format:
            {
                "action":  "pick_and_place",
                "objects": [["pick_name", "place_name"], ...],
                "place_z_offset": 0.0   # optional
            }
        """
        action = task.get("action")
        if action == "pick_and_place":
            pairs  = [tuple(p) for p in task.get("objects", [])]
            offset = task.get("place_z_offset", 0.0)
            self.pick_and_place_objects(pairs, place_z_offset=offset)
        else:
            self._adapter.get_logger().warn(
                f"[Tiago] Unknown task action: '{action}'"
            )

    def start_adapter(self) -> None:
        """No-op — adapter starts automatically in __init__."""
        pass

    def shutdown(self) -> None:
        """Cleanly shut down ROS2 executor and nodes."""
        self._adapter.get_logger().info("[Tiago] Shutting down ...")
        self._executor.shutdown(timeout_sec=3.0)
        for node in (self._adapter._gripper, self._adapter):
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