"""
Tiago
=====
High-level agent for a PAL Tiago mobile manipulator.

Accepts a list of (pick_object, place_object) name pairs and executes them
sequentially.  Each object name is a Gazebo model name; world-frame positions
are resolved automatically via ObjectToRobot (gz-ros2-bridge).

Architecture
------------
                    ┌─────────────────────────────────────┐
                    │  Tiago (Agent subclass)              │
                    │                                     │
                    │  ObjectToRobot ──► world_pose XYZ   │
                    │       │                             │
                    │       ▼                             │
                    │  TiagoAdapter  (world-frame mode)   │
                    │    ├─ TiagoPickPlacePlanner          │
                    │    │    └─ nav pose computation      │
                    │    ├─ P-controller navigation        │
                    │    └─ TiagoGripperAdapter            │
                    └─────────────────────────────────────┘

TiagoAdapter runs in world-frame mode (robot_frame=False), meaning it
receives world-frame XYZ and handles navigation internally — no separate
TiagoNavigator or TiagoMissionController is needed here.

ObjectToRobot shares the adapter's ROS2 node so all subscriptions live on
the same node and are served by the same MultiThreadedExecutor.

Dependency note
---------------
object_to_robot.py must have rclpy.spin_once() replaced with time.sleep()
in _wait_for_poses() and _resolve_base_frame() — the executor already spins
the node so spin_once() is a no-op when called from a separate thread.

Usage
-----
    tiago = Tiago(
        id          = "tiago1",
        robot_name  = "tiago_robot1",
        world_name  = "backyard",          # matches gz-ros2-bridge world name
        skill_weights = ...,
        contexts    = ...,
        role        = ...,
        teamsize    = 1,
    )
    tiago.pick_and_place_objects([
        ("tomato_1", "bowl_1"),
        ("meat_1",   "plate_1"),
    ])

Place on a static surface (e.g. a table whose model centre ≠ surface):
    tiago.pick_and_place_objects(
        [("tomato_1", "prep")],
        place_z_offset = 0.05,   # table model centre is 0.05 m below surface
    )
"""
from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple, Union

import rclpy
import rclpy.parameter
from rclpy.executors import MultiThreadedExecutor

from src.entities.Agent import Agent
from tiago_adapters.TiagoAdapter import TiagoAdapter, RobotMode
from robot_common.object_to_robot import ObjectToRobot
from robot_common.sdf_surface_resolver import SdfSurfaceResolver

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ        = Tuple[float, float, float]
ObjectPair = Tuple[str, str]   # (pick_model_name, place_model_name)


# ---------------------------------------------------------------------------
# Tiago agent
# ---------------------------------------------------------------------------

class Tiago(Agent):
    """
    Parameters
    ----------
    id : str
        Agent identifier (e.g. "tiago1").
    robot_name : str
        Gazebo / ROS2 namespace for this robot (e.g. "tiago_robot1").
    world_name : str
        Gazebo world name that matches the gz-ros2-bridge command, e.g. "backyard".
        Must match <world name="…"> in the .sdf AND the bridge topic
        /world/<world_name>/pose/info.
    skill_weights, contexts, role, teamsize :
        Passed through to the Agent base class.
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83.
    sdf_path : str, optional
        Absolute path to the world .sdf file.  When provided, top-surface z
        offsets are computed automatically from the model geometry — no manual
        place_z_offset needed.  If omitted, raw model origin z is used.
    pose_timeout : float
        Seconds to wait for the first pose/info message on startup. Default 15.
    """

    def __init__(
        self,
        id:           str,
        robot_name:   str,
        world_name:   str,
        skill_weights,
        contexts,
        role,
        teamsize:     int,
        sdf_path:     Optional[str] = None,
        arm_base_z:   float         = 0.83,
        pose_timeout: float         = 15.0,
    ) -> None:
        constraints = {
            "can_move":               True,
            "can_manipulate":         True,
            "max_size_object_cm":     10,
            "max_payload_kg":         3,
            "max_reach_cm":           85,
            "can_transport_objects":  True,
            "workspace":              "mobile",
        }
        super().__init__(id, constraints, skill_weights, contexts, role, teamsize)

        self._robot_name = robot_name
        self._world_name = world_name

        # Event + storage for adapter completion signal
        self._goal_done_event = threading.Event()
        self._last_result: Tuple[bool, str] = (False, "No goal sent yet.")

        # ── ROS2 initialisation ───────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        # TiagoAdapter in world-frame mode:
        # receives world-frame XYZ, computes nav poses internally, drives base.
        self._adapter = TiagoAdapter(
            robot_name      = robot_name,
            mode            = RobotMode.SIMULATION,
            result_callback = self._on_result,
            robot_frame     = False,   # world-frame mode: adapter handles navigation
            arm_base_z      = arm_base_z,
        )

        # ObjectToRobot shares the adapter node so both live on the same executor.
        # It subscribes to /world/<world_name>/pose/info via the gz-ros2-bridge.
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

        # SdfSurfaceResolver: parse SDF once to get top-surface z offsets automatically.
        self._surface = SdfSurfaceResolver(sdf_path, verbose=True) if sdf_path else None

        self._adapter.get_logger().info(
            f"[Tiago] Agent '{id}' ready. "
            f"robot='{robot_name}'  world='{world_name}'"
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick_and_place_objects(
        self,
        object_pairs:   List[ObjectPair],
        place_z_offset: float = 0.0,
    ) -> None:
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
            # Automatic surface offset from SDF geometry (if sdf_path provided),
            # otherwise fall back to manual place_z_offset.
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
                pick_xyz  = pick_xyz,
                place_xyz = place_xyz,
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

            if success:
                self._adapter.get_logger().info(
                    f"[Tiago] ✓ Pair {idx+1} complete: {message}"
                )
            else:
                self._adapter.get_logger().error(
                    f"[Tiago] ✗ Pair {idx+1} failed: {message}"
                )

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

    def shutdown_adapter(self) -> None:
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
        """Return world-frame (x, y, z) of a Gazebo model, or None on error."""
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