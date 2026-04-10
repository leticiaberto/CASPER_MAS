"""
TiagoMissionController
======================
High-level orchestrator for Tiago pick-and-place missions.

Ties together:
  - ObjectToRobot   — resolves Gazebo object names to world / robot-frame poses
  - TiagoNavigator  — drives the mobile base to a reachable position
  - TiagoAdapter    — executes arm + gripper pick-and-place

Single entry point
------------------
    controller.pick_and_place(
        pick  = "tomato_1",          # Gazebo model name
        place = "bowl_1",            # Gazebo model name  OR  (x, y, z) world coords
    )

The controller handles the full sequence automatically:
  1. Resolve pick object → world-frame XYZ
  2. Navigate to the nearest pose from which pick is reachable
  3. Re-query pick object in the (now-fresh) robot frame
  4. Resolve place target → world-frame XYZ
  5. Navigate to the nearest pose from which place is reachable
     (skipped if place is already reachable from the pick pose)
  6. Re-query place target in the (now-fresh) robot frame
  7. Execute pick-and-place with robot-frame coords (no further navigation)

Usage
-----
    import threading
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from tiago_mission_controller import TiagoMissionController

    rclpy.init()

    done   = threading.Event()
    result = []

    def on_done(success, message):
        result.append((success, message))
        done.set()

    controller = TiagoMissionController(
        robot_name  = "tiago_robot1",
        world_name  = "backyard_world",
        result_callback = on_done,
    )

    executor = MultiThreadedExecutor()
    for node in controller.nodes:
        executor.add_node(node)

    import threading
    threading.Thread(target=executor.spin, daemon=True).start()

    controller.pick_and_place(pick="tomato_1", place="bowl_1")
    done.wait(timeout=300)
    print(result[0])

Standalone
----------
    python3 tiago_mission_controller.py \\
        --robot tiago_robot1 --world backyard_world \\
        --pick tomato_1 --place bowl_1
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from typing import Callable, Optional, Tuple, Union

import rclpy
import rclpy.parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_common import ObjectToRobot
from tiago_navigator import TiagoNavigator
from TiagoAdapter import TiagoAdapter, RobotMode

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ            = Tuple[float, float, float]
ResultCallback = Callable[[bool, str], None]
PlaceTarget    = Union[str, XYZ]   # Gazebo model name OR world-frame (x, y, z)


# ---------------------------------------------------------------------------
# TiagoMissionController
# ---------------------------------------------------------------------------

class TiagoMissionController:
    """
    Parameters
    ----------
    robot_name : str
        Gazebo model name of the robot (e.g. "tiago_robot1").
    world_name : str
        Gazebo world name as declared in the .sdf (<world name="…">).
    result_callback : ResultCallback
        Called when a mission finishes: callback(success: bool, message: str).
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83.
    approach_height : float
        Pre-grasp / pre-place vertical offset (m). Default 0.12.
    """

    def __init__(
        self,
        robot_name:      str,
        world_name:      str,
        result_callback: ResultCallback,
        arm_base_z:      float = 0.83,
        approach_height: float = 0.12,
    ) -> None:
        self._robot_name     = robot_name
        self._world_name     = world_name
        self._result_cb      = result_callback
        self._arm_base_z     = arm_base_z
        self._approach_height = approach_height

        self._busy      = False
        self._busy_lock = threading.Lock()

        # ── ROS nodes ─────────────────────────────────────────────────
        # ObjectToRobot needs a live Node to spin — use a dedicated one.
        self._otr_node = _OTRNode(robot_name=robot_name, world_name=world_name)

        self._navigator = TiagoNavigator(
            robot_name = robot_name,
            arm_base_z = arm_base_z,
        )

        self._adapter = TiagoAdapter(
            robot_name      = robot_name,
            mode            = RobotMode.SIMULATION,
            result_callback = self._on_adapter_done,
            robot_frame     = True,   # we supply pre-transformed coords
            arm_base_z      = arm_base_z,
        )

        # Event + storage for adapter completion
        self._adapter_done  = threading.Event()
        self._adapter_ok    = False
        self._adapter_msg   = ""

    # ------------------------------------------------------------------
    # Node list  (add all to the executor)
    # ------------------------------------------------------------------

    @property
    def nodes(self):
        """All ROS2 nodes owned by this controller."""
        return [
            self._otr_node,
            self._navigator,
            self._adapter,
            self._adapter._gripper,
        ]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick_and_place(
        self,
        pick:  str,
        place: PlaceTarget,
    ) -> bool:
        """
        Execute a full pick-and-place mission.

        Parameters
        ----------
        pick : str
            Gazebo model name of the object to pick (e.g. "tomato_1").
        place : str or (x, y, z)
            Destination — either a Gazebo model name (the controller will
            place the item at / above that object) or a world-frame
            (x, y, z) tuple giving the exact drop position.

        Non-blocking.  result_callback fires on completion.
        Returns False if already busy.
        """
        with self._busy_lock:
            if self._busy:
                self._log("warn", "Rejected — already busy.")
                return False
            self._busy = True

        threading.Thread(
            target = self._run_mission,
            args   = (pick, place),
            daemon = True,
        ).start()
        return True

    def cancel(self) -> None:
        """Request cancellation of the current mission (best-effort)."""
        self._navigator.cancel()
        self._adapter.cancel()

    @property
    def is_busy(self) -> bool:
        with self._busy_lock:
            return self._busy

    # ------------------------------------------------------------------
    # Mission sequence
    # ------------------------------------------------------------------

    def _run_mission(self, pick_name: str, place: PlaceTarget) -> None:
        try:
            self._execute_mission(pick_name, place)
        except Exception as exc:
            self._finish(False, f"Unexpected error: {exc}")

    def _execute_mission(self, pick_name: str, place: PlaceTarget) -> None:
        self._log("info", f"Mission start: pick='{pick_name}' place={place!r}")

        # ── Step 1: get pick world pose ──────────────────────────────
        pick_world = self._get_world_xyz(pick_name)
        if pick_world is None:
            self._finish(False, f"Object '{pick_name}' not found in Gazebo.")
            return
        self._log("info", f"Pick world pos: {_fmt(pick_world)}")

        # ── Step 2: navigate near pick object ───────────────────────
        self._log("info", "Navigating to pick position ...")
        ok, msg = self._navigator.drive_to_reach_blocking(pick_world)
        if not ok:
            self._finish(False, f"Navigation to pick failed: {msg}")
            return
        self._log("info", f"At pick position. ({msg})")

        # ── Step 3: re-query pick in fresh robot frame ───────────────
        pick_robot = self._get_robot_xyz(pick_name)
        if pick_robot is None:
            self._finish(False, f"Lost '{pick_name}' after navigation.")
            return
        self._log("info", f"Pick robot-frame: {_fmt(pick_robot)}")

        # ── Step 4: resolve place world pose ────────────────────────
        place_world = self._resolve_place_world(place)
        if place_world is None:
            self._finish(False, f"Cannot resolve place target: {place!r}")
            return
        self._log("info", f"Place world pos: {_fmt(place_world)}")

        # ── Step 5: navigate near place if needed ───────────────────
        nav_pose_now = self._navigator.get_pose()
        if not self._navigator._is_reachable(place_world, nav_pose_now):
            self._log("info", "Place not reachable from pick pose — navigating ...")
            ok, msg = self._navigator.drive_to_reach_blocking(place_world)
            if not ok:
                self._finish(False, f"Navigation to place failed: {msg}")
                return
            self._log("info", f"At place position. ({msg})")
        else:
            self._log("info", "Place reachable from pick pose — no extra navigation.")

        # ── Step 6: re-query place in fresh robot frame ──────────────
        place_robot = self._get_robot_xyz_for_place(place)
        if place_robot is None:
            self._finish(False, f"Cannot get place robot-frame coords for: {place!r}")
            return
        self._log("info", f"Place robot-frame: {_fmt(place_robot)}")

        # ── Step 7: execute pick-and-place (arm + gripper only) ──────
        self._log("info", "Executing pick-and-place ...")
        self._adapter_done.clear()

        accepted = self._adapter.pick_and_place(
            pick_xyz  = pick_robot,
            place_xyz = place_robot,
        )
        if not accepted:
            self._finish(False, "Adapter rejected pick-and-place (busy?).")
            return

        # Wait for adapter to finish (no timeout — adapter has its own timeouts)
        self._adapter_done.wait()
        self._finish(self._adapter_ok, self._adapter_msg)

    # ------------------------------------------------------------------
    # Pose resolution helpers
    # ------------------------------------------------------------------

    def _get_world_xyz(self, object_name: str) -> Optional[XYZ]:
        """Get object position in world frame from ObjectToRobot."""
        result = self._otr_node.otr.get_pose(object_name)
        if "error" in result:
            self._log("error", f"OTR error for '{object_name}': {result['error']}")
            return None
        p = result["world_pose"].position
        return (p.x, p.y, p.z)

    def _get_robot_xyz(self, object_name: str) -> Optional[XYZ]:
        """Get object position in robot frame from ObjectToRobot (current snapshot)."""
        result = self._otr_node.otr.get_pose(object_name)
        if "error" in result:
            self._log("error", f"OTR error for '{object_name}': {result['error']}")
            return None
        p = result["robot_pose"].position
        return (p.x, p.y, p.z)

    def _resolve_place_world(self, place: PlaceTarget) -> Optional[XYZ]:
        """Resolve place target to world-frame XYZ."""
        if isinstance(place, (tuple, list)) and len(place) == 3:
            return tuple(place)
        if isinstance(place, str):
            return self._get_world_xyz(place)
        return None

    def _get_robot_xyz_for_place(self, place: PlaceTarget) -> Optional[XYZ]:
        """
        Resolve place target to robot-frame XYZ at the current robot pose.

        If place is a Gazebo model name, query OTR for the fresh robot-frame pos.
        If place is a world-frame (x, y, z), transform it using the navigator's
        current pose.
        """
        if isinstance(place, str):
            return self._get_robot_xyz(place)

        if isinstance(place, (tuple, list)) and len(place) == 3:
            # Transform world-frame xyz into robot frame manually.
            # robot_frame = R(-theta) * (world - robot_xy), z -= arm_base_z
            rx, ry, rtheta = self._navigator.get_pose()
            wx, wy, wz     = place
            dx    = wx - rx
            dy    = wy - ry
            cos_t = math.cos(rtheta)
            sin_t = math.sin(rtheta)
            # Note: arm_base_z subtracted here so the adapter sees arm-frame z
            # BUT adapter in robot_frame mode subtracts arm_base_z itself.
            # So we give it robot-BASE-frame z (not arm-frame z).
            return (
                 dx * cos_t + dy * sin_t,
                -dx * sin_t + dy * cos_t,
                wz,   # robot-base-frame z = world z (base_footprint is on the floor)
            )
        return None

    # ------------------------------------------------------------------
    # Adapter callback + finish
    # ------------------------------------------------------------------

    def _on_adapter_done(self, success: bool, message: str) -> None:
        self._adapter_ok  = success
        self._adapter_msg = message
        self._adapter_done.set()

    def _finish(self, success: bool, message: str) -> None:
        icon = "✓" if success else "✗"
        level = "info" if success else "error"
        self._log(level, f"{icon} Mission {'complete' if success else 'failed'}: {message}")
        with self._busy_lock:
            self._busy = False
        self._result_cb(success, message)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log(self, level: str, msg: str) -> None:
        logger = self._otr_node.get_logger()
        full   = f"[TiagoMissionController] {msg}"
        if level == "info":
            logger.info(full)
        elif level == "warn":
            logger.warning(full)
        else:
            logger.error(full)


# ---------------------------------------------------------------------------
# _OTRNode  — thin Node wrapper so ObjectToRobot can spin with the executor
# ---------------------------------------------------------------------------

class _OTRNode(Node):
    """Minimal Node that owns an ObjectToRobot instance."""

    def __init__(self, robot_name: str, world_name: str) -> None:
        super().__init__(
            f"otr_{robot_name}",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )
        self.otr = ObjectToRobot(
            node       = self,
            robot_name = robot_name,
            world_name = world_name,
            pose_timeout = 15.0,   # give the bridge more time on startup
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fmt(xyz: XYZ) -> str:
    return f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="TiagoMissionController standalone test.")
    p.add_argument("--robot", default="tiago_robot1",
                   help="Gazebo model name of the robot.")
    p.add_argument("--world", default="backyard_world",
                   help="Gazebo world name (from <world name=…> in the .sdf).")
    p.add_argument("--pick",  required=True,
                   help="Gazebo model name of the object to pick (e.g. tomato_1).")
    p.add_argument("--place", required=True, nargs="+",
                   help="Place target: Gazebo model name OR 'x y z' world coords.")
    p.add_argument("--timeout", type=float, default=600.0,
                   help="Max seconds to wait for mission completion.")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    # Parse place: 1 token = model name, 3 tokens = world xyz
    if len(args.place) == 1:
        place: PlaceTarget = args.place[0]
    elif len(args.place) == 3:
        place = tuple(float(v) for v in args.place)
    else:
        print("[standalone] --place must be a model name or 'x y z' (3 floats).")
        return

    rclpy.init()

    done_event = threading.Event()
    result_box: list = []

    def on_done(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    controller = TiagoMissionController(
        robot_name      = args.robot,
        world_name      = args.world,
        result_callback = on_done,
    )

    executor = MultiThreadedExecutor()
    for node in controller.nodes:
        executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        accepted = controller.pick_and_place(pick=args.pick, place=place)
        if not accepted:
            print("[standalone] Mission rejected — controller is busy.")
        else:
            print(f"[standalone] Mission accepted. Waiting up to {args.timeout}s ...")
            finished = done_event.wait(timeout=args.timeout)
            if not finished:
                print("[standalone] ✗ Timed out.")
            else:
                success, message = result_box[0]
                print(f"[standalone] {'✓' if success else '✗'} {message}")

    except KeyboardInterrupt:
        print("\n[standalone] Interrupted.")
        controller.cancel()
        time.sleep(1.0)
    finally:
        time.sleep(1.0)
        executor.shutdown(timeout_sec=2.0)
        for node in controller.nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
