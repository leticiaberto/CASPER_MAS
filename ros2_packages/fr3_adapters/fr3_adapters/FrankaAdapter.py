"""
ROS2 adapter for the Franka Research 3 arm.

Supports two execution modes selected at instantiation:

    RobotMode.SIMULATION
        Uses ikpy (via PickPlacePlanner) for IK and sends
        FollowJointTrajectory actions to the Gazebo controllers.
        Gripper is controlled via GripperAdapter (also FollowJointTrajectory).

    RobotMode.PHYSICAL
        Uses Franky for motion and gripper control on the real FR3.
        PickPlacePlanner still runs to produce target_pose per step,
        but joint_trajectory is ignored — Franky handles path planning.

Both modes share the same public API, planning pipeline (PickPlacePlanner),
and result contract (PickPlanResult). Only the execution layer differs.

Usage
-----
    # Simulation
    adapter = FrankaAdapter(
        robot_name="fr3_robot1",
        mode=RobotMode.SIMULATION,
        result_callback=on_done,
    )

    # Physical robot
    adapter = FrankaAdapter(
        robot_name="fr3_robot1",
        mode=RobotMode.PHYSICAL,
        result_callback=on_done,
        franky_ip="172.16.0.2",   # robot IP required for physical mode
    )

    # Both modes — same call
    adapter.pick_and_place(pick_xyz=(0.5, 0.0, 0.3), place_xyz=(0.5, 0.4, 0.3))

Standalone usage
----------------
    python3 FrankaAdapter.py --mode sim --robot fr3_robot1 \\
        --pick 0.5 0.0 0.3 --place 0.5 0.4 0.3

    python3 FrankaAdapter.py --mode real --robot fr3_robot1 \\
        --franky-ip 172.16.0.2 \\
        --pick 0.5 0.0 0.3 --place 0.5 0.4 0.3

    python3 FrankaAdapter.py --mode sim --robot fr3_robot1 \\
        --base-height 1.03 --pick  0.4  0.25  1.04 --place 0.3 -0.25  1.05

"""

from __future__ import annotations

import argparse
import threading
import time
from enum import Enum
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from fr3_gripper_adapter import GripperAdapter
from fr3_pick_place_planner import PickPlacePlanner
from fr3_pick_place_result import PickPlanResult, PickPlaceStep, StepKind

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ = Tuple[float, float, float]
ResultCallback = Callable[[bool, str], None]


class RobotMode(str, Enum):
    SIMULATION = "sim"    # Gazebo + FollowJointTrajectory + GripperAdapter
    PHYSICAL   = "real"   # Real FR3 via Franky


# ---------------------------------------------------------------------------
# FrankaAdapter
# ---------------------------------------------------------------------------

class FrankaAdapter(Node):
    """
    Parameters
    ----------
    robot_name : str
        Namespace of the robot (e.g. "fr3_robot1").
    mode : RobotMode
        RobotMode.SIMULATION or RobotMode.PHYSICAL.
    result_callback : ResultCallback
        Called when a pick-and-place task finishes.
        Signature: callback(success: bool, message: str) -> None
    franky_ip : str, optional
        IP address of the real FR3 robot. Required when mode=PHYSICAL.
    franky_gripper_speed : float, optional
        Gripper speed for Franky (m/s). Default: 0.05.
    franky_gripper_force : float, optional
        Gripper grasp force for Franky (N). Default: 10.0.
    node_name : str, optional
        ROS2 node name.
    """

    def __init__(
        self,
        robot_name:           str,
        mode:                 RobotMode,
        result_callback:      ResultCallback,
        franky_ip:            Optional[str] = None,
        franky_gripper_speed: float         = 0.05,
        franky_gripper_force: float         = 10.0,
        base_height:          float         = 0.0,
        node_name:            Optional[str] = None,
    ) -> None:
        node_name = node_name or f"franka_adapter_{robot_name}"
        super().__init__(node_name)

        self._robot_name = robot_name
        self._mode       = mode
        self._result_callback = result_callback
        self._cancelled  = False

        # Task lock
        self._busy      = False
        self._busy_lock = threading.Lock()

        # Planner — used in both modes to compute waypoints / target poses
        self._planner = PickPlacePlanner(
            robot_name  = robot_name,
            base_height = base_height,
        )

        self.get_logger().info(
            f"[FrankaAdapter] Initialising in {mode.value.upper()} mode ..."
        )

        if mode == RobotMode.SIMULATION:
            self._init_simulation()
        else:
            self._init_physical(
                franky_ip,
                franky_gripper_speed,
                franky_gripper_force,
            )

        self.get_logger().info(
            f"[FrankaAdapter] Ready ({mode.value.upper()})."
        )

    # ------------------------------------------------------------------
    # Mode initialisation
    # ------------------------------------------------------------------

    def _init_simulation(self) -> None:
        """Set up ROS2 action clients for Gazebo simulation."""

        # Arm — FollowJointTrajectory
        arm_action = (
            f"/{self._robot_name}/arm_controller/follow_joint_trajectory"
        )
        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self.get_logger().info(
            f"[FrankaAdapter] Waiting for arm server '{arm_action}' ..."
        )
        self._arm_client.wait_for_server()
        self.get_logger().info("[FrankaAdapter] Arm server ready.")

        # Gripper
        self._gripper_done = threading.Event()
        self._gripper_ok   = False
        self._gripper = GripperAdapter(
            robot_name      = self._robot_name,
            result_callback = self._gripper_result_callback,
            node_name       = f"gripper_adapter_{self._robot_name}",
        )

        # Joint state subscriber — used to detect arm completion
        # instead of wall-clock timers (which break with use_sim_time: true)
        self._latest_joint_positions: dict = {}
        self._joint_state_lock = threading.Lock()
        self.create_subscription(
            JointState,
            f"/{self._robot_name}/joint_states",
            self._joint_state_callback,
            10,
        )

        # Physical-mode attributes set to None for safety
        self._franky_robot   = None
        self._franky_gripper = None

    def _init_physical(
        self,
        franky_ip:    Optional[str],
        gripper_speed: float,
        gripper_force: float,
    ) -> None:
        """Set up Franky for the real FR3 robot."""
        if franky_ip is None:
            raise ValueError(
                "[FrankaAdapter] franky_ip is required for PHYSICAL mode."
            )

        try:
            from franky import Robot, Gripper  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "[FrankaAdapter] Franky is not installed. "
                "Install it with: pip install franky-panda"
            ) from exc

        self.get_logger().info(
            f"[FrankaAdapter] Connecting to FR3 at {franky_ip} via Franky ..."
        )
        self._franky_robot   = Robot(franky_ip)
        self._franky_gripper = Gripper(franky_ip)
        self._franky_gripper_speed = gripper_speed
        self._franky_gripper_force = gripper_force
        self.get_logger().info("[FrankaAdapter] Franky connected.")

        # Simulation-mode attributes set to None for safety
        self._arm_client  = None
        self._gripper     = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick_and_place(
        self,
        pick_xyz:  XYZ,
        place_xyz: XYZ,
    ) -> bool:
        """
        Plan and execute a full pick-and-place cycle.

        Non-blocking: returns True if the task was accepted (started),
        False if the adapter is already busy.
        result_callback(success, message) is called when the task finishes.
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    "[FrankaAdapter] Rejected — a task is already in progress."
                )
                return False
            self._busy = True

        self.get_logger().info(
            f"[FrankaAdapter] pick={pick_xyz} → place={place_xyz} "
            f"({self._mode.value})"
        )

        # Planning is pure Python — no ROS2, no blocking
        plan = self._planner.plan(pick_xyz, place_xyz)
        if not plan.success:
            self.get_logger().error(
                f"[FrankaAdapter] Planning failed: {plan.message}"
            )
            with self._busy_lock:
                self._busy = False
            self._result_callback(False, plan.message)
            return True  # accepted, failed at planning stage

        threading.Thread(
            target=self._execute_plan,
            args=(plan,),
            daemon=True,
        ).start()
        return True

    def cancel(self) -> None:
        """Request cancellation of the current task (best-effort)."""
        with self._busy_lock:
            if not self._busy:
                self.get_logger().warn(
                    "[FrankaAdapter] cancel() called but no task is active."
                )
                return
            self._cancelled = True
        self.get_logger().info("[FrankaAdapter] Cancellation requested.")

    @property
    def is_busy(self) -> bool:
        """True while a pick-and-place task is executing."""
        with self._busy_lock:
            return self._busy

    @property
    def mode(self) -> RobotMode:
        """The execution mode this adapter was initialised with."""
        return self._mode

    # ------------------------------------------------------------------
    # Internal — plan execution (mode-agnostic)
    # ------------------------------------------------------------------

    def _execute_plan(self, plan: PickPlanResult) -> None:
        """Execute all steps sequentially, dispatching to the correct backend."""
        self._cancelled = False

        for i, step in enumerate(plan.steps):
            if self._cancelled:
                self._finish(False, "Task cancelled.")
                return

            self.get_logger().info(
                f"[FrankaAdapter] Step {i+1}/{len(plan.steps)}: {step.label}"
            )

            if step.kind == StepKind.MOVE:
                success = self._execute_move_step(step)
            elif step.kind in (StepKind.OPEN_GRIPPER, StepKind.CLOSE_GRIPPER):
                success = self._execute_gripper_step(step)
            else:
                success = False

            if not success:
                self._finish(False, f"Step '{step.label}' failed.")
                return

        self._finish(True, "Pick and place complete.")

    # ------------------------------------------------------------------
    # Internal — move step (mode-specific)
    # ------------------------------------------------------------------

    def _execute_move_step(self, step: PickPlaceStep) -> bool:
        if self._mode == RobotMode.SIMULATION:
            return self._sim_move(step)
        else:
            return self._real_move(step)

    def _joint_state_callback(self, msg: JointState) -> None:
        """Store latest joint positions for completion detection."""
        with self._joint_state_lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_joint_positions[name] = pos

    def _sim_move(self, step: PickPlaceStep) -> bool:
        """
        Simulation: send JointTrajectory and wait for joints to reach target.

        Uses joint state monitoring instead of wall-clock timers.
        Wall-clock timers break with use_sim_time:true because the trajectory
        time_from_start is in sim time but threading.Timer runs on wall clock —
        the timer fires before the robot even starts moving in sim time.

        Instead: poll /joint_states at 10Hz and declare success once all 7
        arm joints are within tolerance of the target. Falls back to a hard
        wall-clock timeout as a safety net.
        """
        if step.joint_trajectory is None:
            self.get_logger().error(
                f"[FrankaAdapter] Step '{step.label}' has no joint trajectory."
            )
            return False

        # Target joint positions (last point in trajectory)
        target_names = step.joint_trajectory.joint_names
        target_pos   = list(step.joint_trajectory.points[-1].positions)
        tolerance    = 0.05   # rad — joint position tolerance to declare "reached"

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = step.joint_trajectory

        done       = threading.Event()
        result_box = [None]

        def on_goal_response(future: Future) -> None:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(
                    f"[FrankaAdapter] Arm goal rejected for '{step.label}'."
                )
                result_box[0] = False
                done.set()
                return
            self.get_logger().info(
                f"[FrankaAdapter] Arm goal accepted for '{step.label}'."
            )
            # Also listen for a real result (works on real controllers)
            handle.get_result_async().add_done_callback(on_result)

        def on_result(future: Future) -> None:
            if result_box[0] is not None:
                return
            res = future.result().result
            result_box[0] = (
                res.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            done.set()

        self._arm_client.send_goal_async(goal).add_done_callback(
            on_goal_response
        )

        # Poll joint states until target is reached — works with sim time
        # Large wall-clock timeout because headless Gazebo without GPU
        # acceleration can run at 1/50–1/100 real time.
        max_wait_wall = 600.0   # 10 min wall clock — covers very slow sims
        poll_interval = 0.5     # seconds
        elapsed = 0.0

        # Wait briefly for goal to be accepted before polling
        import time as _time
        _time.sleep(0.5)

        while elapsed < max_wait_wall:
            if result_box[0] is not None:
                # Real result arrived (non-Gazebo controller)
                return result_box[0]

            if result_box[0] is False:
                return False

            # Check if joints reached target
            with self._joint_state_lock:
                current = dict(self._latest_joint_positions)

            if current:
                errors = []
                for name, target in zip(target_names, target_pos):
                    cur = current.get(name)
                    if cur is not None:
                        errors.append(abs(cur - target))

                if len(errors) == len(target_names) and all(
                    e < tolerance for e in errors
                ):
                    self.get_logger().info(
                        f"[FrankaAdapter] '{step.label}' reached target "
                        f"(max err={max(errors):.4f}rad)."
                    )
                    return True

            _time.sleep(poll_interval)
            elapsed += poll_interval

        self.get_logger().error(
            f"[FrankaAdapter] Timeout waiting for '{step.label}' "
            f"after {max_wait_wall}s wall clock."
        )
        return False

    def _real_move(self, step: PickPlaceStep) -> bool:
        """Physical robot: use Franky for motion."""
        try:
            from franky import (  # type: ignore
                CartesianWaypointMotion,
                CartesianWaypoint,
                JointWaypointMotion,
                JointWaypoint,
                Affine,
            )

            # Home step — use JointWaypointMotion with known home angles.
            # More reliable than Cartesian IK for returning to a known config.
            if step.label == "return_home":
                from ros2_packages.robots_adapters.robots_adapters.Franka_pick_place_planner import _FR3_HOME_JOINTS  # type: ignore
                motion = JointWaypointMotion([
                    JointWaypoint(_FR3_HOME_JOINTS)
                ])
                self._franky_robot.move(motion)
                return True

            # All other steps — Cartesian motion to target pose.
            if step.target_pose is None:
                self.get_logger().error(
                    f"[FrankaAdapter] Step '{step.label}' has no target pose."
                )
                return False

            pose = step.target_pose
            motion = CartesianWaypointMotion([
                CartesianWaypoint(Affine([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ]))
            ])
            self._franky_robot.move(motion)
            return True

        except Exception as exc:
            self.get_logger().error(
                f"[FrankaAdapter] Franky move failed for '{step.label}': {exc}"
            )
            return False

    # ------------------------------------------------------------------
    # Internal — gripper step (mode-specific)
    # ------------------------------------------------------------------

    def _execute_gripper_step(self, step: PickPlaceStep) -> bool:
        if self._mode == RobotMode.SIMULATION:
            return self._sim_gripper(step)
        else:
            return self._real_gripper(step)

    def _sim_gripper(self, step: PickPlaceStep) -> bool:
        """Simulation: delegate to GripperAdapter (FollowJointTrajectory)."""
        self._gripper_done.clear()
        self._gripper_ok = False

        if step.kind == StepKind.OPEN_GRIPPER:
            self._gripper.open()
        else:
            self._gripper.close()

        finished = self._gripper_done.wait(timeout=10.0)
        if not finished:
            self.get_logger().error(
                f"[FrankaAdapter] Timeout on gripper step '{step.label}'."
            )
            return False

        return self._gripper_ok

    def _real_gripper(self, step: PickPlaceStep) -> bool:
        """Physical robot: use Franky Gripper API."""
        try:
            if step.kind == StepKind.OPEN_GRIPPER:
                self._franky_gripper.open(
                    speed=self._franky_gripper_speed
                )
            else:
                self._franky_gripper.grasp(
                    width=0.0,
                    speed=self._franky_gripper_speed,
                    force=self._franky_gripper_force,
                )
            return True

        except Exception as exc:
            self.get_logger().error(
                f"[FrankaAdapter] Franky gripper failed for "
                f"'{step.label}': {exc}"
            )
            return False

    def _gripper_result_callback(self, success: bool, message: str) -> None:
        """Called by GripperAdapter (simulation only) when a goal finishes."""
        self._gripper_ok = success
        self._gripper_done.set()

    # ------------------------------------------------------------------
    # Internal — finish
    # ------------------------------------------------------------------

    def _finish(self, success: bool, message: str) -> None:
        icon   = "✓" if success else "✗"
        log_fn = self.get_logger().info if success else self.get_logger().error
        log_fn(f"[FrankaAdapter] {icon} {message}")
        with self._busy_lock:
            self._busy = False
        self._result_callback(success, message)


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone test for FrankaAdapter pick-and-place."
    )
    parser.add_argument(
        "--mode", choices=["sim", "real"], default="sim",
        help="Execution mode: sim (Gazebo) or real (Franky). Default: sim.",
    )
    parser.add_argument(
        "--robot", default="fr3_robot1", metavar="NAME",
        help="Robot namespace (default: fr3_robot1).",
    )
    parser.add_argument(
        "--franky-ip", default=None, metavar="IP",
        help="FR3 robot IP address. Required when --mode real.",
    )
    parser.add_argument(
        "--pick", nargs=3, type=float, metavar=("X", "Y", "Z"),
        default=[0.5, 0.0, 0.3],
        help="Pick position in metres.",
    )
    parser.add_argument(
        "--place", nargs=3, type=float, metavar=("X", "Y", "Z"),
        default=[0.5, 0.4, 0.3],
        help="Place position in metres.",
    )
    parser.add_argument(
        "--base-height", type=float, default=0.0, metavar="M",
        help="Height of robot base above world origin in metres (default: 0.0). "
             "Set to 1.03 if the robot is spawned at z=1.03.",
    )
    parser.add_argument(
        "--timeout", type=float, default=600.0,
        help="Seconds to wait for task completion (default: 600 — covers slow headless sim).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()

    mode = RobotMode(args.mode)

    done_event = threading.Event()
    result_box: list = []

    def on_result(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    adapter = FrankaAdapter(
        robot_name      = args.robot,
        mode            = mode,
        result_callback = on_result,
        franky_ip       = args.franky_ip,
        base_height     = args.base_height,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(adapter)

    # In simulation mode, GripperAdapter is a separate node that also needs
    # to spin alongside FrankaAdapter.
    if mode == RobotMode.SIMULATION and adapter._gripper is not None:
        executor.add_node(adapter._gripper)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        accepted = adapter.pick_and_place(
            pick_xyz  = tuple(args.pick),
            place_xyz = tuple(args.place),
        )

        if not accepted:
            print("[standalone] Task rejected — adapter is busy.")
        else:
            print(
                f"[standalone] Task accepted ({mode.value}). "
                f"Waiting up to {args.timeout}s ..."
            )
            finished = done_event.wait(timeout=args.timeout)
            if not finished:
                print("[standalone] ✗ Timed out.")
            else:
                success, message = result_box[0]
                print(f"[standalone] {'✓' if success else '✗'} {message}")

    except KeyboardInterrupt:
        print("\n[standalone] Interrupted.")
        adapter.cancel()
        time.sleep(1.0)
    finally:
        executor.shutdown()
        if adapter._gripper is not None:
            adapter._gripper.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()