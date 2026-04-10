"""
ROS2 adapter for the Tiago mobile manipulator (arm + PAL gripper + mobile base).
Simulation only.

Coordinate modes
----------------
World-frame mode  (robot_frame=False, default)
    pick_xyz / place_xyz are in the world / odometry frame.
    The adapter drives the base to a computed nav pose before executing
    the arm trajectory.

Robot-frame mode  (robot_frame=True)
    pick_xyz / place_xyz are already in the robot base frame, as returned
    by ObjectToRobot.get_pose()["robot_pose"].position.
    The adapter does NOT navigate — the caller must:
      1. Drive the robot to a suitable position first.
      2. Re-query ObjectToRobot AFTER arriving so coordinates are fresh.
      3. Call pick_and_place() with those fresh coordinates.
    Only the arm and gripper are moved in this mode.

Usage (world-frame)
-------------------
    adapter = TiagoAdapter(robot_name="tiago_robot1", mode=RobotMode.SIMULATION,
                           result_callback=on_done)
    adapter.pick_and_place(pick_xyz=(2.0, 1.5, 0.875),
                           place_xyz=(4.0, 2.0, 0.875))

Usage (robot-frame with ObjectToRobot)
---------------------------------------
    adapter = TiagoAdapter(robot_name="tiago_robot1", mode=RobotMode.SIMULATION,
                           result_callback=on_done, robot_frame=True)

    pick_result  = obj_transformer.get_pose("tomato_1")
    place_result = obj_transformer.get_pose("bowl_1")
    pick_pos  = pick_result["robot_pose"].position
    place_pos = place_result["robot_pose"].position

    adapter.pick_and_place(
        pick_xyz  = (pick_pos.x,  pick_pos.y,  pick_pos.z),
        place_xyz = (place_pos.x, place_pos.y, place_pos.z),
    )
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from enum import Enum
from typing import Callable, Optional, Tuple

import rclpy
import rclpy.parameter
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from tiago_gripper_adapter import TiagoGripperAdapter
from tiago_pick_place import TiagoPickPlacePlanner
from tiago_pick_place_result import PickPlanResult, PickPlaceStep, StepKind

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ            = Tuple[float, float, float]
NavPose        = Tuple[float, float, float]   # (x, y, theta)
ResultCallback = Callable[[bool, str], None]


class RobotMode(str, Enum):
    SIMULATION = "sim"


# ---------------------------------------------------------------------------
# Navigation constants
# ---------------------------------------------------------------------------

_NAV_RATE_HZ    = 10.0
_GOAL_XY_TOL    = 0.15    # m
_GOAL_THETA_TOL = 0.08    # rad
_MAX_LINEAR     = 0.60    # m/s  (was 0.30)
_MAX_ANGULAR    = 1.20    # rad/s (was 0.80)
_K_LINEAR       = 0.80    # (was 0.50)
_K_ANGULAR      = 2.00    # (was 1.50)
_NAV_TIMEOUT    = 120.0   # s


# ---------------------------------------------------------------------------
# TiagoAdapter
# ---------------------------------------------------------------------------

class TiagoAdapter(Node):
    """
    Parameters
    ----------
    robot_name : str
        Namespace / Gazebo model name (e.g. "tiago_robot1").
    mode : RobotMode
        Currently only RobotMode.SIMULATION is supported.
    result_callback : ResultCallback
        Called on task completion: callback(success: bool, message: str).
    robot_frame : bool
        If True, pick_and_place() expects coordinates in the robot base frame
        (from ObjectToRobot) and skips all navigation steps.
        If False (default), coordinates are in the world frame and the adapter
        drives the base automatically.
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83.
    node_name : str, optional
        ROS2 node name override.
    """

    def __init__(
        self,
        robot_name:      str,
        mode:            RobotMode,
        result_callback: ResultCallback,
        robot_frame:     bool          = False,
        arm_base_z:      float         = 0.83,
        node_name:       Optional[str] = None,
    ) -> None:
        node_name = node_name or f"tiago_adapter_{robot_name}"
        super().__init__(
            node_name,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )

        self._robot_name      = robot_name
        self._mode            = mode
        self._result_callback = result_callback
        self._robot_frame     = robot_frame
        self._cancelled       = False

        self._busy      = False
        self._busy_lock = threading.Lock()

        self._planner = TiagoPickPlacePlanner(
            robot_name  = robot_name,
            robot_frame = robot_frame,
            arm_base_z  = arm_base_z,
        )

        self._robot_pose: NavPose = (0.0, 0.0, 0.0)
        self._robot_pose_lock     = threading.Lock()
        self._pose_ready          = threading.Event()

        mode_label = "ROBOT-FRAME" if robot_frame else "WORLD-FRAME"
        self.get_logger().info(
            f"[TiagoAdapter] Initialising ({mode.value.upper()}, {mode_label}) ..."
        )
        self._init_ros()
        self.get_logger().info("[TiagoAdapter] Ready.")

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _init_ros(self) -> None:
        self._cmd_vel = self.create_publisher(
            Twist,
            f"/{self._robot_name}/mobile_base_controller/cmd_vel_unstamped",
            10,
        )

        arm_action = f"/{self._robot_name}/arm_controller/follow_joint_trajectory"
        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self.get_logger().info(f"[TiagoAdapter] Waiting for arm '{arm_action}' ...")
        self._arm_client.wait_for_server()
        self.get_logger().info("[TiagoAdapter] Arm server ready.")

        self._gripper_done = threading.Event()
        self._gripper_ok   = False
        self._gripper = TiagoGripperAdapter(
            robot_name      = self._robot_name,
            result_callback = self._gripper_result_callback,
            node_name       = f"tiago_gripper_{self._robot_name}",
        )

        self._latest_joint_positions: dict = {}
        self._joint_state_lock = threading.Lock()
        self.create_subscription(
            JointState,
            f"/{self._robot_name}/joint_states",
            self._joint_state_callback,
            10,
        )

        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry,
            f"/{self._robot_name}/mobile_base_controller/odom",
            self._odom_callback,
            10,
        )

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

        Coordinates are in the robot base frame when robot_frame=True,
        or in the world frame when robot_frame=False.

        Non-blocking — returns True if accepted, False if already busy.
        result_callback fires on completion.
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn("[TiagoAdapter] Rejected — already busy.")
                return False
            self._busy = True

        # Odometry is needed even in robot_frame mode for arm joint polling
        # to have a valid robot_pose to pass to the planner (unused for nav).
        if not self._pose_ready.wait(timeout=15.0):
            self.get_logger().error(
                "[TiagoAdapter] No odometry within 15 s. "
                f"Check: ros2 topic echo /{self._robot_name}/mobile_base_controller/odom"
            )
            with self._busy_lock:
                self._busy = False
            self._result_callback(False, "Robot pose unavailable.")
            return True

        robot_pose = self._get_robot_pose()
        frame_label = "robot-frame" if self._robot_frame else "world-frame"
        self.get_logger().info(
            f"[TiagoAdapter] pick_and_place [{frame_label}]"
            f"  pick={pick_xyz}  place={place_xyz}"
            f"  robot=({robot_pose[0]:.2f}, {robot_pose[1]:.2f}, "
            f"{math.degrees(robot_pose[2]):.1f}°)"
        )

        with self._joint_state_lock:
            current_arm_joints = dict(self._latest_joint_positions)

        plan = self._planner.plan(
            pick_xyz, place_xyz, robot_pose,
            current_arm_joints=current_arm_joints,
        )
        if not plan.success:
            self.get_logger().error(f"[TiagoAdapter] Planning failed: {plan.message}")
            with self._busy_lock:
                self._busy = False
            self._result_callback(False, plan.message)
            return True

        threading.Thread(
            target=self._execute_plan,
            args=(plan, pick_xyz, place_xyz),
            daemon=True,
        ).start()
        return True

    def cancel(self) -> None:
        with self._busy_lock:
            if not self._busy:
                return
            self._cancelled = True
        self._stop_base()
        self.get_logger().info("[TiagoAdapter] Cancellation requested.")

    @property
    def is_busy(self) -> bool:
        with self._busy_lock:
            return self._busy

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg) -> None:
        q     = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        with self._robot_pose_lock:
            self._robot_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                theta,
            )
        self._pose_ready.set()

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._joint_state_lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_joint_positions[name] = pos

    def _get_robot_pose(self) -> NavPose:
        with self._robot_pose_lock:
            return self._robot_pose

    # ------------------------------------------------------------------
    # Plan execution
    # ------------------------------------------------------------------

    # Labels that represent arm-tuck / return-home moves.
    # At execution time we check live joint states and skip if already at home.
    _TUCK_LABELS = frozenset({
        "tuck_arm_initial",
        "tuck_arm_pre_place_nav",
        "return_home",
    })

    def _dispatch_step(self, step: PickPlaceStep) -> bool:
        if step.kind == StepKind.NAVIGATE:
            if self._robot_frame:
                self.get_logger().warn(
                    "[TiagoAdapter] NAVIGATE step ignored in robot-frame mode."
                )
                return True
            return self._execute_nav_step(step)
        elif step.kind == StepKind.MOVE:
            # Skip tuck steps if arm is already at the home/tuck pose.
            if step.label in self._TUCK_LABELS:
                with self._joint_state_lock:
                    current = dict(self._latest_joint_positions)
                if self._planner._is_arm_at_home(current):
                    self.get_logger().info(
                        f"[TiagoAdapter] '{step.label}' skipped — arm already at home."
                    )
                    return True
            return self._execute_move_step(step)
        elif step.kind in (StepKind.OPEN_GRIPPER, StepKind.CLOSE_GRIPPER):
            return self._execute_gripper_step(step)
        self.get_logger().error(f"[TiagoAdapter] Unknown step kind: {step.kind}")
        return False

    _MAX_REPLAN_ATTEMPTS = 3

    def _execute_plan(self, plan: PickPlanResult, pick_xyz: XYZ, place_xyz: XYZ) -> None:
        self._cancelled = False
        grasp_complete  = False
        replan_count    = 0

        i = 0
        while i < len(plan.steps):
            if self._cancelled:
                self._finish(False, "Task cancelled.")
                return

            step = plan.steps[i]
            self.get_logger().info(f"[TiagoAdapter] Step {i+1}/{len(plan.steps)}: {step.label}")

            if self._dispatch_step(step):
                if step.label == "close_gripper_grasp":
                    grasp_complete = True
                i += 1
                continue

            self.get_logger().warn(f"[TiagoAdapter] Step '{step.label}' failed.")

            if grasp_complete:
                self._finish(False, f"Step '{step.label}' failed after grasp — aborting.")
                return

            if replan_count >= self._MAX_REPLAN_ATTEMPTS:
                self._finish(
                    False,
                    f"Aborted: '{step.label}' failed after {self._MAX_REPLAN_ATTEMPTS} attempts.",
                )
                return

            replan_count += 1
            robot_pose = self._get_robot_pose()
            with self._joint_state_lock:
                current_arm_joints = dict(self._latest_joint_positions)

            self.get_logger().info(
                f"[TiagoAdapter] Replanning (attempt {replan_count}) ..."
            )
            new_plan = self._planner.plan(
                pick_xyz, place_xyz, robot_pose,
                current_arm_joints=current_arm_joints,
            )
            if new_plan.success:
                plan = new_plan
                i    = 0
            else:
                self.get_logger().error(
                    f"[TiagoAdapter] Replan {replan_count} failed: {new_plan.message}"
                )
                if replan_count >= self._MAX_REPLAN_ATTEMPTS:
                    self._finish(False, f"Aborted: replan failed — {new_plan.message}")
                    return

        self._finish(True, "Pick and place complete.")

    # ------------------------------------------------------------------
    # Navigation  (world-frame mode only)
    # ------------------------------------------------------------------

    def _execute_nav_step(self, step: PickPlaceStep) -> bool:
        if step.nav_goal is None:
            self.get_logger().error(
                f"[TiagoAdapter] NAVIGATE step '{step.label}' has no nav_goal."
            )
            return False

        goal_x, goal_y, goal_theta = step.nav_goal
        dt    = 1.0 / _NAV_RATE_HZ
        start = time.perf_counter()

        self.get_logger().info(
            f"[TiagoAdapter] Navigating to "
            f"({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_theta):.1f}°)."
        )

        while time.perf_counter() - start < _NAV_TIMEOUT:
            if self._cancelled:
                self._stop_base()
                return False

            rx, ry, rtheta = self._get_robot_pose()
            dist_xy = math.hypot(goal_x - rx, goal_y - ry)

            if dist_xy < _GOAL_XY_TOL:
                theta_err = _wrap_angle(goal_theta - rtheta)
                if abs(theta_err) < _GOAL_THETA_TOL:
                    self._stop_base()
                    self.get_logger().info(f"[TiagoAdapter] Reached '{step.label}'.")
                    return True
                angular = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)
                self._publish_vel(0.0, angular)
                time.sleep(dt)
                continue

            desired_heading = math.atan2(goal_y - ry, goal_x - rx)
            theta_err = _wrap_angle(desired_heading - rtheta)
            linear    = _clamp(_K_LINEAR * dist_xy * max(math.cos(theta_err), 0.0),
                               0.0, _MAX_LINEAR)
            angular   = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)
            self._publish_vel(linear, angular)
            time.sleep(dt)

        self._stop_base()
        self.get_logger().error(
            f"[TiagoAdapter] Navigation timeout for '{step.label}'."
        )
        return False

    def _publish_vel(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self._cmd_vel.publish(twist)

    def _stop_base(self) -> None:
        self._cmd_vel.publish(Twist())

    # ------------------------------------------------------------------
    # Arm move step
    # ------------------------------------------------------------------

    def _execute_move_step(self, step: PickPlaceStep) -> bool:
        if step.joint_trajectory is None:
            self.get_logger().error(
                f"[TiagoAdapter] Step '{step.label}' has no joint trajectory."
            )
            return False

        target_names = step.joint_trajectory.joint_names
        target_pos   = list(step.joint_trajectory.points[-1].positions)
        tolerance    = 0.05

        from builtin_interfaces.msg import Duration as RosDuration
        from control_msgs.msg import JointTolerance

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = step.joint_trajectory

        for joint_name in step.joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name     = joint_name
            tol.position = -1.0
            goal.path_tolerance.append(tol)

        for joint_name in step.joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name     = joint_name
            tol.position = 0.1
            goal.goal_tolerance.append(tol)

        goal.goal_time_tolerance = RosDuration(sec=60, nanosec=0)

        goal_accepted = threading.Event()
        goal_rejected = threading.Event()

        def on_goal_response(future: Future) -> None:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(
                    f"[TiagoAdapter] Arm goal rejected for '{step.label}'."
                )
                goal_rejected.set()
            else:
                self.get_logger().info(
                    f"[TiagoAdapter] Arm goal accepted for '{step.label}'."
                )
                goal_accepted.set()

        self._arm_client.send_goal_async(goal).add_done_callback(on_goal_response)

        if not goal_accepted.wait(timeout=10.0):
            if goal_rejected.is_set():
                self.get_logger().error(
                    f"[TiagoAdapter] Arm goal rejected for '{step.label}'."
                )
            else:
                self.get_logger().error(
                    f"[TiagoAdapter] Arm action server unresponsive for '{step.label}'."
                )
            return False

        max_wait      = 90.0
        poll_interval = 0.5
        elapsed       = 0.0
        _logged_names = False

        while elapsed < max_wait:
            if goal_rejected.is_set():
                return False

            with self._joint_state_lock:
                current = dict(self._latest_joint_positions)

            if current:
                if not _logged_names:
                    _logged_names = True
                    missing = set(target_names) - set(current.keys())
                    if missing:
                        self.get_logger().warn(
                            f"[TiagoAdapter] Missing joints for '{step.label}': {sorted(missing)}"
                        )

                errors = [
                    abs(current.get(name, float("inf")) - tgt)
                    for name, tgt in zip(target_names, target_pos)
                ]
                if all(e < tolerance for e in errors):
                    self.get_logger().info(
                        f"[TiagoAdapter] '{step.label}' reached "
                        f"(max err={max(errors):.4f} rad)."
                    )
                    return True

            time.sleep(poll_interval)
            elapsed += poll_interval

        self.get_logger().error(
            f"[TiagoAdapter] Arm timeout after {max_wait:.0f}s for '{step.label}'."
        )
        return False

    # ------------------------------------------------------------------
    # Gripper step
    # ------------------------------------------------------------------

    def _execute_gripper_step(self, step: PickPlaceStep) -> bool:
        self._gripper_done.clear()
        self._gripper_ok = False

        if step.kind == StepKind.OPEN_GRIPPER:
            self._gripper.open()
        else:
            self._gripper.close()

        if not self._gripper_done.wait(timeout=15.0):
            self.get_logger().error(
                f"[TiagoAdapter] Gripper timeout for '{step.label}'."
            )
            return False
        return self._gripper_ok

    def _gripper_result_callback(self, success: bool, message: str) -> None:
        self._gripper_ok = success
        self._gripper_done.set()

    # ------------------------------------------------------------------
    # Finish
    # ------------------------------------------------------------------

    def _finish(self, success: bool, message: str) -> None:
        icon   = "✓" if success else "✗"
        log_fn = self.get_logger().info if success else self.get_logger().error
        log_fn(f"[TiagoAdapter] {icon} {message}")
        with self._busy_lock:
            self._busy = False
        self._result_callback(success, message)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone TiagoAdapter test.")
    parser.add_argument("--robot", default="tiago_robot1")
    parser.add_argument("--pick",  nargs=3, type=float, metavar=("X","Y","Z"),
                        default=[0.5, 0.0, 0.875])
    parser.add_argument("--place", nargs=3, type=float, metavar=("X","Y","Z"),
                        default=[0.5, 0.2, 0.875])
    parser.add_argument("--robot-frame", action="store_true",
                        help="Treat pick/place coords as robot-base-frame "
                             "(as returned by ObjectToRobot). Skips navigation.")
    parser.add_argument("--arm-base-z", type=float, default=0.83)
    parser.add_argument("--timeout",    type=float, default=600.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()

    done_event = threading.Event()
    result_box: list = []

    def on_result(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    adapter = TiagoAdapter(
        robot_name      = args.robot,
        mode            = RobotMode.SIMULATION,
        result_callback = on_result,
        robot_frame     = args.robot_frame,
        arm_base_z      = args.arm_base_z,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(adapter)
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
            print(f"[standalone] Task accepted. Waiting up to {args.timeout}s ...")
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
        adapter._pose_ready.set()
        time.sleep(1.0)
        executor.shutdown(timeout_sec=2.0)
        for node in (adapter._gripper, adapter):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()