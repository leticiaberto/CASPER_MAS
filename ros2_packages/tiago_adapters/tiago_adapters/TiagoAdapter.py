"""
ROS2 adapter for the Tiago mobile manipulator (arm + PAL gripper + mobile base).
Simulation only.

Navigation strategy
-------------------
No Nav2, no map, no SLAM required.

Ignition Fortress publishes /world/<name>/dynamic_pose/info (gz.msgs.Pose_V),
bridged to tf2_msgs/TFMessage.  This gives exact world-frame positions for
every model at every timestep.  With this ground-truth data the adapter runs a
lightweight reactive potential-field controller directly on cmd_vel_unstamped:

  • Attraction : drives the base toward the goal pose.
  • Repulsion  : pushes away from other robots and obstacles proportionally
                 to 1/distance², active within REP_INFLUENCE metres.

This handles dynamic obstacles (other moving robots) naturally, because the
model_states topic is updated continuously and the control loop re-reads it at
every iteration.

Usage
-----
    adapter = TiagoAdapter(
        robot_name="tiago_robot1",
        mode=RobotMode.SIMULATION,
        result_callback=on_done,
    )
    adapter.pick_and_place(
        pick_xyz  = (2.0, 1.5, 0.8),
        place_xyz = (4.0, 2.0, 0.8),
    )

Standalone
----------
    python3 TiagoAdapter.py --robot tiago_robot1 \\
        --pick 2.0 1.5 0.8 --place 4.0 2.0 0.8
"""

from __future__ import annotations

import argparse
import math
import re
import threading
import time
from enum import Enum
from typing import Callable, List, Optional, Tuple

import rclpy
import rclpy.parameter
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from tiago_gripper_adapter import TiagoGripperAdapter
from tiago_pick_place import Obstacle, TiagoPickPlacePlanner
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
# Reactive navigation constants
# ---------------------------------------------------------------------------

_NAV_RATE_HZ    = 10.0   # control loop frequency (Hz)
_GOAL_XY_TOL    = 0.15   # m   — position tolerance to declare "arrived"
_GOAL_THETA_TOL = 0.08   # rad — heading tolerance (~5°)
_MAX_LINEAR     = 0.30   # m/s — maximum forward speed
_MAX_ANGULAR    = 0.80   # rad/s
_K_LINEAR       = 0.50   # attraction gain → linear speed per metre of error
_K_ANGULAR      = 1.50   # heading correction gain
_K_REP          = 0.50   # repulsion gain
_REP_INFLUENCE  = 1.50   # m   — obstacle influence radius
_NAV_TIMEOUT    = 60.0   # s   — wall-clock timeout per navigation step

# Conservative footprint radius for other Gazebo models (no bounding box info)
_DEFAULT_OBSTACLE_RADIUS = 0.30   # m


# ---------------------------------------------------------------------------
# TiagoAdapter
# ---------------------------------------------------------------------------

class TiagoAdapter(Node):
    """
    Parameters
    ----------
    robot_name : str
        Namespace / Gazebo model name of this robot (e.g. "tiago_robot1").
    mode : RobotMode
        Currently only RobotMode.SIMULATION is supported.
    result_callback : ResultCallback
        Called when a task finishes: callback(success: bool, message: str).
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83 m.
    node_name : str, optional
        ROS2 node name override.
    """

    def __init__(
        self,
        robot_name:       str,
        mode:             RobotMode,
        result_callback:  ResultCallback,
        arm_base_z:       float         = 0.83,
        node_name:        Optional[str] = None,
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
        self._cancelled       = False

        self._busy      = False
        self._busy_lock = threading.Lock()

        self._planner = TiagoPickPlacePlanner(
            robot_name = robot_name,
            arm_base_z = arm_base_z,
        )

        # Robot pose in world frame — kept current by model_states callback,
        # with odometry as a fallback if Gazebo bridge is not running.
        self._robot_pose: NavPose = (0.0, 0.0, 0.0)
        self._robot_pose_lock     = threading.Lock()

        # Other robots / objects in the scene
        self._obstacles: List[Obstacle] = []
        self._obstacles_lock            = threading.Lock()

        self.get_logger().info(
            f"[TiagoAdapter] Initialising ({mode.value.upper()}) ..."
        )
        self._init_ros()
        self.get_logger().info("[TiagoAdapter] Ready.")

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _init_ros(self) -> None:

        # ── Base velocity publisher ───────────────────────────────────
        # PAL uses cmd_vel_unstamped, not cmd_vel.
        self._cmd_vel = self.create_publisher(
            Twist,
            f"/{self._robot_name}/mobile_base_controller/cmd_vel_unstamped",
            10,
        )

        # ── Arm action client ─────────────────────────────────────────
        arm_action = f"/{self._robot_name}/arm_controller/follow_joint_trajectory"
        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self.get_logger().info(f"[TiagoAdapter] Waiting for arm '{arm_action}' ...")
        self._arm_client.wait_for_server()
        self.get_logger().info("[TiagoAdapter] Arm server ready.")

        # ── Gripper adapter ───────────────────────────────────────────
        self._gripper_done = threading.Event()
        self._gripper_ok   = False
        self._gripper = TiagoGripperAdapter(
            robot_name      = self._robot_name,
            result_callback = self._gripper_result_callback,
            node_name       = f"tiago_gripper_{self._robot_name}",
        )

        # ── Joint state subscriber — arm completion detection ──────────
        # Polling is used instead of wall-clock timers because
        # use_sim_time=true makes threading.Timer unreliable.
        self._latest_joint_positions: dict = {}
        self._joint_state_lock = threading.Lock()
        self.create_subscription(
            JointState,
            f"/{self._robot_name}/joint_states",
            self._joint_state_callback,
            10,
        )

        # ── Ignition Fortress dynamic poses — robot pose + scene obstacles ──
        # Bridge: gz.msgs.Pose_V  →  tf2_msgs/TFMessage
        # Topic:  /world/<world_name>/dynamic_pose/info
        #
        # The world name is discovered automatically by scanning live topics
        # after the executor starts spinning, so the adapter works regardless
        # of which SDF world is loaded.

        # _pose_ready is set the first time a pose is received for this robot.
        # pick_and_place() waits on it so planning never runs against (0,0,0).
        self._pose_ready = threading.Event()

        # Signals the discovery thread to stop (set before destroying the node).
        self._stop_discovery = threading.Event()

        # Discovery runs in a background thread so it doesn't block __init__
        # (get_topic_names_and_types() only returns data once the executor spins).
        threading.Thread(
            target=self._subscribe_dynamic_poses,
            daemon=True,
        ).start()

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

        Navigation is computed automatically — no nav poses needed.
        Non-blocking: returns True if accepted, False if already busy.
        result_callback(success, message) fires on completion.
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    "[TiagoAdapter] Rejected — a task is already in progress."
                )
                return False
            self._busy = True

        # Wait for the first world-frame pose from /gazebo/model_states.
        # Odometry is NOT used — it is relative to the spawn point, not the
        # world frame, so it gives wrong coordinates for non-zero spawn positions.
        # Wait longer than the 15 s world-discovery window so that discovery
        # has time to find the topic and the first pose message to arrive.
        if not self._pose_ready.wait(timeout=30.0):
            self.get_logger().error(
                "[TiagoAdapter] No pose received from the Ignition dynamic_pose "
                "bridge within 30 s. Is ros_gz_bridge running? "
                "Check: ros2 topic echo /world/<name>/dynamic_pose/info"
            )
            with self._busy_lock:
                self._busy = False
            self._result_callback(False, "Robot pose unavailable.")
            return True

        robot_pose = self._get_robot_pose()
        with self._obstacles_lock:
            obstacles = list(self._obstacles)

        self.get_logger().info(
            f"[TiagoAdapter] pick_and_place  pick={pick_xyz}  place={place_xyz}"
            f"  robot=({robot_pose[0]:.2f}, {robot_pose[1]:.2f}, "
            f"{math.degrees(robot_pose[2]):.1f}°)"
            f"  dynamic_obstacles={len(obstacles)}"
        )

        plan = self._planner.plan(pick_xyz, place_xyz, robot_pose, obstacles)
        if not plan.success:
            self.get_logger().error(
                f"[TiagoAdapter] Planning failed: {plan.message}"
            )
            with self._busy_lock:
                self._busy = False
            self._result_callback(False, plan.message)
            return True

        threading.Thread(
            target = self._execute_plan,
            args   = (plan,),
            daemon = True,
        ).start()
        return True

    def cancel(self) -> None:
        """Request cancellation of the current task (best-effort)."""
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

    def _subscribe_dynamic_poses(self) -> None:
        """
        Background thread: subscribe to every /world/*/dynamic_pose/info topic
        that the bridge advertises.  Only the active Ignition world will ever
        publish data — the others stay silent.  This avoids having to guess
        which world name is currently loaded.
        """
        pattern = re.compile(r"^/world/([^/]+)/dynamic_pose/info$")
        deadline = time.perf_counter() + 15.0

        self.get_logger().info(
            "[TiagoAdapter] Waiting for /world/*/dynamic_pose/info topics ..."
        )

        subscribed: set = set()
        while time.perf_counter() < deadline:
            if self._stop_discovery.is_set():
                return

            for topic_name, _ in self.get_topic_names_and_types():
                if topic_name in subscribed:
                    continue
                if not pattern.match(topic_name):
                    continue

                self.create_subscription(
                    TFMessage,
                    topic_name,
                    self._world_poses_callback,
                    10,
                )
                subscribed.add(topic_name)
                self.get_logger().info(
                    f"[TiagoAdapter] Subscribed to '{topic_name}'."
                )

            if subscribed:
                # Found at least one topic — stop polling.
                self.get_logger().info(
                    "[TiagoAdapter] Dynamic pose subscriptions active. "
                    "Active world will deliver data; inactive worlds stay silent."
                )
                return

            time.sleep(0.5)

        self.get_logger().error(
            "[TiagoAdapter] No /world/*/dynamic_pose/info topics found within "
            "15 s. Check that ros_gz_bridge is running and "
            "tiago_gz_bridge.yaml includes the dynamic_pose entry."
        )

    def _world_poses_callback(self, msg: TFMessage) -> None:
        """
        Parse the bridged Ignition Fortress dynamic_pose topic.

        Each TransformStamped in msg.transforms has:
          child_frame_id  — the Gazebo model name
          transform.translation.{x,y,z}  — world-frame position
          transform.rotation.{x,y,z,w}   — world-frame orientation

        This robot's own entry is used to keep _robot_pose current.
        All other entries (except static scene fixtures) become obstacles.
        """
        _STATIC_MODELS = {"ground_plane", "sun", ""}

        obstacles: List[Obstacle] = []

        for t in msg.transforms:
            name = t.child_frame_id

            if name == self._robot_name:
                q = t.transform.rotation
                theta = 2.0 * math.atan2(q.z, q.w)
                with self._robot_pose_lock:
                    self._robot_pose = (
                        t.transform.translation.x,
                        t.transform.translation.y,
                        theta,
                    )
                self._pose_ready.set()
                continue

            if name in _STATIC_MODELS:
                continue

            obstacles.append(Obstacle(
                name   = name,
                x      = t.transform.translation.x,
                y      = t.transform.translation.y,
                radius = _DEFAULT_OBSTACLE_RADIUS,
            ))

        with self._obstacles_lock:
            self._obstacles = obstacles

    def _joint_state_callback(self, msg: JointState) -> None:
        with self._joint_state_lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_joint_positions[name] = pos

    # ------------------------------------------------------------------
    # Robot pose
    # ------------------------------------------------------------------

    def _get_robot_pose(self) -> NavPose:
        with self._robot_pose_lock:
            return self._robot_pose

    # ------------------------------------------------------------------
    # Plan execution
    # ------------------------------------------------------------------

    def _execute_plan(self, plan: PickPlanResult) -> None:
        self._cancelled = False

        for i, step in enumerate(plan.steps):
            if self._cancelled:
                self._finish(False, "Task cancelled.")
                return

            self.get_logger().info(
                f"[TiagoAdapter] Step {i+1}/{len(plan.steps)}: {step.label}"
            )

            if step.kind == StepKind.NAVIGATE:
                success = self._execute_nav_step(step)
            elif step.kind == StepKind.MOVE:
                success = self._execute_move_step(step)
            elif step.kind in (StepKind.OPEN_GRIPPER, StepKind.CLOSE_GRIPPER):
                success = self._execute_gripper_step(step)
            else:
                self.get_logger().error(
                    f"[TiagoAdapter] Unknown step kind: {step.kind}"
                )
                success = False

            if not success:
                self._finish(False, f"Step '{step.label}' failed.")
                return

        self._finish(True, "Pick and place complete.")

    # ------------------------------------------------------------------
    # Reactive navigation (potential field on cmd_vel_unstamped)
    # ------------------------------------------------------------------

    def _execute_nav_step(self, step: PickPlaceStep) -> bool:
        """
        Drive the base to step.nav_goal using a potential-field controller.

        Attraction toward the goal + repulsion from all scene obstacles,
        re-evaluated at every control tick so moving robots are handled
        without any replanning.

        Two phases:
          1. Move phase — drive toward goal XY while avoiding obstacles.
          2. Align phase — rotate in place to match goal theta.
        """
        if step.nav_goal is None:
            self.get_logger().error(
                f"[TiagoAdapter] NAVIGATE step '{step.label}' has no nav_goal."
            )
            return False

        goal_x, goal_y, goal_theta = step.nav_goal
        dt      = 1.0 / _NAV_RATE_HZ
        start   = time.perf_counter()

        self.get_logger().info(
            f"[TiagoAdapter] Navigating to "
            f"({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_theta):.1f}°) ..."
        )

        while time.perf_counter() - start < _NAV_TIMEOUT:

            if self._cancelled:
                self._stop_base()
                return False

            rx, ry, rtheta = self._get_robot_pose()
            dist_xy = math.hypot(goal_x - rx, goal_y - ry)

            # ── Phase 2: position reached — align heading ──────────────
            if dist_xy < _GOAL_XY_TOL:
                theta_err = _wrap_angle(goal_theta - rtheta)
                if abs(theta_err) < _GOAL_THETA_TOL:
                    self._stop_base()
                    self.get_logger().info(
                        f"[TiagoAdapter] Goal '{step.label}' reached."
                    )
                    return True
                angular = _clamp(_K_ANGULAR * theta_err,
                                 -_MAX_ANGULAR, _MAX_ANGULAR)
                self._publish_vel(0.0, angular)
                time.sleep(dt)
                continue

            # ── Phase 1: drive toward goal ─────────────────────────────

            # Attractive force (unit vector toward goal, world frame)
            att_x = (goal_x - rx) / dist_xy
            att_y = (goal_y - ry) / dist_xy

            # Repulsive forces from all obstacles (world frame)
            rep_x, rep_y = 0.0, 0.0
            with self._obstacles_lock:
                obstacles = list(self._obstacles)

            for obs in obstacles:
                dx_obs = rx - obs.x
                dy_obs = ry - obs.y
                raw_dist = math.hypot(dx_obs, dy_obs)
                # Surface distance (subtract obstacle radius)
                d = max(raw_dist - obs.radius, 0.05)
                if d >= _REP_INFLUENCE:
                    continue
                mag  = _K_REP * (1.0 / d - 1.0 / _REP_INFLUENCE) / (d ** 2)
                norm = max(raw_dist, 1e-6)
                rep_x += mag * dx_obs / norm
                rep_y += mag * dy_obs / norm

            # Combined force (world frame)
            fx = att_x + rep_x
            fy = att_y + rep_y
            f_mag = math.hypot(fx, fy)

            if f_mag < 1e-6:
                # Force is zero (attraction and repulsion cancel exactly) — stop
                self._stop_base()
                time.sleep(dt)
                continue

            # Desired heading from combined force
            desired_heading = math.atan2(fy, fx)
            theta_err = _wrap_angle(desired_heading - rtheta)

            # Linear speed: proportional to force magnitude, zeroed when
            # misaligned by more than 90° so the robot turns before driving.
            cos_err = math.cos(theta_err)
            linear  = _clamp(
                _K_LINEAR * f_mag * max(cos_err, 0.0),
                0.0, _MAX_LINEAR,
            )

            # Angular: correct toward desired heading
            angular = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)

            self._publish_vel(linear, angular)
            time.sleep(dt)

        self._stop_base()
        self.get_logger().error(
            f"[TiagoAdapter] Navigation timeout for '{step.label}' "
            f"after {_NAV_TIMEOUT:.0f}s."
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
        tolerance    = 0.05   # rad

        from builtin_interfaces.msg import Duration as RosDuration
        from control_msgs.msg import JointTolerance

        # stamp=0 means "start immediately, time_from_start is relative to
        # goal receipt time". Setting a non-zero stamp requires the node
        # clock to be fully synced with sim time, which isn't guaranteed
        # during the first callback — stamp=0 is always safe.
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = step.joint_trajectory

        # Disable path tolerance checking (the mid-execution position check).
        # The controller runs at 1 Hz due to missing update_rate parameter;
        # at 1 Hz the arm steps coarsely and violates the default 0.02 rad
        # path tolerance on every tick. We check completion ourselves via
        # joint-state polling, so path tolerance is not needed.
        # Empty list = use controller defaults... so we explicitly zero them.
        for joint_name in step.joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name     = joint_name
            tol.position = -1.0   # -1 = no tolerance check (ros2_control convention)
            goal.path_tolerance.append(tol)

        # Goal tolerance: 0.1 rad — we do our own check at 0.05 rad in polling.
        for joint_name in step.joint_trajectory.joint_names:
            tol = JointTolerance()
            tol.name     = joint_name
            tol.position = 0.1
            goal.goal_tolerance.append(tol)

        # Give the controller plenty of time past trajectory duration.
        goal.goal_time_tolerance = RosDuration(sec=60, nanosec=0)

        goal_rejected = threading.Event()

        def on_goal_response(future: Future) -> None:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(
                    f"[TiagoAdapter] Arm goal rejected for '{step.label}'."
                )
                goal_rejected.set()
                return
            self.get_logger().info(
                f"[TiagoAdapter] Arm goal accepted for '{step.label}'."
            )
            # We do NOT wait for the action result — the controller may abort
            # with goal_time_tolerance even though the joints are close enough.
            # Joint-state polling below is the authoritative completion check.

        self._arm_client.send_goal_async(goal).add_done_callback(on_goal_response)

        # Poll joint states until all joints converge.
        # This is the sole success criterion — it works even when the sim
        # controller sends an Abort before joints fully settle.
        max_wait      = 90.0    # s — must exceed tuck duration (20s traj + sim slowdown)
        poll_interval = 0.5
        elapsed       = 0.0
        _logged_names = False
        time.sleep(0.5)   # wait for goal acceptance before first poll

        while elapsed < max_wait:
            if goal_rejected.is_set():
                return False

            with self._joint_state_lock:
                current = dict(self._latest_joint_positions)

            if current:
                # Log joint name mismatch once so we can diagnose naming issues.
                if not _logged_names:
                    _logged_names = True
                    known = set(current.keys())
                    wanted = set(target_names)
                    missing = wanted - known
                    if missing:
                        self.get_logger().warn(
                            f"[TiagoAdapter] Joint name mismatch for '{step.label}': "
                            f"wanted={sorted(wanted)}, "
                            f"missing={sorted(missing)}, "
                            f"available_sample={sorted(known)[:10]}"
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

        with self._joint_state_lock:
            current = dict(self._latest_joint_positions)
        sample = {n: round(current.get(n, float("nan")), 3) for n in target_names}
        self.get_logger().error(
            f"[TiagoAdapter] Arm timeout after {max_wait:.0f}s for '{step.label}'. "
            f"Final joint errors: { {n: round(abs(current.get(n,float('nan'))-t),3) for n,t in zip(target_names,target_pos)} }"
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

        finished = self._gripper_done.wait(timeout=15.0)
        if not finished:
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
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone TiagoAdapter pick-and-place test."
    )
    parser.add_argument(
        "--robot", default="tiago_robot1", metavar="NAME",
        help="Robot namespace / Gazebo model name (default: tiago_robot1).",
    )
    parser.add_argument(
        "--pick", nargs=3, type=float, metavar=("X", "Y", "Z"),
        default=[2.0, 1.5, 0.8],
        help="Pick position in world frame (m). Default: 2.0 1.5 0.8",
    )
    parser.add_argument(
        "--place", nargs=3, type=float, metavar=("X", "Y", "Z"),
        default=[4.0, 2.0, 0.8],
        help="Place position in world frame (m). Default: 4.0 2.0 0.8",
    )
    parser.add_argument(
        "--arm-base-z", type=float, default=0.83, metavar="M",
        help="Height of arm_1_link above floor (m). Default: 0.83.",
    )
    parser.add_argument(
        "--timeout", type=float, default=600.0,
        help="Max seconds to wait for task completion. Default: 600.",
    )
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
        # Signal background threads to stop before destroying nodes.
        # The discovery thread may call create_subscription() — stopping it
        # first prevents calling ROS APIs on a half-destroyed node (SIGABRT).
        adapter._stop_discovery.set()
        adapter._pose_ready.set()   # unblock any waiting pick_and_place thread

        # Give daemon threads and pending ROS callbacks time to wind down
        # before tearing down the executor and nodes.
        time.sleep(1.0)
        executor.shutdown(timeout_sec=2.0)
        try:
            adapter._gripper.destroy_node()
        except Exception:
            pass
        try:
            adapter.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
