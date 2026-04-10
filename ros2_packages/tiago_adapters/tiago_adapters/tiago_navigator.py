"""
TiagoNavigator
==============
Standalone ROS2 node that drives the Tiago mobile base to goal poses using
a simple proportional controller on cmd_vel_unstamped.  No Nav2, no map,
no SLAM required.  The environment is assumed obstacle-free.

Coordinate frame
----------------
All poses are (x, y, theta) in the WORLD frame (= odometry frame when the
robot spawns at the origin).  This matches the frame used by TiagoAdapter
and TiagoPickPlacePlanner.

Public API
----------
    navigator.drive_to(x, y, theta, callback)
        Drive to an explicit world-frame pose, then call
        callback(success: bool, message: str).

    navigator.drive_to_reach(target_xyz, callback)
        Compute the nearest base pose from which target_xyz is within the
        arm's reach envelope, then drive there.  Uses the same approach-angle
        search as TiagoPickPlacePlanner._compute_nav_pose.

    navigator.cancel()
        Abort the current goal (best-effort).

    navigator.is_busy  -> bool

Typical usage
-------------
    nav = TiagoNavigator(robot_name="tiago_robot1")
    executor.add_node(nav)

    nav.drive_to_reach(
        target_xyz = (2.0, 3.0, 0.875),
        callback   = lambda ok, msg: print(ok, msg),
    )
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional, Tuple

import rclpy
import rclpy.parameter
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ            = Tuple[float, float, float]
NavPose        = Tuple[float, float, float]   # (x, y, theta)
ResultCallback = Callable[[bool, str], None]

# ---------------------------------------------------------------------------
# Controller constants  (mirror TiagoAdapter for consistency)
# ---------------------------------------------------------------------------

_NAV_RATE_HZ    = 10.0    # Hz
_GOAL_XY_TOL    = 0.15    # m   — position tolerance
_GOAL_THETA_TOL = 0.08    # rad — heading tolerance (~5°)
_MAX_LINEAR     = 0.60    # m/s  (was 0.30)
_MAX_ANGULAR    = 1.20    # rad/s (was 0.80)
_K_LINEAR       = 0.80    # proportional gain: linear (was 0.50)
_K_ANGULAR      = 2.00    # proportional gain: angular (was 1.50)
_NAV_TIMEOUT    = 120.0   # s

# ---------------------------------------------------------------------------
# Arm reach envelope  (must match TiagoPickPlacePlanner)
# ---------------------------------------------------------------------------

_REACH_MAX_HORIZ = 0.75   # m
_REACH_MIN_HORIZ = 0.12   # m
_Z_ARM_MAX       =  0.55  # m
_Z_ARM_MIN       = -0.40  # m
_PREFERRED_REACH =  0.50  # m
_N_APPROACH_ANGLES = 12


# ---------------------------------------------------------------------------
# TiagoNavigator
# ---------------------------------------------------------------------------

class TiagoNavigator(Node):
    """
    Parameters
    ----------
    robot_name : str
        Namespace / Gazebo model name (e.g. "tiago_robot1").
    arm_base_z : float
        Height of arm_1_link above the floor (m).  Used only by
        drive_to_reach() to check whether a candidate nav pose gives
        the arm access to the target.  Default 0.83.
    preferred_reach : float
        Preferred horizontal arm extension when computing approach poses (m).
    node_name : str, optional
        ROS2 node name override.
    """

    def __init__(
        self,
        robot_name:      str,
        arm_base_z:      float         = 0.83,
        preferred_reach: float         = _PREFERRED_REACH,
        node_name:       Optional[str] = None,
    ) -> None:
        node_name = node_name or f"tiago_navigator_{robot_name}"
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
        self._arm_base_z      = arm_base_z
        self._preferred_reach = preferred_reach

        self._robot_pose: NavPose = (0.0, 0.0, 0.0)
        self._pose_lock           = threading.Lock()
        self._pose_ready          = threading.Event()

        self._busy      = False
        self._busy_lock = threading.Lock()
        self._cancelled = False

        # ── Publishers / subscribers ──────────────────────────────────
        self._cmd_vel = self.create_publisher(
            Twist,
            f"/{robot_name}/mobile_base_controller/cmd_vel_unstamped",
            10,
        )
        self.create_subscription(
            Odometry,
            f"/{robot_name}/mobile_base_controller/odom",
            self._odom_callback,
            10,
        )
        self.get_logger().info(f"[TiagoNavigator] Ready for robot '{robot_name}'.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def drive_to(
        self,
        x:        float,
        y:        float,
        theta:    float,
        callback: ResultCallback,
    ) -> bool:
        """
        Drive to world-frame pose (x, y, theta).

        Non-blocking.  callback(success, message) fires on completion.
        Returns False immediately if already navigating.
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn("[TiagoNavigator] Rejected — already navigating.")
                return False
            self._busy = True
        self._cancelled = False

        threading.Thread(
            target  = self._run_nav,
            args    = ((x, y, theta), callback),
            daemon  = True,
        ).start()
        return True

    def drive_to_blocking(
        self,
        x:     float,
        y:     float,
        theta: float,
    ) -> Tuple[bool, str]:
        """
        Blocking version of drive_to().  Waits until goal is reached or
        fails, then returns (success, message).
        """
        done   = threading.Event()
        result = [False, ""]

        def cb(success: bool, message: str) -> None:
            result[0] = success
            result[1] = message
            done.set()

        accepted = self.drive_to(x, y, theta, cb)
        if not accepted:
            return False, "Navigator busy."
        done.wait()
        return result[0], result[1]

    def drive_to_reach(
        self,
        target_xyz: XYZ,
        callback:   ResultCallback,
    ) -> bool:
        """
        Compute the best approach pose for target_xyz and drive there.

        If target_xyz is already within the arm's reach envelope from the
        current pose, callback(True, "already reachable") is fired immediately
        without moving.

        Non-blocking.  Returns False if already navigating.
        """
        if not self._wait_for_pose(timeout=10.0):
            callback(False, "Odometry not available.")
            return False

        robot_pose = self._get_pose()

        if self._is_reachable(target_xyz, robot_pose):
            self.get_logger().info(
                f"[TiagoNavigator] {_fmt(target_xyz)} already reachable — no drive needed."
            )
            callback(True, "Already reachable.")
            return True

        nav_pose = self._compute_approach_pose(target_xyz, robot_pose)
        if nav_pose is None:
            callback(False, f"No reachable approach pose found for {_fmt(target_xyz)}.")
            return False

        self.get_logger().info(
            f"[TiagoNavigator] Approach pose for {_fmt(target_xyz)}: "
            f"({nav_pose[0]:.2f}, {nav_pose[1]:.2f}, "
            f"{math.degrees(nav_pose[2]):.1f}°)"
        )
        return self.drive_to(*nav_pose, callback=callback)

    def drive_to_reach_blocking(self, target_xyz: XYZ) -> Tuple[bool, str]:
        """Blocking version of drive_to_reach()."""
        done   = threading.Event()
        result = [False, ""]

        def cb(success: bool, message: str) -> None:
            result[0] = success
            result[1] = message
            done.set()

        accepted = self.drive_to_reach(target_xyz, cb)
        if not accepted:
            return False, "Navigator busy."
        done.wait()
        return result[0], result[1]

    def cancel(self) -> None:
        """Request cancellation of the current navigation goal."""
        with self._busy_lock:
            if not self._busy:
                return
            self._cancelled = True
        self._stop()
        self.get_logger().info("[TiagoNavigator] Cancellation requested.")

    @property
    def is_busy(self) -> bool:
        with self._busy_lock:
            return self._busy

    def get_pose(self) -> NavPose:
        """Return the current robot pose (x, y, theta) in the world frame."""
        return self._get_pose()

    def wait_for_pose(self, timeout: float = 10.0) -> bool:
        """Block until the first odometry message is received."""
        return self._wait_for_pose(timeout)

    # ------------------------------------------------------------------
    # Navigation loop
    # ------------------------------------------------------------------

    def _run_nav(self, goal: NavPose, callback: ResultCallback) -> None:
        goal_x, goal_y, goal_theta = goal
        dt    = 1.0 / _NAV_RATE_HZ
        start = time.perf_counter()

        if not self._wait_for_pose(timeout=10.0):
            self._finish(callback, False, "Odometry not available.")
            return

        self.get_logger().info(
            f"[TiagoNavigator] Driving to "
            f"({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_theta):.1f}°)."
        )

        while time.perf_counter() - start < _NAV_TIMEOUT:
            if self._cancelled:
                self._stop()
                self._finish(callback, False, "Cancelled.")
                return

            rx, ry, rtheta = self._get_pose()
            dist_xy = math.hypot(goal_x - rx, goal_y - ry)

            # Phase 2 — position reached, align heading
            if dist_xy < _GOAL_XY_TOL:
                theta_err = _wrap(goal_theta - rtheta)
                if abs(theta_err) < _GOAL_THETA_TOL:
                    self._stop()
                    self.get_logger().info(
                        f"[TiagoNavigator] Goal reached "
                        f"({goal_x:.2f}, {goal_y:.2f}, "
                        f"{math.degrees(goal_theta):.1f}°)."
                    )
                    self._finish(callback, True, "Goal reached.")
                    return
                angular = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)
                self._pub_vel(0.0, angular)
                time.sleep(dt)
                continue

            # Phase 1 — drive toward XY goal
            heading   = math.atan2(goal_y - ry, goal_x - rx)
            theta_err = _wrap(heading - rtheta)
            cos_err   = math.cos(theta_err)
            linear    = _clamp(_K_LINEAR * dist_xy * max(cos_err, 0.0), 0.0, _MAX_LINEAR)
            angular   = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)
            self._pub_vel(linear, angular)
            time.sleep(dt)

        self._stop()
        self.get_logger().error(
            f"[TiagoNavigator] Timeout ({_NAV_TIMEOUT:.0f}s) driving to "
            f"({goal_x:.2f}, {goal_y:.2f})."
        )
        self._finish(callback, False, f"Navigation timeout after {_NAV_TIMEOUT:.0f}s.")

    def _finish(self, callback: ResultCallback, success: bool, message: str) -> None:
        with self._busy_lock:
            self._busy = False
        callback(success, message)

    # ------------------------------------------------------------------
    # Reach / approach computation
    # ------------------------------------------------------------------

    def _is_reachable(self, target_xyz: XYZ, robot_pose: NavPose) -> bool:
        x_arm, y_arm, z_arm = self._world_to_arm(target_xyz, robot_pose)
        horiz    = math.sqrt(x_arm ** 2 + y_arm ** 2)
        in_horiz = _REACH_MIN_HORIZ <= horiz <= _REACH_MAX_HORIZ
        in_z     = _Z_ARM_MIN <= z_arm <= _Z_ARM_MAX
        in_front = x_arm > 0
        return in_horiz and in_z and in_front

    def _compute_approach_pose(
        self,
        target_xyz: XYZ,
        robot_pose: NavPose,
    ) -> Optional[NavPose]:
        """
        Return the nearest base pose from which target_xyz is reachable.
        Tries _N_APPROACH_ANGLES evenly-spaced directions, picks the
        closest one that passes the reachability check.
        Falls back to the geometric nearest if none pass.
        """
        tx, ty, _  = target_xyz
        rx, ry, _  = robot_pose
        base_angle = math.atan2(ry - ty, rx - tx)  # start from robot's side

        best: Optional[NavPose] = None
        best_dist = float("inf")

        for i in range(_N_APPROACH_ANGLES):
            angle     = base_angle + i * (2.0 * math.pi / _N_APPROACH_ANGLES)
            nav_x     = tx + self._preferred_reach * math.cos(angle)
            nav_y     = ty + self._preferred_reach * math.sin(angle)
            nav_theta = math.atan2(ty - nav_y, tx - nav_x)
            candidate = (nav_x, nav_y, nav_theta)

            if not self._is_reachable(target_xyz, candidate):
                continue

            d = math.hypot(nav_x - rx, nav_y - ry)
            if d < best_dist:
                best_dist = d
                best      = candidate

        if best is not None:
            return best

        # Geometric fallback — closest position regardless of reachability
        self.get_logger().warn(
            f"[TiagoNavigator] No reachable approach pose for {_fmt(target_xyz)} "
            "— using geometric fallback."
        )
        nav_x     = tx + self._preferred_reach * math.cos(base_angle)
        nav_y     = ty + self._preferred_reach * math.sin(base_angle)
        nav_theta = math.atan2(ty - nav_y, tx - nav_x)
        return (nav_x, nav_y, nav_theta)

    def _world_to_arm(self, xyz_world: XYZ, robot_pose: NavPose) -> XYZ:
        nav_x, nav_y, nav_theta = robot_pose
        dx    = xyz_world[0] - nav_x
        dy    = xyz_world[1] - nav_y
        cos_t = math.cos(nav_theta)
        sin_t = math.sin(nav_theta)
        return (
             dx * cos_t + dy * sin_t,
            -dx * sin_t + dy * cos_t,
             xyz_world[2] - self._arm_base_z,
        )

    # ------------------------------------------------------------------
    # ROS helpers
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        q     = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        with self._pose_lock:
            self._robot_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                theta,
            )
        self._pose_ready.set()

    def _get_pose(self) -> NavPose:
        with self._pose_lock:
            return self._robot_pose

    def _wait_for_pose(self, timeout: float = 10.0) -> bool:
        return self._pose_ready.wait(timeout=timeout)

    def _pub_vel(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self._cmd_vel.publish(twist)

    def _stop(self) -> None:
        self._cmd_vel.publish(Twist())


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _fmt(xyz: XYZ) -> str:
    return f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"