"""
ROS2 adapter for the Pioneer 3AT (4-wheel skid-steer mobile base).
Simulation only.

Unlike TiagoAdapter, the Pioneer has no arm/gripper — this adapter's only
job is driving the base to a goal pose. It follows the same conventions as
TiagoAdapter so the two robot families are interchangeable at the call site:

    * A go-to-pose P-controller (rotate-to-bearing, drive, blend heading
      near the goal, optional final rotate to a target theta) — the same
      "lightweight" control strategy used by TiagoAdapter's nav step,
      just without the arm/gripper machinery around it.
    * Ground-truth world pose preferred (via the gz-ros2 bridge topic
      /world/<world_name>/pose/info), falling back to wheel odometry
      (/<robot_name>/odom) when no world_name is supplied or the bridge
      hasn't published yet. Ground truth avoids odometry drift, exactly
      like TiagoAdapter.
    * Same non-blocking dispatch pattern: navigate_to() returns immediately
      (True if accepted), the goal runs on a background thread, and
      result_callback(success, message) fires on completion.

Command topic
-------------
Twist commands are published on ``/<robot_name>/cmd_vel``, which is the
topic name the pioneer3at_sim.launch.py bridge remaps
``/model/<robot_name>/cmd_vel`` to (see pioneer3at_adapters launch file).

Usage
-----
    adapter = PioneerAdapter(robot_name="pioneer3at_1", mode=RobotMode.SIMULATION,
                              result_callback=on_done, world_name="backyard")
    adapter.navigate_to(2.0, 1.5)                  # position only
    adapter.navigate_to(2.0, 1.5, theta=1.57)       # position + final heading
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
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

NavPose        = Tuple[float, float, float]   # (x, y, theta)
ResultCallback = Callable[[bool, str], None]


class RobotMode(str, Enum):
    SIMULATION = "sim"
    REAL_WORLD = "real"


# ---------------------------------------------------------------------------
# Navigation constants
# ---------------------------------------------------------------------------
# Mirrors TiagoAdapter's nav-loop gains/tolerances, retuned to the limits
# declared in the Pioneer's DiffDrive plugin (max_linear_velocity=0.7,
# max_angular_velocity=2.0 in model.sdf) so commanded velocities are never
# clamped by Gazebo silently.

_NAV_RATE_HZ      = 10.0
_GOAL_XY_TOL      = 0.15    # m   — position tolerance
_GOAL_THETA_TOL   = 0.05    # rad (~2.9°) — final heading tolerance
_ROTATE_TOL       = 0.05    # rad — bearing lock tolerance before driving straight
_MAX_LINEAR       = 0.7     # m/s — matches model.sdf DiffDrive max_linear_velocity
_MAX_ANGULAR      = 2.0     # rad/s — matches model.sdf DiffDrive max_angular_velocity
_K_LINEAR         = 1.0
_K_ANGULAR        = 2.0
_HEADING_BLEND_DIST = 0.5   # m — start blending toward final theta within this range
_NAV_TIMEOUT      = 120.0   # s — default; overridable per-call


class PioneerAdapter(Node):
    """
    Parameters
    ----------
    robot_name : str
        Namespace / spawned Gazebo model name (e.g. "pioneer3at_1"). Must
        match the ``-name`` used at spawn time so the bridged topics line up.
    mode : RobotMode
        Currently only RobotMode.SIMULATION is supported.
    result_callback : ResultCallback
        Called on task completion: callback(success: bool, message: str).
    world_name : str, optional
        Gazebo world name. When supplied, the adapter consumes ground-truth
        pose from /world/<world_name>/pose/info instead of wheel odometry.
    max_linear, max_angular : float
        Velocity caps (m/s, rad/s). Defaults match the Pioneer's DiffDrive
        plugin limits.
    node_name : str, optional
        ROS2 node name override.
    """

    def __init__(
        self,
        robot_name:      str,
        mode:            RobotMode,
        result_callback: ResultCallback,
        world_name:      Optional[str] = None,
        max_linear:      float         = _MAX_LINEAR,
        max_angular:     float         = _MAX_ANGULAR,
        node_name:       Optional[str] = None,
    ) -> None:
        node_name = node_name or f"pioneer_adapter_{robot_name}"
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
        self._world_name      = world_name
        self._max_linear      = max_linear
        self._max_angular     = max_angular

        self._cancelled = False
        self._busy      = False
        self._busy_lock = threading.Lock()

        self._robot_pose_odom:  NavPose = (0.0, 0.0, 0.0)
        self._robot_pose_world: NavPose = (0.0, 0.0, 0.0)
        self._use_ground_truth = False
        self._robot_pose_lock  = threading.Lock()
        self._pose_ready       = threading.Event()

        self.get_logger().info(f"[PioneerAdapter] Initialising ({mode.value.upper()}) ...")
        self._init_ros()
        self.get_logger().info("[PioneerAdapter] Ready.")

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _init_ros(self) -> None:
        self._cmd_vel = self.create_publisher(
            Twist, f"/{self._robot_name}/cmd_vel", 10,
        )

        self.create_subscription(
            Odometry, f"/{self._robot_name}/odom", self._odom_callback, 10,
        )

        if self._world_name:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            topic = f"/world/{self._world_name}/pose/info"
            self.create_subscription(TFMessage, topic, self._gz_pose_cb, qos)
            self.get_logger().info(
                f"[PioneerAdapter] Subscribed to Gazebo ground truth: '{topic}'."
            )
        else:
            self.get_logger().warn(
                "[PioneerAdapter] No world_name supplied — using wheel odometry "
                "(will drift over distance). Pass world_name='<gz_world>' to use "
                "ground-truth pose from the gz-ros2 bridge."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def navigate_to(
        self,
        x:       float,
        y:       float,
        theta:   Optional[float] = None,
        timeout: float           = _NAV_TIMEOUT,
    ) -> bool:
        """
        Drive to world-frame (x, y[, theta]).

        Non-blocking — returns True if accepted, False if already busy.
        result_callback fires on completion with (success, message).
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn("[PioneerAdapter] Rejected — already busy.")
                return False
            self._busy      = True
            self._cancelled = False

        if not self._pose_ready.wait(timeout=15.0):
            src = (f"/world/{self._world_name}/pose/info"
                   if self._world_name else f"/{self._robot_name}/odom")
            self.get_logger().error(
                f"[PioneerAdapter] No pose within 15 s from '{src}'. "
                "Check the gz-ros2 bridge is running and the robot is spawned."
            )
            with self._busy_lock:
                self._busy = False
            self._result_callback(False, "Robot pose unavailable.")
            return True

        threading.Thread(
            target=self._run_nav,
            args=(x, y, theta, timeout),
            daemon=True,
        ).start()
        return True

    def cancel(self) -> None:
        with self._busy_lock:
            if not self._busy:
                return
            self._cancelled = True
        self._stop_base()
        self.get_logger().info("[PioneerAdapter] Cancellation requested.")

    @property
    def is_busy(self) -> bool:
        with self._busy_lock:
            return self._busy

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        q     = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)
        with self._robot_pose_lock:
            self._robot_pose_odom = (
                msg.pose.pose.position.x, msg.pose.pose.position.y, theta,
            )
        if not self._world_name:
            self._pose_ready.set()

    def _gz_pose_cb(self, msg: TFMessage) -> None:
        """Ground-truth base pose from the Gazebo Fortress gz-ros2 bridge.

        Only the top-level model name (== robot_name at spawn time) is
        used as the disambiguator, matching TiagoAdapter's approach — safe
        for multi-robot scenes since ros_gz_bridge maps entity names into
        child_frame_id without any namespace prefix.
        """
        for tfs in msg.transforms:
            if tfs.child_frame_id != self._robot_name:
                continue

            t   = tfs.transform.translation
            q   = tfs.transform.rotation
            yaw = 2.0 * math.atan2(q.z, q.w)

            with self._robot_pose_lock:
                self._robot_pose_world = (t.x, t.y, yaw)
                if not self._use_ground_truth:
                    self._use_ground_truth = True
                    self.get_logger().info(
                        f"[PioneerAdapter] Ground-truth pose active "
                        f"({t.x:+.3f}, {t.y:+.3f}, {math.degrees(yaw):+.1f} deg)."
                    )
            self._pose_ready.set()
            return

    def _get_robot_pose(self) -> NavPose:
        """Current robot pose in the WORLD frame (ground truth preferred).

        Note: unlike TiagoAdapter, odometry here is not corrected back into
        a world frame via a spawn-pose offset — if no world_name is given,
        the "world" frame IS the odom frame (equivalent to spawning at the
        world origin with zero yaw). Pass world_name whenever the robot is
        spawned away from the origin.
        """
        with self._robot_pose_lock:
            if self._use_ground_truth:
                return self._robot_pose_world
            return self._robot_pose_odom

    # ------------------------------------------------------------------
    # Go-to-pose P-controller (lightweight — no external planner/costmap)
    # ------------------------------------------------------------------

    def _run_nav(
        self, gx: float, gy: float, gtheta: Optional[float], timeout: float,
    ) -> None:
        dt    = 1.0 / _NAV_RATE_HZ
        start = time.perf_counter()

        self.get_logger().info(
            f"[PioneerAdapter] Navigating to "
            f"({gx:.2f}, {gy:.2f}"
            + (f", {math.degrees(gtheta):.1f}°)" if gtheta is not None else ")")
        )

        locked_bearing: Optional[float] = None

        while time.perf_counter() - start < timeout:
            if self._cancelled:
                self._stop_base()
                self._finish(False, "Navigation cancelled.")
                return

            rx, ry, rtheta = self._get_robot_pose()
            dist_xy = math.hypot(gx - rx, gy - ry)

            # ── Close enough in XY: finish, or settle to final heading ──
            if dist_xy < _GOAL_XY_TOL:
                if gtheta is None:
                    self._stop_base()
                    self._finish(True, f"Reached ({gx:.2f}, {gy:.2f}).")
                    return
                theta_err = _wrap_angle(gtheta - rtheta)
                if abs(theta_err) < _GOAL_THETA_TOL:
                    self._stop_base()
                    self._finish(
                        True,
                        f"Reached ({gx:.2f}, {gy:.2f}, {math.degrees(gtheta):.1f}°).",
                    )
                    return
                angular = _clamp(_K_ANGULAR * theta_err, -self._max_angular, self._max_angular)
                self._publish_vel(0.0, angular)
                time.sleep(dt)
                continue

            # ── Rotate-to-bearing before driving straight ──
            bearing_to_goal = math.atan2(gy - ry, gx - rx)
            bearing_err_raw = _wrap_angle(bearing_to_goal - rtheta)
            if locked_bearing is None:
                if abs(bearing_err_raw) > _ROTATE_TOL:
                    angular = _clamp(
                        _K_ANGULAR * bearing_err_raw, -self._max_angular, self._max_angular,
                    )
                    self._publish_vel(0.0, angular)
                    time.sleep(dt)
                    continue
                locked_bearing = bearing_to_goal

            if abs(bearing_err_raw) > math.pi / 2:
                # Drifted far off bearing (e.g. overshoot) — re-rotate.
                locked_bearing = None
                self._publish_vel(
                    0.0, _clamp(_K_ANGULAR * bearing_err_raw, -self._max_angular, self._max_angular)
                )
                time.sleep(dt)
                continue

            # ── Drive straight, blending toward final theta near the goal ──
            if gtheta is not None and dist_xy < _HEADING_BLEND_DIST:
                alpha   = dist_xy / _HEADING_BLEND_DIST
                blend_x = alpha * math.cos(bearing_to_goal) + (1.0 - alpha) * math.cos(gtheta)
                blend_y = alpha * math.sin(bearing_to_goal) + (1.0 - alpha) * math.sin(gtheta)
                steering_err = _wrap_angle(math.atan2(blend_y, blend_x) - rtheta)
            else:
                steering_err = bearing_err_raw

            linear = _clamp(
                _K_LINEAR * dist_xy * max(0.0, math.cos(steering_err)),
                0.0, self._max_linear,
            )
            angular = _clamp(_K_ANGULAR * steering_err, -self._max_angular, self._max_angular)
            self._publish_vel(linear, angular)
            time.sleep(dt)

        self._stop_base()
        self._finish(False, f"Navigation timeout after {timeout:.0f}s.")

    def _publish_vel(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self._cmd_vel.publish(twist)

    def _stop_base(self) -> None:
        self._cmd_vel.publish(Twist())

    def _finish(self, success: bool, message: str) -> None:
        icon   = "✓" if success else "✗"
        logger = self.get_logger()
        if success:
            logger.info(f"[PioneerAdapter] {icon} {message}")
        else:
            logger.error(f"[PioneerAdapter] {icon} {message}")
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
    parser = argparse.ArgumentParser(description="Standalone PioneerAdapter test.")
    parser.add_argument("--robot", default="pioneer3at_1")
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), default=[2.0, 0.0])
    parser.add_argument("--theta", type=float, default=None, help="Optional final heading (rad).")
    parser.add_argument("--world-name", default=None, dest="world_name")
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()

    done_event = threading.Event()
    result_box: list = []

    def on_result(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    adapter = PioneerAdapter(
        robot_name      = args.robot,
        mode            = RobotMode.SIMULATION,
        result_callback = on_result,
        world_name      = args.world_name,
    )

    # Use an explicit executor (not bare rclpy.spin) so shutdown can be
    # sequenced cleanly: stop the executor *before* destroying the node,
    # otherwise the spin thread can still be touching the node's C-level
    # resources while the main thread frees them underneath it — that's a
    # use-after-free at the rcl layer and shows up as a bare
    # "terminate called without an active exception" / SIGABRT with no
    # Python traceback.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(adapter)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        gx, gy = args.goal
        accepted = adapter.navigate_to(gx, gy, theta=args.theta, timeout=args.timeout)
        if not accepted:
            print("[standalone] Task rejected — adapter is busy.")
        else:
            print(f"[standalone] Task accepted. Waiting up to {args.timeout}s ...")
            finished = done_event.wait(timeout=args.timeout + 5.0)
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
        # Order matters: stop the executor (unblocks/ends the spin thread),
        # join it, THEN destroy the node, THEN shut down the context.
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        adapter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
