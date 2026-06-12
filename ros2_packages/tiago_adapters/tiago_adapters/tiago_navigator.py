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
    navigator.navigate_to(target_x, target_y, callback, target_z=0.0)
        **Preferred entry point for coordinate-based navigation.**
        Given the world-frame (x, y) of the target object/point, computes
        the nearest approach pose and drives there, always facing the target.
        Phase 3 (final alignment) corrects heading from the robot's *actual*
        stopped position, eliminating up to ~12° of orientation error from
        the XY stopping tolerance.  Also accepts target_z for arm-reach
        validation.

    navigator.navigate_to_blocking(target_x, target_y, target_z=0.0)
        Blocking version of navigate_to().

    navigator.drive_to(x, y, theta, callback, nav_target_xy=None)
        Drive to an explicit world-frame pose.  When nav_target_xy=(tx, ty)
        is supplied, Phase 3 recomputes the final heading from the robot's
        actual stopped position toward (tx, ty), eliminating orientation error
        from the XY stopping tolerance.  If omitted, a virtual far-target
        along `theta` is used (< 0.2° residual error).

    navigator.drive_to_reach(target_xyz, callback)
        Compute the nearest base pose from which target_xyz is within the
        arm's reach envelope, then drive there.  Uses the same approach-angle
        search as TiagoPickPlacePlanner._compute_nav_pose.

    navigator.cancel()
        Abort the current goal (best-effort).

    navigator.is_busy  -> bool

Orientation guarantee
---------------------
All navigation methods ensure that the robot's final heading points toward
the navigation target within _GOAL_THETA_TOL (~4.6°, well under the 5°
requirement).  This is achieved by Phase 3 of the P-controller loop, which
recomputes the bearing from the robot's *actual* stopped position to the
target rather than using a pre-planned heading.

Typical usage
-------------
    nav = TiagoNavigator(robot_name="tiago_robot1")
    executor.add_node(nav)

    # Object-name workflow (via ObjectToRobot):
    result = obj_transformer.get_pose("tomato_1")
    pos    = result["world_pose"].position
    nav.navigate_to(pos.x, pos.y, lambda ok, msg: print(ok, msg), target_z=pos.z)

    # Explicit world coordinates:
    nav.navigate_to(2.0, 3.0, lambda ok, msg: print(ok, msg))

    # Low-level: drive to pre-computed approach pose, heading toward (tx, ty):
    nav.drive_to(nav_x, nav_y, nav_theta, callback, nav_target_xy=(tx, ty))
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
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sdf_table_resolver import SdfTableResolver

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
_MAX_LINEAR     = 2    # m/s
_MAX_ANGULAR    = 1.60    # rad/s
_K_LINEAR       = 1.20    # proportional gain: linear
_K_ANGULAR      = 2.40    # proportional gain: angular
_ROTATE_TOL = 0.03        # rad (~1.7°) — bearing lock tolerance before driving straight
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
    world_name : str, optional
        Gazebo world name (e.g. "backyard").  When supplied, the navigator
        subscribes to ``/world/<world_name>/pose/info`` (published by the
        gz-ros2 bridge) and uses the *ground-truth* world pose of the robot
        for closed-loop control.  This eliminates wheel-odometry drift —
        critical for accurate pick-and-place after a long navigation.

        Without this argument the navigator falls back to wheel odometry,
        which accumulates drift (typically several centimetres and a few
        degrees over a few metres of travel in Gazebo Fortress).
    """

    def __init__(
        self,
        robot_name:       str,
        arm_base_z:       float         = 0.83,
        preferred_reach:  float         = _PREFERRED_REACH,
        node_name:        Optional[str] = None,
        world_sdf_path:   Optional[str] = None,
        models_base_dir:  Optional[str] = None,
        table_standoff:   float         = 0.10,
        spawn_world_pose: Optional[Tuple[float, float, float]] = None,
        world_name:       Optional[str] = None,
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
        self._world_name      = world_name

        # Spawn pose in world frame: odom always starts at (0,0,0) at spawn.
        # Used only when ground-truth pose is unavailable (odometry mode).
        if spawn_world_pose is not None:
            self._spawn_x, self._spawn_y, self._spawn_yaw = spawn_world_pose
        else:
            self._spawn_x, self._spawn_y, self._spawn_yaw = 0.0, 0.0, 0.0

        # Table-aware approach pose computation
        self._table_resolver: Optional[SdfTableResolver] = None
        if world_sdf_path:
            self._table_resolver = SdfTableResolver(
                sdf_path        = world_sdf_path,
                base_standoff   = table_standoff,
                models_base_dir = models_base_dir,
            )
            self.get_logger().info(
                f"[TiagoNavigator] Table resolver loaded "
                f"({len(self._table_resolver.known_tables())} models)."
            )

        # ── Pose state ────────────────────────────────────────────────
        # Two sources, kept in lock-step where possible:
        #   _robot_pose_odom   — wheel-odometry frame (raw odom, drifts)
        #   _robot_pose_world  — Gazebo ground truth (only when world_name set)
        # _use_ground_truth flips True after the first pose/info message
        # arrives.  Until then we use odometry so the node still works even
        # if the gz bridge is slow to publish.
        self._robot_pose_odom:  NavPose = (0.0, 0.0, 0.0)
        self._robot_pose_world: NavPose = (
            self._spawn_x, self._spawn_y, self._spawn_yaw,
        )
        self._use_ground_truth = False
        self._pose_lock        = threading.Lock()
        self._pose_ready       = threading.Event()
        # Compatibility alias for any caller that reaches into _robot_pose
        # directly — kept up-to-date by the odom callback so code paths that
        # still use it (like internal nav loop reads) keep working in odom mode.
        self._robot_pose: NavPose = self._robot_pose_odom

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

        # ── Ground-truth pose from Gazebo Fortress (gz-ros2 bridge) ───
        if world_name:
            # The bridge publishes with BEST_EFFORT QoS — must match.
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            topic = f"/world/{world_name}/pose/info"
            self.create_subscription(TFMessage, topic, self._gz_pose_cb, qos)
            self.get_logger().info(
                f"[TiagoNavigator] Subscribed to Gazebo ground truth: '{topic}'."
            )
        else:
            self.get_logger().warn(
                "[TiagoNavigator] No world_name supplied — using wheel odometry "
                "(will drift over distance).  Pass world_name='<gz_world>' to use "
                "Gazebo ground-truth pose."
            )

        self.get_logger().info(f"[TiagoNavigator] Ready for robot '{robot_name}'.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def drive_to(
        self,
        x:              float,
        y:              float,
        theta:          float,
        callback:       ResultCallback,
        nav_target_xy:  Optional[Tuple[float, float]] = None,
    ) -> bool:
        """
        Drive to world-frame pose (x, y, theta).

        nav_target_xy : (tx, ty), optional
            World-frame XY of the object the robot should face on arrival.
            When supplied, Phase 3 recomputes the final heading from the
            robot's *actual* stopped odom position toward (tx, ty).  This
            corrects up to ~arcsin(_GOAL_XY_TOL / preferred_reach) ≈ 12° of
            orientation error caused by the XY stopping tolerance.

            When omitted, a virtual target 50 m along `theta` from the goal
            position is used instead, keeping residual Phase-3 heading error
            below 0.2° for any XY stop offset < _GOAL_XY_TOL.

            **Always supply nav_target_xy when navigating toward a known
            object or point — use navigate_to() for the simplest interface.**

        Non-blocking.  callback(success, message) fires on completion.
        Returns False immediately if already navigating.
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn("[TiagoNavigator] Rejected — already navigating.")
                return False
            self._busy = True
        self._cancelled = False

        # Pack nav_target_xy as optional 4th element so _run_nav can use it
        # without changing the NavPose type alias.
        goal = (x, y, theta, nav_target_xy)

        threading.Thread(
            target  = self._run_nav,
            args    = (goal, callback),
            daemon  = True,
        ).start()
        return True

    def drive_to_blocking(
        self,
        x:             float,
        y:             float,
        theta:         float,
        nav_target_xy: Optional[Tuple[float, float]] = None,
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

        accepted = self.drive_to(x, y, theta, cb, nav_target_xy=nav_target_xy)
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

        robot_pose = self._get_world_pose()

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
        return self.drive_to(
            nav_pose[0], nav_pose[1], nav_pose[2],
            callback      = callback,
            nav_target_xy = (target_xyz[0], target_xyz[1]),
        )

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

    # ------------------------------------------------------------------
    # Coordinate-based navigation  (explicit world-frame target)
    # ------------------------------------------------------------------

    def navigate_to(
        self,
        target_x:  float,
        target_y:  float,
        callback:  ResultCallback,
        target_z:  float = 0.0,
    ) -> bool:
        """
        Navigate to face an explicit world-frame (x, y) target.

        This is the correct entry point when the caller knows the *target*
        coordinates (e.g. from ObjectToRobot.get_pose()["world_pose"]) rather
        than having a pre-computed approach pose.

        The method:
          1. Computes an approach pose at ``preferred_reach`` from the target,
             on the side closest to the robot, so the arm can reach the target.
          2. Calls ``drive_to`` with ``nav_target_xy=(target_x, target_y)`` so
             Phase 3 recomputes the final heading from the robot's *actual*
             stopped position.  This eliminates up to ~12° of orientation error
             that would arise from using a pre-planned heading with a 0.15 m
             XY stopping tolerance.

        Parameters
        ----------
        target_x, target_y : float
            World-frame XY of the object or point the robot must face.
        callback : callable
            ``callback(success: bool, message: str)`` fired on completion.
        target_z : float
            World-frame Z used for arm-reach validation (default 0.0 — the
            reachability check is skipped for z outside the arm's Z envelope).

        Non-blocking.  Returns False if already navigating.

        Example
        -------
            # Object name → world pose via ObjectToRobot:
            result = obj_transformer.get_pose("tomato_1")
            if "error" not in result:
                pos = result["world_pose"].position
                navigator.navigate_to(pos.x, pos.y, callback, target_z=pos.z)

            # Explicit world coordinates:
            navigator.navigate_to(2.5, 1.0, callback)
        """
        target_xyz: XYZ = (target_x, target_y, target_z)

        if not self._wait_for_pose(timeout=10.0):
            callback(False, "Odometry not available.")
            return False

        robot_pose = self._get_world_pose()

        if self._is_reachable(target_xyz, robot_pose):
            self.get_logger().info(
                f"[TiagoNavigator] ({target_x:.2f}, {target_y:.2f}) already "
                "reachable — no drive needed."
            )
            callback(True, "Already reachable.")
            return True

        nav_pose = self._compute_approach_pose(target_xyz, robot_pose)
        if nav_pose is None:
            callback(
                False,
                f"No reachable approach pose for ({target_x:.2f}, {target_y:.2f}).",
            )
            return False

        self.get_logger().info(
            f"[TiagoNavigator] navigate_to ({target_x:.2f}, {target_y:.2f}): "
            f"approach ({nav_pose[0]:.2f}, {nav_pose[1]:.2f}, "
            f"{math.degrees(nav_pose[2]):.1f}°)."
        )
        return self.drive_to(
            nav_pose[0], nav_pose[1], nav_pose[2],
            callback      = callback,
            nav_target_xy = (target_x, target_y),  # always set → Phase 3 corrects heading
        )

    def navigate_to_blocking(
        self,
        target_x: float,
        target_y: float,
        target_z: float = 0.0,
    ) -> Tuple[bool, str]:
        """
        Blocking version of navigate_to().

        Example
        -------
            ok, msg = navigator.navigate_to_blocking(2.5, 1.0)
            if not ok:
                print(f"Navigation failed: {msg}")
        """
        done   = threading.Event()
        result = [False, ""]

        def cb(success: bool, message: str) -> None:
            result[0] = success
            result[1] = message
            done.set()

        accepted = self.navigate_to(target_x, target_y, cb, target_z=target_z)
        if not accepted:
            return False, "Navigator busy."
        done.wait()
        return result[0], result[1]

    def drive_to_table(
        self,
        object_xyz: XYZ,
        callback:   ResultCallback,
        table_name: Optional[str] = None,
    ) -> bool:
        """
        Table-aware navigation: drive to a standoff pose beside the table
        that holds `object_xyz`, using pre-approach waypoints if the direct
        path is blocked by another table.

        If no SDF resolver is configured, or the table cannot be identified,
        falls back to ``drive_to_reach()``.

        Parameters
        ----------
        object_xyz : (x, y, z)
            World-frame position of the target object.
        callback : callable
            ``callback(success: bool, message: str)`` fired on completion.
        table_name : str, optional
            Override automatic table detection (useful when the object
            may not yet be on the table at plan time).

        Non-blocking.  Returns False if already navigating.
        """
        if self._table_resolver is None:
            return self.drive_to_reach(object_xyz, callback)

        if not self._wait_for_pose(timeout=10.0):
            callback(False, "Odometry not available.")
            return False

        robot_pose = self._get_world_pose()
        rx, ry, _  = robot_pose
        tx, ty, _  = object_xyz

        t_name = table_name or self._table_resolver.find_table_for_object((tx, ty))
        if t_name is None:
            self.get_logger().warn(
                f"[TiagoNavigator] No table found for object at "
                f"({tx:.2f}, {ty:.2f}) — falling back to drive_to_reach."
            )
            return self.drive_to_reach(object_xyz, callback)

        waypoints = self._table_resolver.approach_waypoints(
            object_xy  = (tx, ty),
            robot_xy   = (rx, ry),
            table_name = t_name,
        )

        if not waypoints:
            self.get_logger().warn(
                f"[TiagoNavigator] Table resolver returned no waypoints "
                f"for '{t_name}' — falling back to drive_to_reach."
            )
            return self.drive_to_reach(object_xyz, callback)

        self.get_logger().info(
            f"[TiagoNavigator] Table-aware nav to '{t_name}': "
            f"{len(waypoints)} waypoint(s)."
        )

        # Chain the waypoints: each drives to the next, with the final
        # callback firing when the last waypoint is reached.
        threading.Thread(
            target  = self._run_waypoint_chain,
            args    = (waypoints, object_xyz, callback),
            daemon  = True,
        ).start()
        return True

    def drive_to_table_blocking(
        self,
        object_xyz: XYZ,
        table_name: Optional[str] = None,
    ) -> Tuple[bool, str]:
        """Blocking version of drive_to_table()."""
        done   = threading.Event()
        result = [False, ""]

        def cb(success: bool, message: str) -> None:
            result[0] = success
            result[1] = message
            done.set()

        accepted = self.drive_to_table(object_xyz, cb, table_name=table_name)
        if not accepted:
            return False, "Navigator busy."
        done.wait()
        return result[0], result[1]

    def _run_waypoint_chain(
        self,
        waypoints:  list,
        object_xyz: XYZ,
        callback:   ResultCallback,
    ) -> None:
        """
        Sequentially drive through each waypoint.
        The last waypoint is the final approach pose; earlier ones are
        intermediate aisle positions.
        """
        with self._busy_lock:
            if self._busy:
                callback(False, "Navigator busy.")
                return
            self._busy = True
        self._cancelled = False

        for idx, wp in enumerate(waypoints):
            is_last = (idx == len(waypoints) - 1)
            label   = f"waypoint {idx + 1}/{len(waypoints)}"
            self.get_logger().info(
                f"[TiagoNavigator] Driving to {label}: "
                f"({wp[0]:.2f}, {wp[1]:.2f}, {math.degrees(wp[2]):.1f}°)."
            )

            done_ev = threading.Event()
            result  = [False, ""]

            # Release busy lock briefly so _run_nav can re-acquire it.
            # We hold the chain lock here via the thread itself.
            with self._busy_lock:
                self._busy = False   # temporarily — _run_nav will set it again

            def _cb(s, m, _ev=done_ev, _r=result):
                _r[0] = s
                _r[1] = m
                _ev.set()

            # Only pass nav_target_xy on the final waypoint so the robot
            # corrects its heading to face the object from its actual stopped
            # position.  Intermediate waypoints use their own baked-in theta.
            target_xy = (object_xyz[0], object_xyz[1]) if is_last else None
            accepted  = self.drive_to(
                wp[0], wp[1], wp[2],
                callback      = _cb,
                nav_target_xy = target_xy,
            )
            if not accepted:
                self._finish(callback, False, f"Navigator rejected {label}.")
                return

            done_ev.wait()

            if not result[0]:
                self._finish(callback, False, f"Failed at {label}: {result[1]}")
                return

            if self._cancelled:
                self._finish(callback, False, "Cancelled.")
                return

        # All waypoints reached
        with self._busy_lock:
            self._busy = False
        callback(True, "Table approach complete.")

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
        return self._get_world_pose()

    def wait_for_pose(self, timeout: float = 10.0) -> bool:
        """Block until the first odometry message is received."""
        return self._wait_for_pose(timeout)

    # ------------------------------------------------------------------
    # Navigation loop
    # ------------------------------------------------------------------

    def _run_nav(self, goal: tuple, callback: ResultCallback) -> None:
        # goal[0:3] = world-frame (x, y, theta); goal[3] = optional nav_target_xy (world frame)
        world_x, world_y, world_theta = goal[0], goal[1], goal[2]
        nav_target_xy_world = goal[3] if len(goal) > 3 else None

        # Convert world goal → odom once.  The P-controller runs entirely in odom.
        goal_x, goal_y, goal_theta = self._world_to_odom(world_x, world_y, world_theta)

        # Pre-convert nav_target_xy to odom so Phase 3 atan2 is in odom frame.
        # When nav_target_xy_world is not provided we synthesise a virtual
        # far-point in the goal-heading direction.  This makes Phase 3
        # recompute the effective bearing from the robot's *actual* stopped
        # position rather than using the pre-planned goal_theta, removing up
        # to ~arcsin(_GOAL_XY_TOL / preferred_reach) ≈ 12° of heading error
        # caused by the XY stopping tolerance.
        _VIRTUAL_TARGET_DIST = 50.0   # m — large enough that XY offset < 0.3°
        if nav_target_xy_world is not None:
            tx_o, ty_o, _ = self._world_to_odom(
                nav_target_xy_world[0], nav_target_xy_world[1], 0.0
            )
        else:
            # No explicit target: project goal heading from the goal XY so the
            # effective bearing is computed from the actual stopped position.
            # At 50 m the angular error from a 0.15 m XY offset is < 0.2°.
            vt_w_x = world_x + _VIRTUAL_TARGET_DIST * math.cos(world_theta)
            vt_w_y = world_y + _VIRTUAL_TARGET_DIST * math.sin(world_theta)
            tx_o, ty_o, _ = self._world_to_odom(vt_w_x, vt_w_y, 0.0)

        dt    = 1.0 / _NAV_RATE_HZ
        start = time.perf_counter()

        if not self._wait_for_pose(timeout=10.0):
            self._finish(callback, False, "Odometry not available.")
            return

        self.get_logger().info(
            f"[TiagoNavigator] Driving to world "
            f"({world_x:.2f}, {world_y:.2f}, {math.degrees(world_theta):.1f}°) "
            f"→ odom ({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_theta):.1f}°)."
        )

        locked_bearing: Optional[float] = None
        # phase3_locked uses hysteresis to prevent Phase 2 from overriding
        # Phase 3 due to tiny physics drift around the XY tolerance boundary.
        # Entry: dist_xy < _GOAL_XY_TOL (0.15 m)
        # Exit : dist_xy > _GOAL_XY_TOL * 2 (0.30 m) — only if robot drifts far
        phase3_locked = False

        while time.perf_counter() - start < _NAV_TIMEOUT:
            if self._cancelled:
                self._stop()
                self._finish(callback, False, "Cancelled.")
                return

            # All comparisons in ODOM frame
            rx, ry, rtheta = self._get_pose()
            dist_xy = math.hypot(goal_x - rx, goal_y - ry)

            # ── Phase 3 gate (hysteresis) ─────────────────────────────────
            # Once the robot enters the XY tolerance zone we stay in Phase 3
            # until it drifts more than 2× the tolerance away.  This prevents
            # Phase 2 from briefly running (and commanding the wrong rotation
            # direction) when physics noise pushes dist_xy just above the edge.
            prev_locked = phase3_locked
            if dist_xy < _GOAL_XY_TOL:
                phase3_locked = True
            elif dist_xy > _GOAL_XY_TOL * 2.0:
                phase3_locked = False

            # Reset locked_bearing the first time we enter Phase 3 so that if
            # Phase 3 eventually exits, Phase 1/2 start fresh.
            if phase3_locked and not prev_locked:
                locked_bearing = None

            # ── Phase 3 — in-place heading alignment ──────────────────────
            # No _stop() here: we do NOT stop before computing the correction.
            # Calling _stop() on every cycle was causing the wrong-direction
            # rotation symptom — residual Phase-2 angular momentum (opposite
            # to Phase-3 direction) would carry the robot in the wrong
            # direction for ~1 cycle before the Phase-3 command took effect.
            # Instead we let the proportional angular command decelerate and
            # reverse the angular velocity in one smooth motion.
            if phase3_locked:
                effective_theta = math.atan2(ty_o - ry, tx_o - rx)
                theta_err = _wrap(effective_theta - rtheta)
                if abs(theta_err) < _GOAL_THETA_TOL:
                    self._stop()
                    self.get_logger().info(
                        f"[TiagoNavigator] Goal reached: odom "
                        f"({rx:.2f}, {ry:.2f}, {math.degrees(rtheta):.1f}°)."
                    )
                    self._finish(callback, True, "Goal reached.")
                    return
                angular = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)
                self._pub_vel(0.0, angular)
                time.sleep(dt)
                continue

            # ── Phase 1 — initial rotation: face the goal before driving ──
            # Only active while locked_bearing is None (before first lock-in).
            bearing_to_goal = math.atan2(goal_y - ry, goal_x - rx)
            # Raw bearing error used for Phase-1 rotation and the >90° restart
            # check — always tracks the actual direction to the goal XY.
            bearing_err_raw = _wrap(bearing_to_goal - rtheta)
            if locked_bearing is None:
                if abs(bearing_err_raw) > _ROTATE_TOL:
                    angular = _clamp(_K_ANGULAR * bearing_err_raw,
                                     -_MAX_ANGULAR, _MAX_ANGULAR)
                    self._pub_vel(0.0, angular)
                    time.sleep(dt)
                    continue
                locked_bearing = bearing_to_goal

            # ── Phase 2 — unicycle drive toward goal ──────────────────────
            # If the robot is pointing away (>90°), stop forward motion and
            # correct using the raw bearing so Phase 1 re-locks correctly.
            if abs(bearing_err_raw) > math.pi / 2:
                locked_bearing = None
                self._pub_vel(0.0, _clamp(_K_ANGULAR * bearing_err_raw,
                                          -_MAX_ANGULAR, _MAX_ANGULAR))
                time.sleep(dt)
                continue

            # Heading blend: within _HEADING_BLEND_DIST of the goal, smoothly
            # rotate the steering target from bearing_to_goal → goal_theta.
            #
            # Root cause of the wrong-direction Phase-3 rotation:
            #   bearing_to_goal near the approach pose points roughly parallel
            #   to the table face (e.g. ≈ 4°), not toward the target (≈ 134°).
            #   The unicycle was delivering the robot at heading ≈ 4–8° instead
            #   of ≈ 134°.  Phase 3 then had to make a large unexpected rotation
            #   that ended up on the wrong side of the approach heading.
            #
            # With blending, the robot arrives already facing goal_theta so
            # Phase 3 only makes a small, geometrically predictable correction.
            _HEADING_BLEND_DIST = 0.5   # m — start blending at this distance
            if dist_xy < _HEADING_BLEND_DIST:
                alpha   = dist_xy / _HEADING_BLEND_DIST  # 1 = far (bearing), 0 = at goal (heading)
                blend_x = alpha * math.cos(bearing_to_goal) + (1.0 - alpha) * math.cos(goal_theta)
                blend_y = alpha * math.sin(bearing_to_goal) + (1.0 - alpha) * math.sin(goal_theta)
                steering_err = _wrap(math.atan2(blend_y, blend_x) - rtheta)
            else:
                steering_err = bearing_err_raw

            linear  = _clamp(
                _K_LINEAR * dist_xy * max(0.0, math.cos(steering_err)),
                0.0, _MAX_LINEAR,
            )
            angular = _clamp(_K_ANGULAR * steering_err, -_MAX_ANGULAR, _MAX_ANGULAR)
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
        Return the best base pose from which target_xyz is reachable.

        Table-aware mode (world_sdf_path was supplied)
        -----------------------------------------------
        Uses SdfTableResolver to stop at table_edge + standoff, facing the
        object.  Falls through to the angle-sweep if the standoff pose is
        not within the arm reach envelope (e.g. very wide table).

        Fallback (no resolver or table not found)
        -----------------------------------------
        Sweep _N_APPROACH_ANGLES candidates at preferred_reach and pick the
        lowest-cost reachable one (original behaviour).

        Candidates are scored by:
            cost = 10 * |final_align| + distance
        where final_align is the angle between nav_theta and the drive heading.
        Minimising final_align means the robot arrives already facing the object.
        """
        tx, ty, _    = target_xyz
        rx, ry, rth  = robot_pose

        # ── Table-aware primary candidate ─────────────────────────────
        if self._table_resolver is not None:
            table_name = self._table_resolver.find_table_for_object((tx, ty))
            if table_name is not None:
                standoff_pose = self._table_resolver.approach_pose_for_object(
                    object_xy  = (tx, ty),
                    robot_xy   = (rx, ry),
                    table_name = table_name,
                )
                if standoff_pose is not None and self._is_reachable(target_xyz, standoff_pose):
                    self.get_logger().info(
                        f"[TiagoNavigator] Table-aware approach pose for '{table_name}': "
                        f"({standoff_pose[0]:.2f}, {standoff_pose[1]:.2f}, "
                        f"{math.degrees(standoff_pose[2]):.1f}°)"
                    )
                    return standoff_pose

        # ── Angle-sweep fallback ──────────────────────────────────────
        base_angle   = math.atan2(ry - ty, rx - tx)
        best: Optional[NavPose] = None
        best_cost = float("inf")

        for i in range(_N_APPROACH_ANGLES):
            angle     = base_angle + i * (2.0 * math.pi / _N_APPROACH_ANGLES)
            nav_x     = tx + self._preferred_reach * math.cos(angle)
            nav_y     = ty + self._preferred_reach * math.sin(angle)
            nav_theta = math.atan2(ty - nav_y, tx - nav_x)
            candidate = (nav_x, nav_y, nav_theta)

            if not self._is_reachable(target_xyz, candidate):
                continue

            d             = math.hypot(nav_x - rx, nav_y - ry)
            drive_heading = math.atan2(nav_y - ry, nav_x - rx) if d > 1e-3 else nav_theta
            final_align   = abs(_wrap(nav_theta - drive_heading))
            cost          = 10.0 * final_align + d

            if cost < best_cost:
                best_cost = cost
                best      = candidate

        if best is not None:
            return best

        # Geometric fallback — robot-side position regardless of reachability
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
            self._robot_pose_odom = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                theta,
            )
            # Mirror to legacy alias for any external reader
            if not self._use_ground_truth:
                self._robot_pose = self._robot_pose_odom
        # Only signal pose_ready from odom if we are NOT in ground-truth
        # mode — in GT mode we wait for the first pose/info message instead
        # so the nav loop reads correct world coordinates from the start.
        if not self._world_name:
            self._pose_ready.set()

    def _gz_pose_cb(self, msg: TFMessage) -> None:
        """
        Ground-truth pose callback.

        The Gazebo Fortress gz-ros2 bridge republishes ``/world/<world>/pose/info``
        as a tf2_msgs/TFMessage where each transform's ``child_frame_id`` is
        the model name and the translation/rotation is its world pose.

        We pluck the entry whose child_frame_id matches our robot's model name
        and copy it into ``_robot_pose_world``.
        """
        for tfs in msg.transforms:
            child = tfs.child_frame_id
            # Skip link-level entries like "robot::base_link"
            if "::" in child:
                continue
            if child != self._robot_name:
                continue

            t = tfs.transform.translation
            q = tfs.transform.rotation
            # 2D yaw from quaternion (z, w only — assumes flat ground)
            yaw = 2.0 * math.atan2(q.z, q.w)

            with self._pose_lock:
                self._robot_pose_world = (t.x, t.y, yaw)
                # Latch ground-truth mode on first valid message
                if not self._use_ground_truth:
                    self._use_ground_truth = True
                    # Mirror to legacy alias so any direct readers see GT now
                    self._robot_pose = (t.x, t.y, yaw)
                    self.get_logger().info(
                        f"[TiagoNavigator] Ground-truth pose active "
                        f"({t.x:+.3f}, {t.y:+.3f}, {math.degrees(yaw):+.1f}°)."
                    )
                else:
                    self._robot_pose = (t.x, t.y, yaw)

            self._pose_ready.set()
            return   # found our robot — done with this message

    def _get_pose(self) -> NavPose:
        """
        Return the current robot pose used by the nav-control loop.

        Ground-truth mode  (world_name supplied AND first GT msg received):
            Returns the Gazebo world pose directly.  Frame conversions
            below become identity, so the nav loop runs in WORLD frame.

        Odometry mode  (no world_name, or GT not yet received):
            Returns the raw odom pose.  The loop runs in odom frame and
            goals are pre-converted via _world_to_odom.
        """
        with self._pose_lock:
            if self._use_ground_truth:
                return self._robot_pose_world
            return self._robot_pose_odom

    def _get_world_pose(self) -> NavPose:
        """Return the current robot pose in the WORLD frame (always)."""
        with self._pose_lock:
            if self._use_ground_truth:
                return self._robot_pose_world
            ox, oy, oth = self._robot_pose_odom
        # Out of the lock for the math
        return self._odom_to_world(ox, oy, oth)

    def _wait_for_pose(self, timeout: float = 10.0) -> bool:
        return self._pose_ready.wait(timeout=timeout)

    # ------------------------------------------------------------------
    # World ↔ "loop frame" conversion
    #
    # The nav loop reads poses with _get_pose() and compares against goals
    # in the same frame.  When ground truth is active, _get_pose() returns
    # WORLD coords, so the conversion below collapses to identity — no drift
    # and no transform error.  When falling back to odometry, the original
    # spawn-pose offset transform is used.
    # ------------------------------------------------------------------

    def _world_to_odom(self, wx: float, wy: float, wtheta: float) -> NavPose:
        # Identity in ground-truth mode: the loop reads world pose directly.
        if self._use_ground_truth:
            return (wx, wy, _wrap(wtheta))
        sx, sy, syaw = self._spawn_x, self._spawn_y, self._spawn_yaw
        dx  = wx - sx
        dy  = wy - sy
        ox  =  dx * math.cos(syaw) + dy * math.sin(syaw)
        oy  = -dx * math.sin(syaw) + dy * math.cos(syaw)
        oth = _wrap(wtheta - syaw)
        return (ox, oy, oth)

    def _odom_to_world(self, ox: float, oy: float, otheta: float) -> NavPose:
        if self._use_ground_truth:
            return (ox, oy, _wrap(otheta))
        sx, sy, syaw = self._spawn_x, self._spawn_y, self._spawn_yaw
        wx  = sx + ox * math.cos(syaw) - oy * math.sin(syaw)
        wy  = sy + ox * math.sin(syaw) + oy * math.cos(syaw)
        wth = _wrap(otheta + syaw)
        return (wx, wy, wth)

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