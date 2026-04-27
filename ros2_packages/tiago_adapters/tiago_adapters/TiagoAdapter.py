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
from typing import Callable, Dict, Optional, Tuple

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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# tf2 — used to look up the gripper pose in the robot's namespaced TF tree.
# Gazebo's /world/<n>/pose/info feed publishes bare link names with empty
# header.frame_id, which is unsafe for multi-robot setups.  TF frames
# published by robot_state_publisher are prefixed with the robot name
# (frame_prefix="tiago_robot1/") so they disambiguate cleanly.
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration as RclpyDuration

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

_NAV_RATE_HZ       = 10.0
_GOAL_XY_TOL       = 0.15    # m   — position tolerance
_GOAL_THETA_TOL    = 0.03    # rad (~1.7°) — tight final alignment tolerance
_MAX_LINEAR        = 0.90    # m/s
_MAX_ANGULAR       = 1.60    # rad/s
_K_LINEAR          = 1.20
_K_ANGULAR         = 2.40
_ROTATE_TOL        = 0.03    # rad (~1.7°) — bearing lock tolerance before driving straight
_NAV_TIMEOUT       = 120.0   # s

# ---------------------------------------------------------------------------
# Arm move completion tolerances
# ---------------------------------------------------------------------------

# Pre-grasp staging steps are rough positioning moves that bring the arm into
# the elbow-up configuration before the IK-planned pick/place trajectory runs.
# They don't need to be precise: a loose joint tolerance (0.15 rad ≈ 8.6°)
# and a longer wait window (60 s) prevents a near-complete staging move from
# being mistakenly declared failed due to controller settling time.
#
# All other MOVE steps (IK-planned pick/place/retrace and tuck/home) keep the
# tight tolerance (0.05 rad) and 90 s window.
_PRE_GRASP_JOINT_TOL = 0.15   # rad — staging moves
_DEFAULT_JOINT_TOL   = 0.05   # rad - precision IK trajectories
_PRE_GRASP_MAX_WAIT  = 60.0   # s
_DEFAULT_MAX_WAIT    = 90.0   # s

# ---------------------------------------------------------------------------
# Ground-truth alignment correction (pre-grasp)
# ---------------------------------------------------------------------------
#
# After the IK-planned pick trajectory completes, the gripper may not be
# aligned with the object centre.  ikpy's IK accepts solutions with up to
# 15 cm FK error and chooses from multiple elbow/wrist branches based on
# the seed -- so the gripper can land several cm short/forward/up/down
# relative to the commanded tip position.  We close the loop with the
# ground-truth gripper pose from TF and correct each axis independently.
#
# Axis -> actuator mapping (arm base frame, robot facing +X):
#   Vertical error (Z)      ->  arm_2_joint (shoulder lift).
#       In the right-bend pose a LARGER arm_2 raises the wrist.
#       Delta_arm_2 = +k_z * Delta_z.
#   Lateral error (Y_arm)   ->  arm_1_joint (shoulder pan).
#       In the right-bend pose +arm_1 sweeps the gripper toward +y_arm
#       (robot's left).  Delta_arm_1 = +k_y * Delta_y_arm.
#   Forward error (X_arm)   ->  BASE forward/back drive via cmd_vel.
#       We drive the base instead of the elbow (arm_4) because elbow
#       extension also drops the wrist in the right-bend pose, which
#       can plant the fingers into the table.  Driving the base keeps
#       the arm configuration frozen -- the gripper translates with
#       the base in a straight line along the approach direction.
#
# X correction is applied FIRST (base motion is slower than arm motion),
# then Y and Z with a single-pass arm move.  A final GT re-read logs
# residual error for diagnostics.
_ALIGN_XY_TOL          = 0.020   # m - accept within 2 cm lateral
_ALIGN_Z_TOL           = 0.020   # m - accept within 2 cm vertical
_ALIGN_X_TOL           = 0.030   # m - accept within 3 cm approach-direction
_ALIGN_MAX_DY          = 0.08    # m - cap arm Y correction per pass
_ALIGN_MAX_DZ          = 0.20    # m - cap arm Z correction.  Generous so
                                  # arm_2 can recover large IK droops; the
                                  # actual movement is further clamped by
                                  # arm_2's joint limits inside the method.
_ALIGN_MAX_DX_BASE     = 0.20    # m - cap base drive distance per pass
_ALIGN_K_ARM1          = 1.10    # rad/m - gain Delta_y_arm -> Delta_arm_1
_ALIGN_K_ARM2          = 1.20    # rad/m - gain Delta_z     -> Delta_arm_2
_ALIGN_BASE_SPEED      = 0.05    # m/s  - base forward/back speed
_ALIGN_BASE_DRIVE_SCALE= 2.0     # Gazebo base only covers ~50% of commanded
                                  # distance at 0.05 m/s; scale drive time up.
_ALIGN_MOVE_TIME       = 2       # s   - arm corrective trajectory duration
_ALIGN_SETTLE_TIME     = 0.5     # s   - wait after correction before re-read

# Safety: if the gripper is more than this far below the commanded grasp
# target at any alignment checkpoint, we assume arm_2 has saturated and
# the gripper cannot be lifted to a safe height.  The alignment method
# returns False, the caller SKIPS the close_gripper_grasp step, and the
# fingers do not descend onto the object / table.
#
# Reference (top-down grasp with _GRASP_FRAME_OFFSET_Z = 0.085):
#   commanded target = object_centre + 0.075 (world-frame Z)
#   "gripper at target"            -> fingers grip object body correctly
#   "gripper 2-3 cm below target"  -> fingers near object mid-bottom
#   "gripper 5 cm below target"    -> fingers near object bottom, risky
#   "gripper 8+ cm below target"   -> fingers at/below table
# Threshold 0.05 m tolerates small undershoots where fingers still
# clear the supporting surface.
_ALIGN_Z_ABORT_BELOW   = 0.05    # m

# ---------------------------------------------------------------------------
# Closed-loop vertical motion (descend / ascend via arm_2)
# ---------------------------------------------------------------------------
#
# After pick_motion (which is now horizontal-only at an elevated Z) the
# adapter runs a closed-loop arm_2 descent to bring the gripper down to
# the grasp height, while reading gripper_grasping_frame via TF after
# every small step.  This replaces Cartesian-space waypoint descent,
# which was prone to IK-branch switching and wrist drooping through the
# table surface.
#
# The arm_2 -> gripper_Z sign depends on the arm configuration, so the
# first step is used to auto-detect it: we command a small arm_2 move
# and check whether the gripper went up or down, then use that sign for
# the rest of the descent.
# Descent strategy: ONE DIRECTION ONLY.  Once we start moving arm_2 in
# the direction that lowers the gripper, we never reverse.  This is
# critical because:
#   - arm_2's Z-gain varies with configuration; a proportional controller
#     with a single-measurement gain can oscillate (observed: descent
#     overshoots target, reverse goes too high, reverse again, etc.).
#   - Reversing arm_2 with small deltas is sluggish on the Gazebo
#     controller and trips false-positive "stuck" aborts.
#
# To minimise overshoot we scale the computed step by _DESCENT_STEP_SCALE
# (< 1.0).  The descent stops as soon as the gripper is at or past the
# target in the descent direction.  Any overshoot is accepted silently
# (a gripper 1-3 cm low still grasps the can cleanly; overshooting is
# strictly less dangerous than oscillating).
#
# "Stuck" is detected from joint-state feedback: arm_2 was commanded
# further in the descent direction but did not respond.  See the
# constants _DESCENT_JOINT_ABS_EPS and _DESCENT_JOINT_FRAC below for
# the exact criterion (both must be true to declare stuck).
_DESCENT_PROBE_RAD      = 0.10    # rad - first step, used to measure gain
_DESCENT_MAX_RAD        = 0.12    # rad - cap on any single step (safety)
_DESCENT_STEP_SCALE     = 0.70    # scale applied to proportional step to
                                   # bias toward undershoot rather than
                                   # overshoot (avoids oscillation).
_DESCENT_TOL            = 0.030   # m - accept within 3 cm of target Z.
                                   # Loose enough that the controller's
                                   # native resolution doesn't trip
                                   # false-positive stuck aborts when
                                   # chasing the last cm of error, while
                                   # still tight enough that the gripper
                                   # is correctly placed for grasping.
_DESCENT_UNDERSHOOT_OK  = 0.12    # m - if pick_motion lands the gripper
                                   # BELOW the nominal target by this much
                                   # or less, accept without moving.  This
                                   # handles the common case where ikpy's
                                   # FK overestimates the gripper's reach
                                   # at elevated poses; trying to ascend
                                   # typically fails because arm_2 is
                                   # already near its upper joint limit.
_DESCENT_MAX_STEPS      = 12      # hard cap on iterations (safety)
_DESCENT_STEP_TIME      = 2       # s - trajectory duration per step
_DESCENT_SETTLE         = 0.20    # s - wait after each step before TF re-read
# "Stuck" detection (joint-level): the controller has finite resolution,
# so a small commanded delta may produce an even smaller realised motion.
# We declare the joint truly stuck only if BOTH:
#   - the absolute joint motion is below _DESCENT_JOINT_ABS_EPS, AND
#   - the joint motion is less than _DESCENT_JOINT_FRAC of the commanded.
# Either condition alone is normal for very small commands (e.g. 49 mrad
# commanded -> 3.4 mrad realised is 7% but absolute motion still happened
# and the gripper is descending; not stuck).
_DESCENT_JOINT_ABS_EPS  = 0.002   # rad - absolute floor for "no motion"
_DESCENT_JOINT_FRAC     = 0.20    # min realised/commanded ratio to count
                                   # as "moving" for non-tiny commands.


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
        robot_name:              str,
        mode:                    RobotMode,
        result_callback:         ResultCallback,
        robot_frame:             bool                                   = False,
        arm_base_z:              float                                  = 0.83,
        node_name:               Optional[str]                          = None,
        world_sdf_path:          Optional[str]                          = None,
        models_base_dir:         Optional[str]                          = None,
        table_standoff:          float                                  = 0.10,
        spawn_world_pose:        Optional[NavPose]                      = None,
        world_name:              Optional[str]                          = None,
    ) -> None:
        """
        spawn_world_pose : (x, y, yaw) in the Gazebo world frame at spawn time.
        world_name : Gazebo world name.  When supplied, the adapter consumes
            the ground-truth pose from /world/<world_name>/pose/info instead
            of wheel odometry -- eliminating drift.
        """
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
        self._world_name      = world_name

        self._busy      = False
        self._busy_lock = threading.Lock()

        if spawn_world_pose is not None:
            self._spawn_x, self._spawn_y, self._spawn_yaw = spawn_world_pose
        else:
            self._spawn_x, self._spawn_y, self._spawn_yaw = 0.0, 0.0, 0.0

        self._planner = TiagoPickPlacePlanner(
            robot_name           = robot_name,
            robot_frame          = robot_frame,
            arm_base_z           = arm_base_z,
            world_sdf_path       = world_sdf_path,
            models_base_dir      = models_base_dir,
            table_standoff       = table_standoff,
        )

        self._robot_pose_odom:  NavPose = (0.0, 0.0, 0.0)
        self._robot_pose_world: NavPose = (
            self._spawn_x, self._spawn_y, self._spawn_yaw,
        )
        self._use_ground_truth = False
        self._robot_pose_lock  = threading.Lock()
        self._pose_ready       = threading.Event()

        # TF buffer/listener for namespaced gripper pose lookup.
        # See _init_ros() — needs a Node handle, so the actual listener is
        # created there.  Gripper pose is looked up on demand via
        # _get_gripper_pose_world(); nothing is cached here because TF's
        # own buffer handles that.
        self._tf_buffer:   Optional[Buffer]            = None
        self._tf_listener: Optional[TransformListener] = None

        mode_label = "ROBOT-FRAME" if robot_frame else "WORLD-FRAME"
        self.get_logger().info(
            f"[TiagoAdapter] Initialising ({mode.value.upper()}, {mode_label}) ..."
        )
        if spawn_world_pose is not None:
            self.get_logger().info(
                f"[TiagoAdapter] Spawn world pose: "
                f"({self._spawn_x:.3f}, {self._spawn_y:.3f}, "
                f"{math.degrees(self._spawn_yaw):.1f}°)"
            )
        self._init_ros()
        self.get_logger().info("[TiagoAdapter] Ready.")

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _init_ros(self) -> None:
        # TF buffer + listener for namespaced gripper pose lookup.
        # Spin happens on the shared MultiThreadedExecutor, so the buffer
        # is populated in the background and lookups are non-blocking.
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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

        if self._world_name:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            topic = f"/world/{self._world_name}/pose/info"
            self.create_subscription(TFMessage, topic, self._gz_pose_cb, qos)
            self.get_logger().info(
                f"[TiagoAdapter] Subscribed to Gazebo ground truth: '{topic}'."
            )
        else:
            self.get_logger().warn(
                "[TiagoAdapter] No world_name supplied — using wheel odometry "
                "(will drift over distance).  Pass world_name='<gz_world>' to "
                "use ground-truth pose from the gz-ros2 bridge."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick_and_place(
        self,
        pick_xyz:         XYZ,
        place_xyz:        XYZ,
        pick_object_name: Optional[str]                        = None,
        pick_object_size: Optional[Tuple[float, float, float]] = None,
    ) -> bool:
        """
        Plan and execute a full pick-and-place cycle.

        Parameters
        ----------
        pick_xyz, place_xyz : (x, y, z)
            Pick and place positions.
        pick_object_name : str, optional
            Name of the pick object (directory name under the planner's
            models_base_dir).  Used by the planner to look up the object's
            vertical extent and set a safe grasp clearance.
        pick_object_size : (sx, sy, sz), optional
            Explicit object extent in metres.  Overrides SDF-parsed size.
            Use this for mesh-based models that have no primitive geometry.

        Non-blocking -- returns True if accepted, False if already busy.
        result_callback fires on completion.
        """
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn("[TiagoAdapter] Rejected - already busy.")
                return False
            self._busy = True

        if not self._pose_ready.wait(timeout=15.0):
            src = (f"/world/{self._world_name}/pose/info"
                   if self._world_name else
                   f"/{self._robot_name}/mobile_base_controller/odom")
            self.get_logger().error(
                f"[TiagoAdapter] No pose within 15 s from '{src}'. "
                "Check the gz-ros2 bridge is running and the robot is spawned."
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
            pick_object_name=pick_object_name,
            pick_object_size=pick_object_size,
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
            self._robot_pose_odom = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                theta,
            )
        if not self._world_name:
            self._pose_ready.set()


    def _gz_pose_cb(self, msg) -> None:
        """Ground-truth base pose from the Gazebo Fortress gz-ros2 bridge.

        The /world/<n>/pose/info topic publishes a transform for every
        model and every link in the scene.  The gz-ros2 bridge maps
        Gazebo entity names into child_frame_id WITHOUT any namespace
        prefix, which makes nested link names (arm_7_link, wrist_ft_link,
        etc.) ambiguous across multiple robots.  Only the top-level model
        name is usable as a disambiguator, so we restrict this callback
        to reading the ROBOT BASE pose:

            child_frame_id == self._robot_name  -->  (x, y, yaw) in world

        The gripper pose is NOT read from this topic.  gripper_grasping_frame
        does not appear in pose/info at all (it is a TF-only frame from
        the URDF, not a physical Gazebo link), and the nearby link names
        that ARE published would be ambiguous for multi-robot.  Gripper
        pose is looked up via TF in _get_gripper_pose_world() using the
        namespaced frame names published by robot_state_publisher.
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
                        f"[TiagoAdapter] Ground-truth pose active "
                        f"({t.x:+.3f}, {t.y:+.3f}, {math.degrees(yaw):+.1f} deg)."
                    )
            self._pose_ready.set()
            return


    def _joint_state_callback(self, msg: JointState) -> None:
        with self._joint_state_lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_joint_positions[name] = pos

    def _get_robot_pose(self) -> NavPose:
        """Return current robot pose in the WORLD frame (GT preferred)."""
        with self._robot_pose_lock:
            if self._use_ground_truth:
                return self._robot_pose_world
            odom_x, odom_y, odom_theta = self._robot_pose_odom
        return self._odom_to_world(odom_x, odom_y, odom_theta)

    def _get_gripper_pose_world(self) -> Optional[Tuple[float, float, float]]:
        """
        Return the gripper_grasping_frame pose (x, y, z) in the WORLD frame,
        or None if the lookup fails (TF not yet populated, frame missing, etc.).

        Pipeline
        --------
        1. Look up the namespaced TF transform
               {robot_name}/base_footprint  -->  {robot_name}/gripper_grasping_frame
           This gives the gripper pose in the robot base frame.  TF frames
           are properly per-robot-namespaced via frame_prefix in the launch
           file, so this is safe for multi-robot setups.

        2. Compose with the current ground-truth base pose (rx, ry, ryaw)
           to produce world-frame coordinates:
               world_x = rx + dx*cos(yaw) - dy*sin(yaw)
               world_y = ry + dx*sin(yaw) + dy*cos(yaw)
               world_z = dz + arm_base_z is NOT applied -- base_footprint
                         is at z=0 on the floor, so gripper world-Z is the
                         TF translation.z directly (already measured from
                         base_footprint which sits on the ground).
        """
        if self._tf_buffer is None:
            return None

        source = f"{self._robot_name}/base_footprint"
        target = f"{self._robot_name}/gripper_grasping_frame"

        try:
            tf = self._tf_buffer.lookup_transform(
                source, target,
                rclpy.time.Time(),                # latest available
                timeout=RclpyDuration(seconds=0.2),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"[TiagoAdapter] TF lookup {source} --> {target} failed: {exc}"
            )
            return None

        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        dz = tf.transform.translation.z

        rx, ry, ryaw = self._get_robot_pose()
        cos_t = math.cos(ryaw)
        sin_t = math.sin(ryaw)

        world_x = rx + dx * cos_t - dy * sin_t
        world_y = ry + dx * sin_t + dy * cos_t
        world_z = dz

        return (world_x, world_y, world_z)

    # ------------------------------------------------------------------
    # World ↔ Odom coordinate conversion
    # ------------------------------------------------------------------

    def _world_to_odom(
        self, wx: float, wy: float, wtheta: float
    ) -> NavPose:
        if self._use_ground_truth:
            return (wx, wy, _wrap_angle(wtheta))
        sx, sy, syaw = self._spawn_x, self._spawn_y, self._spawn_yaw
        dx   = wx - sx
        dy   = wy - sy
        ox   =  dx * math.cos(syaw) + dy * math.sin(syaw)
        oy   = -dx * math.sin(syaw) + dy * math.cos(syaw)
        oth  = _wrap_angle(wtheta - syaw)
        return (ox, oy, oth)

    def _odom_to_world(
        self, ox: float, oy: float, otheta: float
    ) -> NavPose:
        if self._use_ground_truth:
            return (ox, oy, _wrap_angle(otheta))
        sx, sy, syaw = self._spawn_x, self._spawn_y, self._spawn_yaw
        wx  = sx + ox * math.cos(syaw) - oy * math.sin(syaw)
        wy  = sy + ox * math.sin(syaw) + oy * math.cos(syaw)
        wth = _wrap_angle(otheta + syaw)
        return (wx, wy, wth)

    # ------------------------------------------------------------------
    # Plan execution
    # ------------------------------------------------------------------

    _TUCK_LABELS = frozenset({
        "tuck_arm_initial",
        "tuck_arm_pre_place_nav",
        "tuck_after_grasp",
        "return_home",
    })

    _PRE_GRASP_LABELS = frozenset({
        "pre_grasp_pick",
        "pre_grasp_place",
    })

    # Labels for the new closed-loop GT descent/ascent sentinel steps.
    # They arrive as plain MOVE steps (no joint_trajectory) with these
    # distinctive labels.  The adapter handles them via
    # _execute_vertical_motion() which commands arm_2 in small increments
    # while monitoring gripper_grasping_frame via TF.
    _DESCEND_LABELS = frozenset({"descend_to_grasp", "descend_to_place"})
    _ASCEND_LABELS  = frozenset({"ascend_after_grasp", "ascend_after_place"})

    def _dispatch_step(self, step: PickPlaceStep) -> bool:
        if step.kind == StepKind.NAVIGATE:
            if self._robot_frame:
                self.get_logger().warn(
                    "[TiagoAdapter] NAVIGATE step ignored in robot-frame mode."
                )
                return True
            return self._execute_nav_step(step)
        elif step.kind == StepKind.MOVE:
            # Closed-loop vertical descent / ascent (GT-feedback arm_2).
            # These sentinel steps have no joint_trajectory -- the target
            # Z is derived from pick_xyz / place_xyz stored on the adapter
            # by _execute_plan().
            if step.label in self._DESCEND_LABELS:
                return self._execute_vertical_motion(step.label, direction="down")
            if step.label in self._ASCEND_LABELS:
                return self._execute_vertical_motion(step.label, direction="up")

            if step.label in self._TUCK_LABELS:
                with self._joint_state_lock:
                    current = dict(self._latest_joint_positions)
                if self._planner._is_arm_at_home(current):
                    self.get_logger().info(
                        f"[TiagoAdapter] '{step.label}' skipped — arm already at home."
                    )
                    return True
            if step.label in self._PRE_GRASP_LABELS:
                self.get_logger().info(
                    f"[TiagoAdapter] Executing pre-grasp staging: '{step.label}'."
                )
            return self._execute_move_step(step)
        elif step.kind in (StepKind.OPEN_GRIPPER, StepKind.CLOSE_GRIPPER):
            return self._execute_gripper_step(step)
        self.get_logger().error(f"[TiagoAdapter] Unknown step kind: {step.kind}")
        return False

    _MAX_REPLAN_ATTEMPTS = 3

    def _execute_plan(self, plan: PickPlanResult, pick_xyz: XYZ, place_xyz: XYZ) -> None:
        # Stash pick/place so _execute_vertical_motion can read them.
        self._current_pick_xyz  = pick_xyz
        self._current_place_xyz = place_xyz

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

            # ── Ground-truth alignment check, right before grasp ─────
            # Before the gripper closes we compare the ground-truth gripper
            # pose to the planner's COMMANDED grasp target and apply small
            # corrective nudges.  ORDER: Z first (arm_2), then Y (arm_1),
            # then X (base drive).  Z first so the arm retreats to a safe
            # height before the base moves -- otherwise a low gripper can
            # drag along the table surface while the base translates.
            #
            # The commanded target = object_xyz + (0, 0, grasp_z_offset)
            # where grasp_z_offset = _GRASP_FRAME_OFFSET_Z set in plan()
            # (calibrated offset from object centre to where
            # gripper_grasping_frame should land for a correct grasp).
            #
            # If alignment cannot bring Z within a safe envelope (e.g.
            # arm_2 saturates at its upper joint limit), it returns False
            # and we SKIP the gripper close to avoid crashing the fingers
            # into the table.  The plan then proceeds with retrace/place
            # steps, which will likely fail downstream but not damage the
            # robot.
            if (step.kind == StepKind.CLOSE_GRIPPER
                    and step.label == "close_gripper_grasp"):
                grasp_target = (
                    pick_xyz[0],
                    pick_xyz[1],
                    pick_xyz[2] + self._planner._current_grasp_z_offset,
                )
                if not self._align_gripper_to_object(grasp_target):
                    self.get_logger().error(
                        "[TiagoAdapter] Grasp aborted: alignment unsafe. "
                        "Stopping plan to avoid table collision."
                    )
                    self._finish(
                        False,
                        "Grasp aborted: pre-close alignment failed "
                        "(gripper too low after Z correction)."
                    )
                    return

            if self._dispatch_step(step):
                if step.label == "close_gripper_grasp":
                    grasp_complete = True

                if step.kind == StepKind.NAVIGATE and not self._robot_frame:
                    actual_pose = self._get_robot_pose()
                    self.get_logger().info(
                        f"[TiagoAdapter] Replanning arm steps after '{step.label}' "
                        f"from actual pose ({actual_pose[0]:.2f}, {actual_pose[1]:.2f}, "
                        f"{math.degrees(actual_pose[2]):.1f}°)."
                    )
                    for j in range(i + 1, len(plan.steps)):
                        if plan.steps[j].kind == StepKind.NAVIGATE:
                            break
                        self._planner.replan_step(plan.steps[j], actual_pose)

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

        wgx, wgy, wgtheta = step.nav_goal
        goal_x, goal_y, goal_theta = self._world_to_odom(wgx, wgy, wgtheta)

        _VIRTUAL_TARGET_DIST = 50.0
        if step.nav_target_xy is not None:
            tx_w, ty_w = step.nav_target_xy
            tx_o, ty_o, _ = self._world_to_odom(tx_w, ty_w, 0.0)
        else:
            vt_w_x = wgx + _VIRTUAL_TARGET_DIST * math.cos(wgtheta)
            vt_w_y = wgy + _VIRTUAL_TARGET_DIST * math.sin(wgtheta)
            tx_o, ty_o, _ = self._world_to_odom(vt_w_x, vt_w_y, 0.0)

        dt    = 1.0 / _NAV_RATE_HZ
        start = time.perf_counter()

        self.get_logger().info(
            f"[TiagoAdapter] Navigating to world "
            f"({wgx:.2f}, {wgy:.2f}, {math.degrees(wgtheta):.1f}°) "
            f"→ odom ({goal_x:.2f}, {goal_y:.2f}, {math.degrees(goal_theta):.1f}°)."
        )

        locked_bearing: Optional[float] = None
        phase3_locked = False

        while time.perf_counter() - start < _NAV_TIMEOUT:
            if self._cancelled:
                self._stop_base()
                return False

            rx, ry, rtheta = self._get_robot_pose()
            dist_xy = math.hypot(goal_x - rx, goal_y - ry)

            prev_locked = phase3_locked
            if dist_xy < _GOAL_XY_TOL:
                phase3_locked = True
            elif dist_xy > _GOAL_XY_TOL * 2.0:
                phase3_locked = False
            if phase3_locked and not prev_locked:
                locked_bearing = None

            if phase3_locked:
                effective_theta = math.atan2(ty_o - ry, tx_o - rx)
                theta_err = _wrap_angle(effective_theta - rtheta)
                if abs(theta_err) < _GOAL_THETA_TOL:
                    self._stop_base()
                    self.get_logger().info(f"[TiagoAdapter] Reached '{step.label}'.")
                    return True
                angular = _clamp(_K_ANGULAR * theta_err, -_MAX_ANGULAR, _MAX_ANGULAR)
                self._publish_vel(0.0, angular)
                time.sleep(dt)
                continue

            bearing_to_goal  = math.atan2(goal_y - ry, goal_x - rx)
            bearing_err_raw  = _wrap_angle(bearing_to_goal - rtheta)
            if locked_bearing is None:
                if abs(bearing_err_raw) > _ROTATE_TOL:
                    angular = _clamp(_K_ANGULAR * bearing_err_raw,
                                     -_MAX_ANGULAR, _MAX_ANGULAR)
                    self._publish_vel(0.0, angular)
                    time.sleep(dt)
                    continue
                locked_bearing = bearing_to_goal

            if abs(bearing_err_raw) > math.pi / 2:
                locked_bearing = None
                self._publish_vel(0.0, _clamp(_K_ANGULAR * bearing_err_raw,
                                              -_MAX_ANGULAR, _MAX_ANGULAR))
                time.sleep(dt)
                continue

            _HEADING_BLEND_DIST = 0.5
            if dist_xy < _HEADING_BLEND_DIST:
                alpha   = dist_xy / _HEADING_BLEND_DIST
                blend_x = alpha * math.cos(bearing_to_goal) + (1.0 - alpha) * math.cos(goal_theta)
                blend_y = alpha * math.sin(bearing_to_goal) + (1.0 - alpha) * math.sin(goal_theta)
                steering_err = _wrap_angle(math.atan2(blend_y, blend_x) - rtheta)
            else:
                steering_err = bearing_err_raw

            linear  = _clamp(
                _K_LINEAR * dist_xy * max(0.0, math.cos(steering_err)),
                0.0, _MAX_LINEAR,
            )
            angular = _clamp(_K_ANGULAR * steering_err, -_MAX_ANGULAR, _MAX_ANGULAR)
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
    # Ground-truth alignment correction (pre-grasp)
    # ------------------------------------------------------------------

    def _measure_grip_error(
        self,
        object_xyz_world: XYZ,
    ) -> Optional[Tuple[Tuple[float, float, float],
                        Tuple[float, float, float],
                        float, float, float]]:
        """
        Read the ground-truth gripper pose and compute the arm-frame error
        vs the commanded grasp target.

        Returns (obj_world, gripper_world, err_x_arm, err_y_arm, err_z),
        or None if the TF lookup fails.
        err_* = object - gripper, so positive err_x_arm means the object
        is ahead of the gripper.
        """
        gripper_world = self._get_gripper_pose_world()
        if gripper_world is None:
            self.get_logger().warn(
                "[TiagoAdapter] Alignment: TF lookup failed "
                f"({self._robot_name}/base_footprint -> "
                f"{self._robot_name}/gripper_grasping_frame)."
            )
            return None

        rx, ry, rtheta = self._get_robot_pose()
        if self._robot_frame:
            xa, ya, za = object_xyz_world
            c0 = math.cos(rtheta); s0 = math.sin(rtheta)
            obj_world = (rx + xa * c0 - ya * s0,
                         ry + xa * s0 + ya * c0,
                         za)
        else:
            obj_world = object_xyz_world

        err_wx = obj_world[0] - gripper_world[0]
        err_wy = obj_world[1] - gripper_world[1]
        err_wz = obj_world[2] - gripper_world[2]

        cos_t = math.cos(rtheta)
        sin_t = math.sin(rtheta)
        err_x_arm =  err_wx * cos_t + err_wy * sin_t
        err_y_arm = -err_wx * sin_t + err_wy * cos_t

        return obj_world, gripper_world, err_x_arm, err_y_arm, err_wz


    def _send_arm_correction(
        self,
        arm1_target: float,
        arm2_target: float,
    ) -> bool:
        """
        Send a single-waypoint JointTrajectory that overrides arm_1 and
        arm_2 to the given targets while holding every other arm joint at
        its current value.  Blocks until the joints have settled or a
        bounded timeout expires.  Returns True on success.
        """
        with self._joint_state_lock:
            current = dict(self._latest_joint_positions)

        from builtin_interfaces.msg import Duration as RosDuration
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        joint_names = [
            "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
            "arm_5_joint", "arm_6_joint", "arm_7_joint",
        ]
        positions: list = []
        for j in joint_names:
            if   j == "arm_1_joint": positions.append(arm1_target)
            elif j == "arm_2_joint": positions.append(arm2_target)
            else:
                cur = current.get(j)
                if cur is None:
                    self.get_logger().warn(
                        f"[TiagoAdapter] Alignment skipped: joint '{j}' "
                        "state unavailable."
                    )
                    return False
                positions.append(cur)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions       = positions
        pt.time_from_start = RosDuration(sec=_ALIGN_MOVE_TIME, nanosec=0)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        for name in joint_names:
            pth = JointTolerance(); pth.name = name; pth.position = -1.0
            goal.path_tolerance.append(pth)
            gol = JointTolerance(); gol.name = name; gol.position = _DEFAULT_JOINT_TOL
            goal.goal_tolerance.append(gol)
        goal.goal_time_tolerance = RosDuration(sec=10, nanosec=0)

        accepted = threading.Event()

        def _on_resp(future: Future) -> None:
            handle = future.result()
            if handle.accepted:
                accepted.set()

        self._arm_client.send_goal_async(goal).add_done_callback(_on_resp)
        if not accepted.wait(timeout=5.0):
            self.get_logger().warn(
                "[TiagoAdapter] Alignment goal rejected/unresponsive."
            )
            return False

        waited = 0.0
        poll   = 0.2
        budget = float(_ALIGN_MOVE_TIME) + 2.0
        while waited < budget:
            with self._joint_state_lock:
                a1 = self._latest_joint_positions.get("arm_1_joint")
                a2 = self._latest_joint_positions.get("arm_2_joint")
            if (a1 is not None and a2 is not None
                    and abs(a1 - arm1_target) < _DEFAULT_JOINT_TOL
                    and abs(a2 - arm2_target) < _DEFAULT_JOINT_TOL):
                break
            time.sleep(poll)
            waited += poll
        time.sleep(_ALIGN_SETTLE_TIME)
        return True


    def _align_gripper_to_object(self, object_xyz_world: XYZ) -> bool:
        """
        Align gripper_grasping_frame with the commanded grasp target before
        closing the fingers.  Runs AFTER pick_motion, BEFORE close_gripper.

        CORRECTION ORDER (important): Z, then Y, then X.
          * Z first (arm_2 shoulder lift) -- retreats the gripper to a safe
            height before anything else.  A low gripper combined with a
            base drive can scrape the table surface during translation.
          * Y next (arm_1 shoulder pan)   -- fine lateral alignment.
          * X last (base forward/back)    -- closes the approach gap.

        Returns
        -------
        True  : gripper is safely positioned and ready to close.
        False : unsafe -- caller should SKIP the gripper close (arm_2
                saturated at its joint limit with Z still too low, or TF
                lookup failed).

        The commanded target already includes the planner's calibrated
        gripper_grasping_frame offset above the object centre.
        """
        # ── Initial GT read ─────────────────────────────────────────
        m = self._measure_grip_error(object_xyz_world)
        if m is None:
            return False
        obj_world, gripper_world, err_x_arm, err_y_arm, err_z = m
        self.get_logger().info(
            f"[TiagoAdapter] Pre-grasp GT alignment: "
            f"gripper=({gripper_world[0]:.3f}, {gripper_world[1]:.3f}, {gripper_world[2]:.3f})  "
            f"target=({obj_world[0]:.3f}, {obj_world[1]:.3f}, {obj_world[2]:.3f})  "
            f"err_arm=(x={err_x_arm:+.3f}, y={err_y_arm:+.3f}, z={err_z:+.3f}) m"
        )

        # ── Z correction via arm_2 (FIRST) ──────────────────────────
        # Positive err_z means the target is above the gripper -> raise.
        # In the right-bend pose, larger arm_2 raises the wrist.
        # If arm_2 would saturate at its upper limit and still can't
        # reach the target, report unsafe so the caller skips the grasp.
        if abs(err_z) > _ALIGN_Z_TOL:
            with self._joint_state_lock:
                arm2_now = self._latest_joint_positions.get("arm_2_joint")
                arm1_now = self._latest_joint_positions.get("arm_1_joint")
            if arm2_now is None or arm1_now is None:
                self.get_logger().warn(
                    "[TiagoAdapter] Alignment: arm_1/arm_2 joint state unavailable."
                )
                return False

            dz_clamped  = _clamp(err_z, -_ALIGN_MAX_DZ, _ALIGN_MAX_DZ)
            delta_arm2  = _ALIGN_K_ARM2 * dz_clamped
            arm2_target = _clamp(arm2_now + delta_arm2, -1.571, 1.091)
            effective_delta = arm2_target - arm2_now  # after limit clamping

            self.get_logger().info(
                f"[TiagoAdapter] Z correction via arm_2: "
                f"d_arm_2={effective_delta:+.3f} rad "
                f"({arm2_now:+.3f} -> {arm2_target:+.3f}, "
                f"limit cap {'HIT' if abs(arm2_target - 1.091) < 1e-3 or abs(arm2_target + 1.571) < 1e-3 else 'ok'})."
            )

            if not self._send_arm_correction(arm1_now, arm2_target):
                return False

            # Re-read and log.
            m = self._measure_grip_error(object_xyz_world)
            if m is None:
                return False
            obj_world, gripper_world, err_x_arm, err_y_arm, err_z = m
            self.get_logger().info(
                f"[TiagoAdapter] Post-Z alignment: "
                f"gripper=({gripper_world[0]:.3f}, {gripper_world[1]:.3f}, {gripper_world[2]:.3f})  "
                f"err_arm=(x={err_x_arm:+.3f}, y={err_y_arm:+.3f}, "
                f"z={err_z:+.3f}) m"
            )

        # ── Safety gate: abort if gripper is still far below target ──
        # After Z correction, if the gripper is still significantly below
        # the commanded grasp target, arm_2 ran out of range.  Driving
        # the base from this height will scrape the table.
        if err_z > _ALIGN_Z_ABORT_BELOW:
            self.get_logger().error(
                f"[TiagoAdapter] UNSAFE GRASP: gripper {err_z*100:.1f} cm "
                f"below target after Z correction (limit "
                f"{_ALIGN_Z_ABORT_BELOW*100:.1f} cm).  arm_2 likely "
                "saturated.  Aborting grasp to avoid table collision."
            )
            return False

        # ── Y correction via arm_1 ──────────────────────────────────
        if abs(err_y_arm) > _ALIGN_XY_TOL:
            with self._joint_state_lock:
                arm1_now = self._latest_joint_positions.get("arm_1_joint")
                arm2_now = self._latest_joint_positions.get("arm_2_joint")
            if arm1_now is None or arm2_now is None:
                self.get_logger().warn(
                    "[TiagoAdapter] Alignment: arm_1/arm_2 joint state "
                    "unavailable."
                )
                return False

            dy_clamped  = _clamp(err_y_arm, -_ALIGN_MAX_DY, _ALIGN_MAX_DY)
            delta_arm1  = _ALIGN_K_ARM1 * dy_clamped
            arm1_target = _clamp(arm1_now + delta_arm1, 0.0, 2.749)

            self.get_logger().info(
                f"[TiagoAdapter] Y correction via arm_1: "
                f"d_arm_1={(arm1_target - arm1_now):+.3f} rad "
                f"({arm1_now:+.3f} -> {arm1_target:+.3f})."
            )

            if not self._send_arm_correction(arm1_target, arm2_now):
                return False

            m = self._measure_grip_error(object_xyz_world)
            if m is None:
                return False
            obj_world, gripper_world, err_x_arm, err_y_arm, err_z = m
            self.get_logger().info(
                f"[TiagoAdapter] Post-Y alignment: "
                f"gripper=({gripper_world[0]:.3f}, {gripper_world[1]:.3f}, {gripper_world[2]:.3f})  "
                f"err_arm=(x={err_x_arm:+.3f}, y={err_y_arm:+.3f}, "
                f"z={err_z:+.3f}) m"
            )

        # ── X correction via base drive (LAST) ──────────────────────
        # The arm is at its final grasp height and Y now; driving the base
        # translates the gripper horizontally along the approach axis.
        # Gazebo base covers only ~50% of the commanded distance at low
        # speeds, so drive_time is scaled by _ALIGN_BASE_DRIVE_SCALE.
        if abs(err_x_arm) > _ALIGN_X_TOL:
            dx         = _clamp(err_x_arm, -_ALIGN_MAX_DX_BASE, _ALIGN_MAX_DX_BASE)
            direction  = +1.0 if dx > 0.0 else -1.0
            drive_time = (abs(dx) / _ALIGN_BASE_SPEED) * _ALIGN_BASE_DRIVE_SCALE

            self.get_logger().info(
                f"[TiagoAdapter] X correction via base drive: "
                f"dx={dx:+.3f} m (drive {'fwd' if direction > 0 else 'rev'} "
                f"{drive_time:.2f} s at {_ALIGN_BASE_SPEED:.2f} m/s, "
                f"{_ALIGN_BASE_DRIVE_SCALE:.1f}x scale)."
            )

            twist = Twist()
            twist.linear.x = direction * _ALIGN_BASE_SPEED
            deadline = time.monotonic() + drive_time
            while time.monotonic() < deadline:
                self._cmd_vel.publish(twist)
                time.sleep(0.05)
            self._stop_base()
            time.sleep(_ALIGN_SETTLE_TIME)

            m = self._measure_grip_error(object_xyz_world)
            if m is None:
                return False
            obj_world, gripper_world, err_x_arm, err_y_arm, err_z = m
            self.get_logger().info(
                f"[TiagoAdapter] Post-X alignment: "
                f"gripper=({gripper_world[0]:.3f}, {gripper_world[1]:.3f}, {gripper_world[2]:.3f})  "
                f"err_arm=(x={err_x_arm:+.3f}, y={err_y_arm:+.3f}, "
                f"z={err_z:+.3f}) m"
            )

        # ── Final safety gate ───────────────────────────────────────
        if err_z > _ALIGN_Z_ABORT_BELOW:
            self.get_logger().error(
                f"[TiagoAdapter] UNSAFE GRASP after full alignment: "
                f"gripper {err_z*100:.1f} cm below target (limit "
                f"{_ALIGN_Z_ABORT_BELOW*100:.1f} cm)."
            )
            return False

        self.get_logger().info(
            "[TiagoAdapter] Alignment complete, grasp cleared for close."
        )
        return True


    # ------------------------------------------------------------------
    # Closed-loop vertical motion (descend / ascend via arm_2)
    # ------------------------------------------------------------------

    def _execute_vertical_motion(self, label: str, direction: str) -> bool:
        """
        Drive the gripper_grasping_frame vertically to a target Z using
        arm_2 only, monitoring gripper_grasping_frame via TF after each
        step.  Other arm joints are held at their current values.

        ONE-DIRECTIONAL algorithm: once the probe determines which way
        arm_2 needs to move to reach the target, we only move in that
        direction.  The loop stops as soon as the gripper is at or past
        the target along that direction.  We never reverse.  This is a
        deliberate choice -- proportional control with a configuration-
        dependent gain oscillates badly on this arm, and the cost of
        slight overshoot (gripper 1-3 cm past target) is much lower than
        the cost of an oscillation abort.

        Steps are scaled by _DESCENT_STEP_SCALE (0.7) to bias toward
        undershoot, so convergence typically takes 2-4 steps instead of 1
        overshooting step.

        Returns True when we reach OR PASS the target in the commanded
        direction.  Returns False on abort (joint truly stuck, TF
        failure, or iteration cap).
        """
        # Resolve target Z in WORLD frame.
        if label in ("descend_to_grasp", "ascend_after_grasp"):
            base_xyz = self._current_pick_xyz
        else:
            base_xyz = self._current_place_xyz
        if base_xyz is None:
            self.get_logger().error(
                f"[TiagoAdapter] '{label}': no target xyz stored on adapter."
            )
            return False

        # Read the actual lift height from the planner so the ascent
        # target matches exactly where pick_motion's elevated waypoints
        # were.  Using a hardcoded value here caused sync bugs when
        # _PRE_APPROACH_Z_LIFT was changed in the planner.
        _Z_LIFT = self._planner._pre_approach_z_lift
        if direction == "up":
            target_z = base_xyz[2] + _Z_LIFT
        else:
            # descend_to_grasp: gripper_grasping_frame at object centre + offset.
            # (descend_to_place is no longer emitted; place_motion aims at the
            # correct place height directly via IK.)
            target_z = base_xyz[2] + self._planner._current_grasp_z_offset

        # Direction of the actual motion is decided on the FIRST step,
        # based on where the gripper actually is vs the target.  The
        # step name ('descend_to_grasp' vs 'ascend_after_grasp') hints at
        # the typical direction, but pick_motion's IK can land the gripper
        # above OR below the target depending on the solution branch, so
        # we trust the measured gripper position instead.
        #
        # Once need_descend is set on step 1, it stays fixed for the rest
        # of the loop (the "one-directional" property that prevents
        # oscillation).
        need_descend: Optional[bool] = None

        self.get_logger().info(
            f"[TiagoAdapter] '{label}': starting closed-loop arm_2 motion "
            f"to world Z={target_z:.3f} m (direction determined at step 1)."
        )

        gain: Optional[float] = None   # m per rad (signed)

        for step_i in range(1, _DESCENT_MAX_STEPS + 1):
            grip = self._get_gripper_pose_world()
            if grip is None:
                self.get_logger().error(
                    f"[TiagoAdapter] '{label}' step {step_i}: TF lookup failed."
                )
                return False
            current_z = grip[2]

            # On step 1: decide what to do based on where the gripper
            # actually landed vs the nominal planner target.
            #
            # IMPORTANT: the planner's target_z is a nominal value
            # (object_z + grasp_z_offset).  In practice ikpy's pick_motion
            # leaves the gripper 5-15 cm BELOW this nominal target because
            # of the systematic offset between ikpy's virtual chain tip
            # and the real gripper_grasping_frame at this arm config.
            # That is NOT a bug -- the gripper is exactly where it should
            # be for grasping.  So we use an ASYMMETRIC tolerance for
            # DESCENT steps only:
            #
            #   gripper above target (err_z < 0):
            #       need to DESCEND.  Tight tolerance.
            #   gripper below target (err_z > 0):
            #       accept up to _DESCENT_UNDERSHOOT_OK (default 12 cm)
            #       as "pick_motion already placed us correctly".
            #
            # For ASCENT steps this gate does NOT apply: being below the
            # ascent target is exactly the problem we need to fix (the arm
            # is at grasp height and needs to lift before retracing).
            if need_descend is None:
                err0 = target_z - current_z
                if direction == "down" and err0 > 0 and err0 <= _DESCENT_UNDERSHOOT_OK:
                    self.get_logger().info(
                        f"[TiagoAdapter] '{label}' no motion needed: "
                        f"gripper Z={current_z:.3f} is {err0*100:.1f} cm below "
                        f"nominal target {target_z:.3f}, within expected "
                        f"pick_motion undershoot ({_DESCENT_UNDERSHOOT_OK*100:.1f} cm).  "
                        f"Accepting current grasp position."
                    )
                    return True
                if abs(err0) <= _DESCENT_TOL:
                    self.get_logger().info(
                        f"[TiagoAdapter] '{label}' no motion needed: "
                        f"Z={current_z:.3f} already within "
                        f"{_DESCENT_TOL*100:.1f} cm of target {target_z:.3f}."
                    )
                    return True
                need_descend = err0 < 0   # err<0 means gripper above target -> descend
                self.get_logger().info(
                    f"[TiagoAdapter] '{label}' direction set at step 1: "
                    f"{'DESCEND' if need_descend else 'ASCEND'} "
                    f"(gripper Z={current_z:.3f}, target {target_z:.3f}, "
                    f"err {err0:+.3f})."
                )

            # One-directional stopping condition (tolerance-based, NOT
            # "past target" alone).  We stop when we are within TOL of
            # the target OR when we have passed it in the commanded
            # direction by any amount (this preserves the no-reverse
            # property: if we overshoot we accept).
            err_z = target_z - current_z
            within_tol = abs(err_z) <= _DESCENT_TOL
            passed_target = (need_descend and current_z < target_z) \
                         or ((not need_descend) and current_z > target_z)
            if within_tol or (passed_target and step_i > 1):
                self.get_logger().info(
                    f"[TiagoAdapter] '{label}' reached target at step {step_i}: "
                    f"Z={current_z:.3f} (target {target_z:.3f}, err {err_z:+.3f})."
                )
                return True

            with self._joint_state_lock:
                arm2_now = self._latest_joint_positions.get("arm_2_joint")
                current_joints = dict(self._latest_joint_positions)
            if arm2_now is None:
                self.get_logger().error(
                    f"[TiagoAdapter] '{label}': arm_2 joint state unavailable."
                )
                return False

            err_z = target_z - current_z   # >0 means need to raise gripper

            if gain is None:
                # Probe step: fixed magnitude, direction by best guess.
                # Right-bend pose: decreasing arm_2 usually lowers gripper
                # in the high-reach starting config (gain sign = +, since
                # we saw: Dtheta<0 produced Dz<0).  Try the direction that
                # is most likely to move gripper the needed way.
                # If guess is wrong, gain comes out with opposite sign and
                # subsequent steps use the correct direction -- but the
                # one-directional lock then prevents us from continuing
                # since the probe will have moved us AWAY from the target.
                # In that case we abort on the next iteration's stopping
                # check (which will see us past the target in the wrong
                # direction and report).
                delta = -_DESCENT_PROBE_RAD if need_descend else _DESCENT_PROBE_RAD
            else:
                # Use measured gain, scaled by _DESCENT_STEP_SCALE to bias
                # toward undershoot.  If gain is near zero (flat region)
                # we max out the step in the direction of err_z.
                if abs(gain) < 0.01:
                    delta = _DESCENT_MAX_RAD * (-1.0 if need_descend else +1.0)
                else:
                    raw_delta = (err_z / gain) * _DESCENT_STEP_SCALE
                    delta = _clamp(raw_delta, -_DESCENT_MAX_RAD, _DESCENT_MAX_RAD)

            # ONE-DIRECTION ENFORCEMENT: if the proportional step would
            # REVERSE (which happens if gripper overshot target), don't
            # reverse -- just accept and return True.  This is the key
            # line that prevents oscillation.
            if gain is not None:
                desired_gripper_dir = -1 if need_descend else +1   # gripper Dz sign we want
                commanded_gripper_dir = 1 if (delta * gain) > 0 else -1
                if commanded_gripper_dir != desired_gripper_dir:
                    self.get_logger().info(
                        f"[TiagoAdapter] '{label}' step {step_i}: step would "
                        f"reverse (gripper past target by {-err_z*100:+.1f} cm).  "
                        f"Accepting current position Z={current_z:.3f} "
                        f"(target {target_z:.3f})."
                    )
                    return True

            arm2_target = _clamp(arm2_now + delta, -1.571, 1.091)
            if abs(arm2_target - arm2_now) < 1e-3:
                self.get_logger().error(
                    f"[TiagoAdapter] '{label}' step {step_i}: arm_2 at its "
                    f"joint limit ({arm2_now:+.3f}), cannot move further "
                    f"{'down' if delta < 0 else 'up'} (commanded {delta:+.3f} rad).  "
                    f"Aborting (gripper Z={current_z:.3f}, target {target_z:.3f})."
                )
                return False

            commanded_delta = arm2_target - arm2_now
            if not self._send_arm2_step(arm2_target, current_joints):
                return False

            time.sleep(_DESCENT_SETTLE)
            new_grip = self._get_gripper_pose_world()
            if new_grip is None:
                self.get_logger().error(
                    f"[TiagoAdapter] '{label}' step {step_i}: TF lookup "
                    "failed after arm_2 step."
                )
                return False
            new_z = new_grip[2]
            observed_z = new_z - current_z

            # Joint-level stuck check.
            with self._joint_state_lock:
                arm2_after = self._latest_joint_positions.get("arm_2_joint")
            if arm2_after is None:
                self.get_logger().error(
                    f"[TiagoAdapter] '{label}': arm_2 state lost mid-step."
                )
                return False
            actual_joint_delta = arm2_after - arm2_now
            # Stuck = effectively no motion AND poor command-following.
            # Both conditions required: large commands with small realised
            # motion can be normal controller lag; small commands that get
            # mostly executed are also fine.  Stuck requires both to look
            # bad simultaneously.
            commanded_abs = abs(commanded_delta)
            actual_abs    = abs(actual_joint_delta)
            below_floor   = actual_abs < _DESCENT_JOINT_ABS_EPS
            poor_follow   = (commanded_abs > 1e-4
                             and (actual_abs / commanded_abs) < _DESCENT_JOINT_FRAC)
            if below_floor and poor_follow:
                self.get_logger().error(
                    f"[TiagoAdapter] '{label}' step {step_i}: arm_2 commanded "
                    f"{commanded_delta:+.3f} rad but joint moved only "
                    f"{actual_joint_delta*1000:+.1f} mrad ("
                    f"{(actual_abs/commanded_abs*100 if commanded_abs > 1e-4 else 0):.0f}% follow-through).  "
                    f"Truly stuck (joint limit or block).  Aborting at Z={new_z:.3f}."
                )
                return False

            # Update gain from the observed gripper motion vs actual joint
            # motion.  Low-pass smoothing once we have a value.
            new_gain = observed_z / actual_joint_delta if abs(actual_joint_delta) > 1e-4 else 0.0
            if gain is None:
                gain = new_gain
                self.get_logger().info(
                    f"[TiagoAdapter] '{label}' probe: arm_2 Dtheta="
                    f"{actual_joint_delta*1000:+.1f} mrad produced "
                    f"gripper Dz={observed_z*1000:+.1f} mm  "
                    f"-> local gain {gain:+.3f} m/rad."
                )
                # If the probe moved gripper the WRONG way (away from
                # target), abort early -- our sign guess was wrong.
                # The one-directional lock would also catch this on the
                # next iteration, but explicit is better.
                moved_right_way = (
                    (need_descend and observed_z <= 0) or
                    ((not need_descend) and observed_z >= 0)
                )
                if not moved_right_way and abs(observed_z) > 0.005:
                    self.get_logger().error(
                        f"[TiagoAdapter] '{label}' probe moved gripper the "
                        f"WRONG way (wanted {'down' if need_descend else 'up'}, "
                        f"got Dz={observed_z*1000:+.1f} mm).  "
                        f"arm_2/Z coupling has an unexpected sign at this config.  "
                        f"Aborting."
                    )
                    return False
            else:
                gain = 0.5 * gain + 0.5 * new_gain

        self.get_logger().error(
            f"[TiagoAdapter] '{label}' did not converge in "
            f"{_DESCENT_MAX_STEPS} steps.  Aborting."
        )
        return False


    def _send_arm2_step(
        self,
        arm2_target: float,
        current_joints: Dict[str, float],
    ) -> bool:
        """
        Command arm_2 to arm2_target while holding every other arm joint
        at its current value.  Short single-waypoint trajectory with a
        bounded wait for settle.  Returns True on success.
        """
        from builtin_interfaces.msg import Duration as RosDuration
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        joint_names = [
            "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
            "arm_5_joint", "arm_6_joint", "arm_7_joint",
        ]
        positions: list = []
        for j in joint_names:
            if j == "arm_2_joint":
                positions.append(arm2_target)
                continue
            cur = current_joints.get(j)
            if cur is None:
                self.get_logger().warn(
                    f"[TiagoAdapter] arm_2 step: joint '{j}' state unavailable."
                )
                return False
            positions.append(cur)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions       = positions
        pt.time_from_start = RosDuration(sec=_DESCENT_STEP_TIME, nanosec=0)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        for name in joint_names:
            pth = JointTolerance(); pth.name = name; pth.position = -1.0
            goal.path_tolerance.append(pth)
            gol = JointTolerance(); gol.name = name; gol.position = _DEFAULT_JOINT_TOL
            goal.goal_tolerance.append(gol)
        goal.goal_time_tolerance = RosDuration(sec=5, nanosec=0)

        accepted = threading.Event()

        def _on_resp(future: Future) -> None:
            handle = future.result()
            if handle.accepted:
                accepted.set()

        self._arm_client.send_goal_async(goal).add_done_callback(_on_resp)
        if not accepted.wait(timeout=3.0):
            self.get_logger().warn(
                "[TiagoAdapter] arm_2 step: goal rejected/unresponsive."
            )
            return False

        # Wait briefly for arm_2 to reach target (bounded).
        waited = 0.0
        poll   = 0.1
        budget = float(_DESCENT_STEP_TIME) + 1.5
        while waited < budget:
            with self._joint_state_lock:
                a2 = self._latest_joint_positions.get("arm_2_joint")
            if a2 is not None and abs(a2 - arm2_target) < _DEFAULT_JOINT_TOL:
                return True
            time.sleep(poll)
            waited += poll
        return True


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

        # Pre-grasp staging moves are approximate positioning steps — they
        # bring the arm into the elbow-up configuration before the precision
        # IK trajectory runs.  Use a relaxed tolerance so a near-complete
        # staging motion is not wrongly declared failed due to controller
        # settling time.  All other steps use the tight tolerance.
        is_pre_grasp = step.label in self._PRE_GRASP_LABELS
        tolerance    = _PRE_GRASP_JOINT_TOL if is_pre_grasp else _DEFAULT_JOINT_TOL
        max_wait     = _PRE_GRASP_MAX_WAIT  if is_pre_grasp else _DEFAULT_MAX_WAIT

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
            tol.position = tolerance   # send same tolerance to controller
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
                        f"(tol={tolerance:.3f} rad, max err={max(errors):.4f} rad)."
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
                        help="Treat pick/place coords as robot-base-frame. Skips navigation.")
    parser.add_argument("--arm-base-z",   type=float, default=0.83)
    parser.add_argument("--timeout",      type=float, default=600.0)
    parser.add_argument("--world-sdf",    default=None)
    parser.add_argument("--models-dir",   default=None)
    parser.add_argument("--table-standoff", type=float, default=0.50)
    parser.add_argument("--grasp-offset", type=float, default=0.05,
                        dest="grasp_surface_offset",
                        help="Distance (m) short of object centre where gripper stops "
                             "(≈ object radius). Default 0.05.")
    parser.add_argument(
        "--world-name", default="backyard", dest="world_name",
        help="Gazebo world name for ground-truth pose (default: backyard).",
    )
    parser.add_argument(
        "--spawn", nargs=3, type=float, metavar=("X", "Y", "YAW_RAD"),
        default=[0.0, 0.0, 0.0],
        help="Robot spawn pose in world frame: x y yaw_in_RADIANS.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()

    world_name = args.world_name or None
    sx, sy, syaw = args.spawn

    done_event = threading.Event()
    result_box: list = []

    def on_result(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    adapter = TiagoAdapter(
        robot_name           = args.robot,
        mode                 = RobotMode.SIMULATION,
        result_callback      = on_result,
        robot_frame          = args.robot_frame,
        arm_base_z           = args.arm_base_z,
        world_sdf_path       = args.world_sdf,
        models_base_dir      = args.models_dir,
        table_standoff       = args.table_standoff,
        world_name           = world_name,
        spawn_world_pose     = (sx, sy, syaw),
    )

    executor = MultiThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(adapter._gripper)

    def _spin_with_restart():
        """Run executor.spin(); restart on exception (e.g. rclpy logger
        severity bug in gripper adapter) so ROS callbacks keep firing."""
        while rclpy.ok():
            try:
                executor.spin()
            except Exception as exc:  # noqa: BLE001
                print(f"[standalone] Spin thread exception (restarting): {exc}")

    spin_thread = threading.Thread(target=_spin_with_restart, daemon=True)
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
