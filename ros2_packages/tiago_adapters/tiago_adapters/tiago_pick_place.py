"""
Pure planning module for Tiago pick-and-place tasks.

Coordinate modes
----------------
World-frame mode  (robot_frame=False, default)
    pick_xyz / place_xyz are in the WORLD frame (same as the odometry frame).
    The planner computes navigation poses and transforms targets into the
    arm-base frame internally.

Robot-frame mode  (robot_frame=True)
    pick_xyz / place_xyz are already in the ROBOT BASE frame, as returned by
    object_to_robot.ObjectToRobot.get_pose()["robot_pose"].position.
    In this mode:
      - No _world_to_arm rotation is applied (coords are already relative to
        the robot, only arm_base_z is subtracted from z).
      - _compute_nav_pose is skipped entirely — the caller is responsible for
        driving the robot to a suitable position BEFORE calling plan(), and
        for re-querying object_to_robot AFTER arriving so coordinates are fresh.
      - robot_pose argument to plan() is accepted but unused for navigation.

Arm-frame convention
--------------------
After the robot is at pose (nav_x, nav_y, nav_theta), a world point P is in
the arm-base frame as:

    dx, dy = P.xy - (nav_x, nav_y)
    x_arm  =  dx * cos(theta) + dy * sin(theta)
    y_arm  = -dx * sin(theta) + dy * cos(theta)
    z_arm  =  P.z - arm_base_z

where arm_base_z ≈ 0.83 m (arm_1_link height above the floor at default torso).
In robot-frame mode, dx=x, dy=y, theta=0, so x_arm=x, y_arm=y directly.

Arm reach envelope (conservative)
----------------------------------
    REACH_MIN_HORIZ ≤ sqrt(x²+y²) ≤ REACH_MAX_HORIZ
    Z_ARM_MIN       ≤ z_arm        ≤ Z_ARM_MAX
    x_arm           > 0            (in front of the robot)
"""

from __future__ import annotations

import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from ikpy.chain import Chain
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tiago_pick_place_result import PickPlanResult, PickPlaceStep, StepKind

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XYZ     = Tuple[float, float, float]
NavPose = Tuple[float, float, float]   # (x, y, theta) in world frame


# ---------------------------------------------------------------------------
# Arm reach envelope constants
# ---------------------------------------------------------------------------

_REACH_MAX_HORIZ = 0.75   # m
_REACH_MIN_HORIZ = 0.12   # m
_Z_ARM_MAX       =  0.55  # m (above arm_1_link)
_Z_ARM_MIN       = -0.40  # m (below arm_1_link)
_PREFERRED_REACH =  0.50  # m (comfortable horizontal extension)
_N_APPROACH_ANGLES = 12

# ---------------------------------------------------------------------------
# Motion constants
# ---------------------------------------------------------------------------

_APPROACH_HEIGHT       = 0.12   # m above target for pre-grasp / pre-place
_WAYPOINT_DURATION_SEC = 10     # s per arm trajectory waypoint
_TUCK_STAGE1_SEC       =  3
_TUCK_STAGE2_SEC       =  6
_TUCK_STAGE3_SEC       = 10
_TOP_DOWN_QUAT         = (1.0, 0.0, 0.0, 0.0)

_DEFAULT_URDF = os.environ.get(
    "TIAGO_URDF_PATH",
    "/ros2_ws/src/tiago_description/robots/tiago.urdf.xacro",
)

_ARM_JOINTS = [
    "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
    "arm_5_joint", "arm_6_joint", "arm_7_joint",
]
_ARM_HOME_JOINTS  = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
_ARM_TUCK_STAGE1  = [0.20,  0.80, -0.20, 0.80,  0.00,  0.00, 0.0]
_ARM_TUCK_STAGE2  = [0.20,  0.80, -0.20, 2.50,  0.00,  0.00, 0.0]
_ARM_TUCK_STAGE3  = [0.20, -1.34, -0.20, 1.94, -1.57,  1.37, 0.0]

_GRIPPER_OPEN   = 0.044
_GRIPPER_CLOSED = 0.002
_ARM_TIP_LINK   = "arm_tool_link"


# ---------------------------------------------------------------------------
# TiagoPickPlacePlanner
# ---------------------------------------------------------------------------

class TiagoPickPlacePlanner:
    """
    Computes a PickPlanResult for a Tiago pick-and-place task.

    Parameters
    ----------
    robot_name : str
        Robot namespace (e.g. "tiago_robot1").
    robot_frame : bool
        If True, pick_xyz / place_xyz passed to plan() are already in the
        robot base frame (e.g. from ObjectToRobot.get_pose()).
        Navigation steps are suppressed — the caller must navigate first.
        If False (default), coordinates are in the world frame and the
        planner computes navigation poses automatically.
    urdf_path : str, optional
        Path to Tiago URDF/xacro.
    approach_height : float
        Pre-grasp / pre-place vertical offset (m). Default 0.12.
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83.
    preferred_reach : float
        Preferred horizontal reach when computing nav poses (m). Default 0.50.
    """

    def __init__(
        self,
        robot_name:      str,
        robot_frame:     bool          = False,
        urdf_path:       Optional[str] = None,
        approach_height: float         = _APPROACH_HEIGHT,
        arm_base_z:      float         = 0.83,
        preferred_reach: float         = _PREFERRED_REACH,
    ) -> None:
        self._robot_name      = robot_name
        self._robot_frame     = robot_frame
        self._approach_height = approach_height
        self._arm_base_z      = arm_base_z
        self._preferred_reach = preferred_reach
        self._urdf_path       = urdf_path or self._find_urdf()

        mode = "robot-frame" if robot_frame else "world-frame"
        print(f"[TiagoPickPlacePlanner] Mode: {mode}")

        self._chain: Optional[Chain] = None
        if IKPY_AVAILABLE:
            self._chain = self._load_chain()
        else:
            print(
                "[TiagoPickPlacePlanner] WARNING: ikpy not installed. "
                "IK will fall back to home pose (not suitable for real use)."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        pick_xyz:           XYZ,
        place_xyz:          XYZ,
        robot_pose:         NavPose,
        current_arm_joints: Optional[Dict[str, float]] = None,
    ) -> PickPlanResult:
        """
        Compute a full pick-and-place plan.

        Parameters
        ----------
        pick_xyz : (x, y, z)
            Pick position.  World frame if robot_frame=False,
            robot base frame if robot_frame=True.
        place_xyz : (x, y, z)
            Place position.  Same frame as pick_xyz.
        robot_pose : (x, y, theta)
            Current robot pose in world frame.
            Unused for navigation when robot_frame=True, but still used
            for the initial arm-home check.
        current_arm_joints : dict, optional
            Current joint positions keyed by joint name.

        Returns a PickPlanResult.
        """
        steps: List[PickPlaceStep] = []

        # ── Tuck arm at start ────────────────────────────────────────
        if not self._is_arm_at_home(current_arm_joints):
            steps.append(PickPlaceStep(
                kind             = StepKind.MOVE,
                label            = "tuck_arm_initial",
                joint_trajectory = self._make_tuck_trajectory(),
            ))
        else:
            print("[TiagoPickPlacePlanner] Arm already at home — skipping initial tuck.")

        # ── Navigation to pick ───────────────────────────────────────
        if self._robot_frame:
            # Coordinates are in the robot frame right now — the caller
            # must already be in position.  No nav step generated.
            pose_at_pick = robot_pose
            print(
                "[TiagoPickPlacePlanner] robot_frame=True — "
                "navigation suppressed for pick. Caller must be in position."
            )
        else:
            pick_nav = self._compute_nav_pose(pick_xyz, robot_pose)
            pose_at_pick = pick_nav if pick_nav is not None else robot_pose
            if pick_nav is not None:
                steps.append(PickPlaceStep(
                    kind     = StepKind.NAVIGATE,
                    label    = "navigate_to_pick",
                    nav_goal = pick_nav,
                ))
                print(
                    f"[TiagoPickPlacePlanner] Navigate to pick: "
                    f"({pick_nav[0]:.2f}, {pick_nav[1]:.2f}, "
                    f"{math.degrees(pick_nav[2]):.1f}°)"
                )
            else:
                print("[TiagoPickPlacePlanner] Pick reachable from current pose.")

        # ── Open gripper ─────────────────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_pre_pick",
            gripper_width = _GRIPPER_OPEN,
        ))

        # ── Approach and pick ────────────────────────────────────────
        pick_approach = _offset_z(pick_xyz, self._approach_height)
        for label, xyz in [("approach_pick", pick_approach), ("pick", pick_xyz)]:
            xyz_arm = self._to_arm_frame(xyz, pose_at_pick)
            step = self._make_move_step(label, xyz_arm, xyz)
            if step is None:
                return PickPlanResult.failure(
                    f"IK failed for '{label}' (arm={_fmt(xyz_arm)})"
                )
            steps.append(step)

        # ── Close gripper ────────────────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.CLOSE_GRIPPER,
            label         = "close_gripper_grasp",
            gripper_width = _GRIPPER_CLOSED,
        ))

        # ── Retreat from pick ─────────────────────────────────────────
        xyz_arm = self._to_arm_frame(pick_approach, pose_at_pick)
        step = self._make_move_step("retreat_pick", xyz_arm, pick_approach)
        if step is None:
            return PickPlanResult.failure("IK failed for 'retreat_pick'")
        steps.append(step)

        # ── Navigation to place ──────────────────────────────────────
        if self._robot_frame:
            # Same reasoning: caller re-queries object_to_robot after
            # driving to the place position, then calls plan() again,
            # or caller handles navigation externally.
            pose_at_place = robot_pose
            print(
                "[TiagoPickPlacePlanner] robot_frame=True — "
                "navigation suppressed for place. Caller must be in position."
            )
        else:
            place_nav = self._compute_nav_pose(place_xyz, pose_at_pick)
            pose_at_place = place_nav if place_nav is not None else pose_at_pick
            if place_nav is not None:
                steps.append(PickPlaceStep(
                    kind             = StepKind.MOVE,
                    label            = "tuck_arm_pre_place_nav",
                    joint_trajectory = self._make_tuck_trajectory(),
                ))
                steps.append(PickPlaceStep(
                    kind     = StepKind.NAVIGATE,
                    label    = "navigate_to_place",
                    nav_goal = place_nav,
                ))
                print(
                    f"[TiagoPickPlacePlanner] Navigate to place: "
                    f"({place_nav[0]:.2f}, {place_nav[1]:.2f}, "
                    f"{math.degrees(place_nav[2]):.1f}°)"
                )
            else:
                print("[TiagoPickPlacePlanner] Place reachable from pick pose.")

        # ── Approach and place ───────────────────────────────────────
        place_approach = _offset_z(place_xyz, self._approach_height)
        for label, xyz in [("approach_place", place_approach), ("place", place_xyz)]:
            xyz_arm = self._to_arm_frame(xyz, pose_at_place)
            step = self._make_move_step(label, xyz_arm, xyz)
            if step is None:
                return PickPlanResult.failure(
                    f"IK failed for '{label}' (arm={_fmt(xyz_arm)})"
                )
            steps.append(step)

        # ── Open gripper (release) ───────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_release",
            gripper_width = _GRIPPER_OPEN,
        ))

        # ── Retreat from place ───────────────────────────────────────
        xyz_arm = self._to_arm_frame(place_approach, pose_at_place)
        step = self._make_move_step("retreat_place", xyz_arm, place_approach)
        if step is None:
            return PickPlanResult.failure("IK failed for 'retreat_place'")
        steps.append(step)

        # ── Return home ──────────────────────────────────────────────
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "return_home",
            joint_trajectory = self._make_tuck_trajectory(),
        ))

        return PickPlanResult(
            steps   = steps,
            success = True,
            message = f"Plan ready: {len(steps)} steps.",
        )

    # ------------------------------------------------------------------
    # Reachability
    # ------------------------------------------------------------------

    def is_reachable(self, target_xyz: XYZ, robot_pose: NavPose) -> bool:
        """
        Return True if target_xyz is within the arm's reach envelope.

        In robot_frame mode, target_xyz is already relative to the robot so
        robot_pose is ignored (theta=0, no translation).
        In world_frame mode, robot_pose is used to transform first.
        """
        x_arm, y_arm, z_arm = self._to_arm_frame(target_xyz, robot_pose)
        horiz    = math.sqrt(x_arm ** 2 + y_arm ** 2)
        in_horiz = _REACH_MIN_HORIZ <= horiz <= _REACH_MAX_HORIZ
        in_z     = _Z_ARM_MIN <= z_arm <= _Z_ARM_MAX
        in_front = x_arm > 0
        return in_horiz and in_z and in_front

    # ------------------------------------------------------------------
    # Coordinate transform
    # ------------------------------------------------------------------

    def _to_arm_frame(self, xyz: XYZ, robot_pose: NavPose) -> XYZ:
        """
        Convert xyz to the arm-base frame.

        robot_frame=True
            xyz is already in the robot base frame (from ObjectToRobot).
            Only subtract arm_base_z from z; x and y pass through unchanged.

        robot_frame=False
            Full world → arm transform: translate by robot XY, rotate by
            -nav_theta, subtract arm_base_z from z.
        """
        if self._robot_frame:
            return (xyz[0], xyz[1], xyz[2] - self._arm_base_z)
        return self._world_to_arm(xyz, robot_pose)

    def _world_to_arm(self, xyz_world: XYZ, robot_pose: NavPose) -> XYZ:
        """World frame → arm-base frame (world_frame mode only)."""
        nav_x, nav_y, nav_theta = robot_pose
        dx = xyz_world[0] - nav_x
        dy = xyz_world[1] - nav_y
        cos_t =  math.cos(nav_theta)
        sin_t =  math.sin(nav_theta)
        x_arm =  dx * cos_t + dy * sin_t
        y_arm = -dx * sin_t + dy * cos_t
        z_arm =  xyz_world[2] - self._arm_base_z
        return (x_arm, y_arm, z_arm)

    # ------------------------------------------------------------------
    # Nav pose computation  (world_frame mode only)
    # ------------------------------------------------------------------

    def _compute_nav_pose(
        self,
        target_xyz: XYZ,
        robot_pose: NavPose,
    ) -> Optional[NavPose]:
        """Return nav pose needed to reach target, or None if already reachable."""
        if self.is_reachable(target_xyz, robot_pose):
            return None

        tx, ty, _ = target_xyz
        rx, ry    = robot_pose[0], robot_pose[1]
        base_angle = math.atan2(ry - ty, rx - tx)

        best: Optional[NavPose] = None
        best_dist = float("inf")

        for i in range(_N_APPROACH_ANGLES):
            angle     = base_angle + i * (2.0 * math.pi / _N_APPROACH_ANGLES)
            nav_x     = tx + self._preferred_reach * math.cos(angle)
            nav_y     = ty + self._preferred_reach * math.sin(angle)
            nav_theta = math.atan2(ty - nav_y, tx - nav_x)
            candidate = (nav_x, nav_y, nav_theta)

            if not self.is_reachable(target_xyz, candidate):
                continue

            d = math.hypot(nav_x - rx, nav_y - ry)
            if d < best_dist:
                best_dist = d
                best      = candidate

        if best is not None:
            return best

        # Geometric fallback
        print("[TiagoPickPlacePlanner] WARNING: no reachable nav pose — geometric fallback.")
        nav_x     = tx + self._preferred_reach * math.cos(base_angle)
        nav_y     = ty + self._preferred_reach * math.sin(base_angle)
        nav_theta = math.atan2(ty - nav_y, tx - nav_x)
        return (nav_x, nav_y, nav_theta)

    # ------------------------------------------------------------------
    # Step construction
    # ------------------------------------------------------------------

    def _make_move_step(
        self,
        label:   str,
        xyz_arm: XYZ,
        xyz_ref: XYZ,   # original (world or robot-frame) coords for the Pose field
    ) -> Optional[PickPlaceStep]:
        world_pose = _make_pose(xyz_ref, _TOP_DOWN_QUAT)
        joint_pos  = self._solve_ik(xyz_arm)
        if joint_pos is None:
            return None
        return PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = label,
            target_pose      = world_pose,
            joint_trajectory = self._make_trajectory(joint_pos),
        )

    def _solve_ik(self, xyz_arm: XYZ) -> Optional[List[float]]:
        if not IKPY_AVAILABLE or self._chain is None:
            print(f"[TiagoPickPlacePlanner] ikpy unavailable — home joints for {_fmt(xyz_arm)}.")
            return _ARM_HOME_JOINTS[:]

        target_matrix = _xyz_to_matrix(xyz_arm, _TOP_DOWN_QUAT)
        n = len(self._chain.links)
        revolute_idx = [
            i for i, lnk in enumerate(self._chain.links)
            if getattr(lnk, "joint_type", "fixed") == "revolute"
        ]

        seeds = []
        for scale in (1.0, 0.5, 0.0):
            s = [0.0] * n
            for k, idx in enumerate(revolute_idx):
                if k < len(_ARM_HOME_JOINTS):
                    s[idx] = _ARM_HOME_JOINTS[k] * scale
            seeds.append(s)

        import warnings
        best_joints = None
        best_error  = float("inf")

        for seed in seeds:
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    ik_result = self._chain.inverse_kinematics_frame(
                        target           = target_matrix,
                        initial_position = seed,
                        orientation_mode = None,
                    )
            except Exception as exc:
                print(f"[TiagoPickPlacePlanner] IK error for {_fmt(xyz_arm)}: {exc}")
                continue

            joints = [ik_result[i] for i in revolute_idx]
            if len(joints) != 7:
                continue

            fk    = self._chain.forward_kinematics(ik_result)
            error = np.linalg.norm(fk[:3, 3] - np.array(xyz_arm))
            if error < best_error:
                best_error  = error
                best_joints = joints

        if best_joints is None:
            print(f"[TiagoPickPlacePlanner] IK failed for {_fmt(xyz_arm)}.")
            return None

        if best_error > 0.08:
            print(
                f"[TiagoPickPlacePlanner] IK error {best_error*100:.1f} cm > 8 cm "
                f"for {_fmt(xyz_arm)} — rejecting."
            )
            return None

        print(f"[TiagoPickPlacePlanner] IK {_fmt(xyz_arm)}: FK error {best_error*100:.1f} cm.")
        return best_joints

    # ------------------------------------------------------------------
    # Arm home / tuck
    # ------------------------------------------------------------------

    _HOME_TOLERANCE = 0.15

    def _is_arm_at_home(self, current_joints: Optional[Dict[str, float]]) -> bool:
        if not current_joints:
            return False
        for joint, home_pos in zip(_ARM_JOINTS, _ARM_HOME_JOINTS):
            current = current_joints.get(joint)
            if current is None or abs(current - home_pos) > self._HOME_TOLERANCE:
                return False
        return True

    def _make_tuck_trajectory(self) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS
        for positions, sec in [
            (_ARM_TUCK_STAGE1, _TUCK_STAGE1_SEC),
            (_ARM_TUCK_STAGE2, _TUCK_STAGE2_SEC),
            (_ARM_TUCK_STAGE3, _TUCK_STAGE3_SEC),
        ]:
            pt = JointTrajectoryPoint()
            pt.positions       = positions[:]
            pt.time_from_start = Duration(sec=sec, nanosec=0)
            traj.points.append(pt)
        return traj

    def _make_trajectory(
        self,
        joint_positions: List[float],
        duration_sec:    int = _WAYPOINT_DURATION_SEC,
    ) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions       = joint_positions
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        traj.points = [pt]
        return traj

    # ------------------------------------------------------------------
    # URDF loading
    # ------------------------------------------------------------------

    def _find_urdf(self) -> str:
        candidates = [
            _DEFAULT_URDF,
            "/ros2_ws/src/tiago_robot/tiago_description/robots/tiago.urdf.xacro",
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("tiago_description")
            xacro = os.path.join(share, "robots", "tiago.urdf.xacro")
            if os.path.exists(xacro):
                return xacro
        except Exception:
            pass
        return _DEFAULT_URDF

    def _load_chain(self) -> Optional[Chain]:
        if not os.path.exists(self._urdf_path):
            print(f"[TiagoPickPlacePlanner] URDF not found: '{self._urdf_path}'.")
            return None
        urdf_path = self._urdf_path
        if urdf_path.endswith(".xacro"):
            urdf_path = self._process_xacro(urdf_path)
            if urdf_path is None:
                return None
        arm_urdf = self._extract_arm_urdf(urdf_path)
        if arm_urdf is None:
            return None
        return self._build_chain(arm_urdf)

    def _process_xacro(self, xacro_path: str) -> Optional[str]:
        out = os.path.join(tempfile.gettempdir(), "tiago_processed.urdf")
        cmd = ["xacro", xacro_path, "arm:=True", "end_effector:=pal-gripper"]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            with open(out, "w") as f:
                f.write(result.stdout)
            print(f"[TiagoPickPlacePlanner] xacro → '{out}'.")
            return out
        except subprocess.CalledProcessError as exc:
            print(f"[TiagoPickPlacePlanner] xacro failed:\n{exc.stderr}")
            return None
        except FileNotFoundError:
            print("[TiagoPickPlacePlanner] 'xacro' not found.")
            return None

    def _extract_arm_urdf(self, urdf_path: str) -> Optional[str]:
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
        except Exception as exc:
            print(f"[TiagoPickPlacePlanner] URDF parse error: {exc}")
            return None

        all_links = {e.get("name") for e in root.findall("link")}
        tip_link  = next((t for t in [_ARM_TIP_LINK, "arm_7_link"] if t in all_links), None)
        if tip_link is None:
            print(f"[TiagoPickPlacePlanner] Tip link not found. Available: {sorted(all_links)}")
            return None

        joint_map: Dict = {}
        for joint in root.findall("joint"):
            p = joint.find("parent")
            c = joint.find("child")
            if p is not None and c is not None:
                joint_map[c.get("link")] = (p.get("link"), joint)

        path_links:  List[str] = []
        path_joints: List     = []
        current = tip_link
        while current in joint_map:
            parent, joint_elem = joint_map[current]
            path_links.append(current)
            path_joints.append(joint_elem)
            current = parent
        path_links.append(current)
        path_links.reverse()
        path_joints.reverse()

        link_map  = {e.get("name"): e for e in root.findall("link")}
        new_robot = ET.Element("robot")
        new_robot.set("name", "tiago_arm")
        for lnk_name in path_links:
            if lnk_name in link_map:
                new_robot.append(link_map[lnk_name])
        for jnt in path_joints:
            new_robot.append(jnt)

        out = os.path.join(tempfile.gettempdir(), "tiago_arm_only.urdf")
        ET.ElementTree(new_robot).write(out, xml_declaration=True, encoding="unicode")
        print(f"[TiagoPickPlacePlanner] Arm-only URDF → '{out}'.")
        return out

    def _build_chain(self, arm_urdf_path: str) -> Optional[Chain]:
        try:
            arm_root   = ET.parse(arm_urdf_path).getroot()
            all_links  = {e.get("name") for e in arm_root.findall("link")}
            child_links = {
                j.find("child").get("link")
                for j in arm_root.findall("joint")
                if j.find("child") is not None
            }
            root_link = next(iter(all_links - child_links), "base_link")
        except Exception:
            root_link = "base_link"

        import warnings
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                chain = Chain.from_urdf_file(
                    arm_urdf_path,
                    base_elements=[root_link],
                    last_link_vector=[0, 0, 0.1],
                )
            revolute_idx = [
                i for i, lnk in enumerate(chain.links)
                if getattr(lnk, "joint_type", "fixed") == "revolute"
            ]
            print(
                f"[TiagoPickPlacePlanner] IK chain: {len(chain.links)} links, "
                f"{len(revolute_idx)} revolute joints at {revolute_idx}."
            )
            return chain
        except Exception as exc:
            print(f"[TiagoPickPlacePlanner] Chain load failed: {exc}")
            return None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _offset_z(xyz: XYZ, dz: float) -> XYZ:
    return (xyz[0], xyz[1], xyz[2] + dz)


def _make_pose(xyz: XYZ, quat_xyzw: tuple) -> Pose:
    pose = Pose()
    pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
    pose.orientation = Quaternion(
        x=quat_xyzw[0], y=quat_xyzw[1],
        z=quat_xyzw[2], w=quat_xyzw[3],
    )
    return pose


def _xyz_to_matrix(xyz: XYZ, quat_xyzw: tuple) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    mat = np.eye(4)
    mat[:3, 3]  = xyz
    mat[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    return mat


def _fmt(xyz: XYZ) -> str:
    return f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"