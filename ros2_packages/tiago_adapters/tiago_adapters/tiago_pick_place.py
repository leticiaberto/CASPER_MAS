"""
Pure planning module for Tiago pick-and-place tasks.

Responsibilities
----------------
- Accepts world-frame (x, y, z) for pick and place positions only.
  The caller does NOT provide navigation poses — the planner figures those out.
- Checks if the target is already within arm reach from the robot's current pose.
  If yes, no navigation step is generated.
- If not reachable, computes an optimal base position that:
    • Places the target at a comfortable arm extension (~0.5 m forward in arm frame).
    • Keeps the robot away from all known scene obstacles.
    • Tries up to 12 approach angles (every 30°) if the preferred angle is blocked.
- Converts world-frame targets to the ARM-BASE frame for IK.
- Returns a PickPlanResult — no ROS2, no execution.

Coordinate frames
-----------------
All (x, y, z) inputs are in the WORLD frame (same as the Nav2 map).

After the robot is at pose (nav_x, nav_y, nav_theta), a world point P is in
the arm-base frame as:

    dx, dy = P.xy - (nav_x, nav_y)
    x_arm  =  dx * cos(theta) + dy * sin(theta)
    y_arm  = -dx * sin(theta) + dy * cos(theta)
    z_arm  =  P.z - arm_base_z

where arm_base_z ≈ 0.83 m (arm_1_link height above the floor at default torso).

Arm reach envelope (conservative, for reliable manipulation)
------------------------------------------------------------
A point (x_arm, y_arm, z_arm) is considered reachable if:
    REACH_MIN_HORIZ ≤ sqrt(x²+y²) ≤ REACH_MAX_HORIZ  (horizontal distance)
    Z_ARM_MIN       ≤ z_arm        ≤ Z_ARM_MAX          (height in arm frame)
    x_arm           > 0                                  (in front of the robot)

These values are conservative — the actual IK solver will confirm feasibility.

Obstacle handling
-----------------
The caller passes a list of Obstacle(x, y, radius) objects describing the 2D
footprint of scene objects in the world frame.  The planner tries to place the
robot at least (ROBOT_RADIUS + SAFETY_MARGIN + obstacle.radius) away from each
obstacle.  The robot itself is ignored (matched by robot_name prefix).

URDF
----
Loaded from TIAGO_URDF_PATH env var or ament_index, same as before.
"""

from __future__ import annotations

import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
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


@dataclass
class Obstacle:
    """2D circular footprint of a scene object in the world frame."""
    name:   str
    x:      float
    y:      float
    radius: float   # metres — approximation of the object's footprint


# ---------------------------------------------------------------------------
# Arm reach constants (conservative values for reliable simulation manipulation)
# ---------------------------------------------------------------------------

# Horizontal distance from arm_1_link (directly above base_footprint)
_REACH_MAX_HORIZ = 0.75   # m — max comfortable horizontal reach
_REACH_MIN_HORIZ = 0.12   # m — too close to body

# Height in arm-base frame (positive = above arm_1_link)
_Z_ARM_MAX =  0.55    # m
_Z_ARM_MIN = -0.40    # m

# Preferred horizontal distance in arm frame (comfortable extension)
_PREFERRED_REACH = 0.50   # m

# Robot footprint radius (PMB2 base, conservative)
_ROBOT_RADIUS = 0.45   # m

# Clearance added on top of obstacle radius when checking nav pose
_SAFETY_MARGIN = 0.10   # m

# How many approach angles to try around the full circle (every 30°)
_N_APPROACH_ANGLES = 12

# ---------------------------------------------------------------------------
# Other constants
# ---------------------------------------------------------------------------

_APPROACH_HEIGHT        = 0.12   # m above target for pre-grasp / pre-place
_WAYPOINT_DURATION_SEC  = 10     # s per arm trajectory waypoint (pick/place moves)
_TUCK_DURATION_SEC      = 20     # s for tuck — larger movement from any starting pose
_TOP_DOWN_QUAT         = (1.0, 0.0, 0.0, 0.0)  # 180° around X → gripper down

_DEFAULT_URDF = os.environ.get(
    "TIAGO_URDF_PATH",
    "/ros2_ws/src/tiago_description/robots/tiago.urdf.xacro",
)

_ARM_JOINTS = [
    "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
    "arm_5_joint", "arm_6_joint", "arm_7_joint",
]

_ARM_HOME_JOINTS = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

# Safe tucked position — PAL Tiago home pose.
# arm_2=-1.34 rotates the upper arm back, arm_4=1.94 folds the forearm UP
# (not down), keeping the gripper well above the floor during navigation.
_ARM_TUCK_JOINTS = [0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

_GRIPPER_OPEN   = 0.044   # m per finger
_GRIPPER_CLOSED = 0.002

_ARM_TIP_LINK = "arm_tool_link"


# ---------------------------------------------------------------------------
# TiagoPickPlacePlanner
# ---------------------------------------------------------------------------

class TiagoPickPlacePlanner:
    """
    Computes a PickPlanResult for a Tiago pick-and-place task.

    Parameters
    ----------
    robot_name : str
        Robot namespace (e.g. "tiago_robot1"). Used for logging and to
        exclude the robot itself from the obstacle list.
    urdf_path : str, optional
        Path to Tiago URDF/xacro. Defaults to TIAGO_URDF_PATH env var.
    approach_height : float
        How far above pick/place targets to approach (m). Default 0.12.
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83 m (Tiago
        with default torso). Increase if torso_lift_joint is raised.
    preferred_reach : float
        Preferred horizontal distance (m) in the arm-base frame to the
        target when computing a navigation pose. Default 0.50 m.
    """

    def __init__(
        self,
        robot_name:      str,
        urdf_path:       Optional[str] = None,
        approach_height: float         = _APPROACH_HEIGHT,
        arm_base_z:      float         = 0.83,
        preferred_reach: float         = _PREFERRED_REACH,
    ) -> None:
        self._robot_name      = robot_name
        self._approach_height = approach_height
        self._arm_base_z      = arm_base_z
        self._preferred_reach = preferred_reach
        self._urdf_path       = urdf_path or self._find_urdf()

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
        pick_xyz:   XYZ,
        place_xyz:  XYZ,
        robot_pose: NavPose,
        obstacles:  Optional[List[Obstacle]] = None,
    ) -> PickPlanResult:
        """
        Compute a full pick-and-place plan.

        Parameters
        ----------
        pick_xyz : (x, y, z)
            World-frame position of the object to pick.
        place_xyz : (x, y, z)
            World-frame position to place the object.
        robot_pose : (x, y, theta)
            Current robot base pose in the world frame.
        obstacles : list of Obstacle, optional
            Scene objects to avoid when computing navigation goals.
            Objects whose name starts with self._robot_name are ignored.

        The planner automatically decides whether navigation is needed:
        - If pick_xyz is already reachable from robot_pose, no NAV step
          is generated for pick.
        - After picking (robot is now at pick_nav or still at robot_pose),
          same logic applies for place_xyz.

        Returns a PickPlanResult with up to 12 steps.
        """
        obstacles = obstacles or []
        obs_filtered = [o for o in obstacles
                        if not o.name.startswith(self._robot_name)]

        steps: List[PickPlaceStep] = []

        # ── Tuck arm at start so the robot can navigate safely ───────
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "tuck_arm_initial",
            target_pose      = None,
            joint_trajectory = self._make_tuck_trajectory(),
        ))

        # ── Decide pick nav pose ────────────────────────────────────
        pick_nav = self._compute_nav_pose(pick_xyz, robot_pose, obs_filtered)
        pose_at_pick = pick_nav if pick_nav is not None else robot_pose

        if pick_nav is not None:
            # Tuck before navigating so arm doesn't hit environment
            steps.append(PickPlaceStep(
                kind     = StepKind.NAVIGATE,
                label    = "navigate_to_pick",
                nav_goal = pick_nav,
            ))
            print(
                f"[TiagoPickPlacePlanner] Navigation to pick needed: "
                f"({pick_nav[0]:.2f}, {pick_nav[1]:.2f}, "
                f"{math.degrees(pick_nav[2]):.1f}°)"
            )
        else:
            print(
                "[TiagoPickPlacePlanner] Pick reachable from current pose "
                "— no navigation needed."
            )

        # ── Open gripper ────────────────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_pre_pick",
            gripper_width = _GRIPPER_OPEN,
        ))

        # ── Approach and pick ────────────────────────────────────────
        pick_approach_world = _offset_z(pick_xyz, self._approach_height)
        for label, xyz_world in [
            ("approach_pick", pick_approach_world),
            ("pick",          pick_xyz),
        ]:
            xyz_arm = self._world_to_arm(xyz_world, pose_at_pick)
            step = self._make_move_step(label, xyz_arm, xyz_world)
            if step is None:
                return PickPlanResult.failure(
                    f"IK failed for '{label}' "
                    f"(world={xyz_world}, arm={_fmt(xyz_arm)})"
                )
            steps.append(step)

        # ── Close gripper ────────────────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.CLOSE_GRIPPER,
            label         = "close_gripper_grasp",
            gripper_width = _GRIPPER_CLOSED,
        ))

        # ── Retreat up from pick ─────────────────────────────────────
        xyz_arm = self._world_to_arm(pick_approach_world, pose_at_pick)
        step = self._make_move_step("retreat_pick", xyz_arm, pick_approach_world)
        if step is None:
            return PickPlanResult.failure("IK failed for 'retreat_pick'")
        steps.append(step)

        # ── Decide place nav pose ────────────────────────────────────
        # Robot is now at pose_at_pick; check reachability from there.
        place_nav = self._compute_nav_pose(place_xyz, pose_at_pick, obs_filtered)
        pose_at_place = place_nav if place_nav is not None else pose_at_pick

        if place_nav is not None:
            # Tuck before navigating so arm doesn't hit environment
            steps.append(PickPlaceStep(
                kind             = StepKind.MOVE,
                label            = "tuck_arm_pre_place_nav",
                target_pose      = None,
                joint_trajectory = self._make_tuck_trajectory(),
            ))
            steps.append(PickPlaceStep(
                kind     = StepKind.NAVIGATE,
                label    = "navigate_to_place",
                nav_goal = place_nav,
            ))
            print(
                f"[TiagoPickPlacePlanner] Navigation to place needed: "
                f"({place_nav[0]:.2f}, {place_nav[1]:.2f}, "
                f"{math.degrees(place_nav[2]):.1f}°)"
            )
        else:
            print(
                "[TiagoPickPlacePlanner] Place reachable from pick pose "
                "— no navigation needed."
            )

        # ── Approach and place ───────────────────────────────────────
        place_approach_world = _offset_z(place_xyz, self._approach_height)
        for label, xyz_world in [
            ("approach_place", place_approach_world),
            ("place",          place_xyz),
        ]:
            xyz_arm = self._world_to_arm(xyz_world, pose_at_place)
            step = self._make_move_step(label, xyz_arm, xyz_world)
            if step is None:
                return PickPlanResult.failure(
                    f"IK failed for '{label}' "
                    f"(world={xyz_world}, arm={_fmt(xyz_arm)})"
                )
            steps.append(step)

        # ── Open gripper (release) ───────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_release",
            gripper_width = _GRIPPER_OPEN,
        ))

        # ── Retreat from place ───────────────────────────────────────
        xyz_arm = self._world_to_arm(place_approach_world, pose_at_place)
        step = self._make_move_step("retreat_place", xyz_arm, place_approach_world)
        if step is None:
            return PickPlanResult.failure("IK failed for 'retreat_place'")
        steps.append(step)

        # ── Tuck arm back to home ─────────────────────────────────────
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "return_home",
            target_pose      = None,
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
        Return True if target_xyz is within the arm's reach envelope
        from the given robot_pose, without needing to navigate.

        Uses a conservative geometric envelope — the IK solver does the
        final confirmation when building each trajectory step.
        """
        x_arm, y_arm, z_arm = self._world_to_arm(target_xyz, robot_pose)

        horiz = math.sqrt(x_arm ** 2 + y_arm ** 2)

        in_horiz = _REACH_MIN_HORIZ <= horiz <= _REACH_MAX_HORIZ
        in_z     = _Z_ARM_MIN <= z_arm <= _Z_ARM_MAX
        in_front = x_arm > 0   # must be in front of the robot

        return in_horiz and in_z and in_front

    # ------------------------------------------------------------------
    # Nav pose computation
    # ------------------------------------------------------------------

    def _compute_nav_pose(
        self,
        target_xyz: XYZ,
        robot_pose: NavPose,
        obstacles:  List[Obstacle],
    ) -> Optional[NavPose]:
        """
        Return the optimal base pose to reach target_xyz, or None if the
        target is already reachable from robot_pose.

        Strategy
        --------
        1. Check if already reachable — return None if yes.
        2. Compute the preferred approach angle (facing the target).
        3. Try _N_APPROACH_ANGLES evenly-spaced angles around the full circle.
           For each angle, compute the candidate nav pose (stand at
           _preferred_reach in front of the target) and check for obstacle
           collisions.
        4. Return the first collision-free candidate, or the nearest candidate
           ignoring obstacles if all are blocked.
        """
        if self.is_reachable(target_xyz, robot_pose):
            return None

        tx, ty = target_xyz[0], target_xyz[1]
        rx, ry = robot_pose[0], robot_pose[1]

        # Preferred approach: robot faces toward the target from its current side
        base_angle = math.atan2(ty - ry, tx - rx)

        candidates: List[NavPose] = []
        for i in range(_N_APPROACH_ANGLES):
            angle = base_angle + i * (2 * math.pi / _N_APPROACH_ANGLES)
            # Position robot at preferred_reach behind the target along this angle
            nav_x = tx - self._preferred_reach * math.cos(angle)
            nav_y = ty - self._preferred_reach * math.sin(angle)
            nav_theta = angle

            # Sanity: confirm the target is now reachable from this candidate
            candidate = (nav_x, nav_y, nav_theta)
            if not self.is_reachable(target_xyz, candidate):
                continue

            if not self._collides(nav_x, nav_y, obstacles):
                return candidate   # first collision-free option wins

            candidates.append(candidate)

        # All options collide — return the geometrically closest one (let
        # Nav2 handle obstacle avoidance at execution time)
        if candidates:
            print(
                "[TiagoPickPlacePlanner] WARNING: all approach angles have "
                "obstacle conflicts — returning nearest candidate; "
                "Nav2 will route around obstacles at runtime."
            )
            # Pick the candidate closest to the current robot position
            best = min(candidates,
                       key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
            return best

        # No valid candidate at all — try a wider search at max reach
        print(
            "[TiagoPickPlacePlanner] WARNING: no reachable nav pose found at "
            f"preferred_reach={self._preferred_reach:.2f} m — "
            "target may be unreachable."
        )
        # Fallback: use closest geometric option regardless
        angle = base_angle
        nav_x = tx - _REACH_MAX_HORIZ * 0.8 * math.cos(angle)
        nav_y = ty - _REACH_MAX_HORIZ * 0.8 * math.sin(angle)
        return (nav_x, nav_y, angle)

    def _collides(self, nav_x: float, nav_y: float, obstacles: List[Obstacle]) -> bool:
        """Return True if placing the robot at (nav_x, nav_y) would overlap an obstacle."""
        for obs in obstacles:
            dist = math.hypot(nav_x - obs.x, nav_y - obs.y)
            min_clearance = _ROBOT_RADIUS + _SAFETY_MARGIN + obs.radius
            if dist < min_clearance:
                return True
        return False

    # ------------------------------------------------------------------
    # Coordinate transform
    # ------------------------------------------------------------------

    def _world_to_arm(self, xyz_world: XYZ, robot_pose: NavPose) -> XYZ:
        """
        Convert a world-frame point to the Tiago arm-base frame.

        After robot drives to (nav_x, nav_y, nav_theta):
          - Translate by robot base XY.
          - Rotate by -nav_theta.
          - Subtract arm-base height from z.
        """
        nav_x, nav_y, nav_theta = robot_pose
        dx = xyz_world[0] - nav_x
        dy = xyz_world[1] - nav_y

        cos_t = math.cos(nav_theta)
        sin_t = math.sin(nav_theta)

        x_arm =  dx * cos_t + dy * sin_t
        y_arm = -dx * sin_t + dy * cos_t
        z_arm = xyz_world[2] - self._arm_base_z

        return (x_arm, y_arm, z_arm)

    # ------------------------------------------------------------------
    # Step construction
    # ------------------------------------------------------------------

    def _make_move_step(
        self,
        label:     str,
        xyz_arm:   XYZ,
        xyz_world: XYZ,
    ) -> Optional[PickPlaceStep]:
        world_pose = _make_pose(xyz_world, _TOP_DOWN_QUAT)
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
            print(
                f"[TiagoPickPlacePlanner] ikpy unavailable — "
                f"using home joints for {_fmt(xyz_arm)}."
            )
            return _ARM_HOME_JOINTS[:]

        target_matrix = _xyz_to_matrix(xyz_arm, _TOP_DOWN_QUAT)

        n = len(self._chain.links)
        revolute_idx = [
            i for i, lnk in enumerate(self._chain.links)
            if getattr(lnk, "joint_type", "fixed") == "revolute"
        ]

        # Build a set of seeds: home pose + a few neutral variants.
        # ikpy's numerical solver is seed-sensitive; trying several seeds
        # and keeping the best result reduces FK error significantly.
        seeds = []
        for home_scale in (1.0, 0.5, 0.0):
            s = [0.0] * n
            for k, idx in enumerate(revolute_idx):
                if k < len(_ARM_HOME_JOINTS):
                    s[idx] = _ARM_HOME_JOINTS[k] * home_scale
            seeds.append(s)

        import warnings
        best_joints = None
        best_error  = float("inf")

        for seed in seeds:
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    # orientation_mode=None: solve for position only.
                    # "Y" or "all" prioritises orientation matching at the
                    # cost of position accuracy on Tiago's 7-DOF arm.
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

        # 8 cm tolerance: position-only IK on Tiago's 7-DOF arm converges to
        # 2–6 cm depending on the target; the joint controller will still reach
        # the joint angles precisely, so small FK residuals are acceptable in sim.
        if best_error > 0.08:
            print(
                f"[TiagoPickPlacePlanner] IK position error {best_error*100:.1f} cm > 8 cm "
                f"for {_fmt(xyz_arm)} — rejecting."
            )
            return None

        print(
            f"[TiagoPickPlacePlanner] IK solved {_fmt(xyz_arm)}: "
            f"FK error {best_error*100:.1f} cm."
        )
        return best_joints

    def _make_tuck_trajectory(self) -> JointTrajectory:
        return self._make_trajectory(_ARM_TUCK_JOINTS[:], duration_sec=_TUCK_DURATION_SEC)

    def _make_trajectory(self, joint_positions: List[float], duration_sec: int = _WAYPOINT_DURATION_SEC) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions       = joint_positions
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)

        traj.points = [pt]
        return traj

    # ------------------------------------------------------------------
    # URDF loading (unchanged from before)
    # ------------------------------------------------------------------

    def _find_urdf(self) -> str:
        candidates = [
            _DEFAULT_URDF,
            "/ros2_ws/src/tiago_robot/tiago_description/robots/tiago.urdf.xacro",
        ]
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
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
            print(
                f"[TiagoPickPlacePlanner] URDF not found at '{self._urdf_path}'. "
                "Set TIAGO_URDF_PATH to override."
            )
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
        out_path = os.path.join(tempfile.gettempdir(), "tiago_processed.urdf")
        cmd = ["xacro", xacro_path, "arm:=True", "end_effector:=pal-gripper"]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            with open(out_path, "w") as f:
                f.write(result.stdout)
            print(f"[TiagoPickPlacePlanner] xacro → '{out_path}'.")
            return out_path
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
        tip_link = next(
            (t for t in [_ARM_TIP_LINK, "arm_7_link"] if t in all_links),
            None,
        )
        if tip_link is None:
            print(
                f"[TiagoPickPlacePlanner] Tip link not found. "
                f"Available: {sorted(all_links)}"
            )
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

        revolute_count = sum(1 for j in path_joints if j.get("type") == "revolute")
        print(
            f"[TiagoPickPlacePlanner] Arm path: {len(path_links)} links, "
            f"{revolute_count} revolute joints."
        )

        link_map = {e.get("name"): e for e in root.findall("link")}
        new_robot = ET.Element("robot")
        new_robot.set("name", "tiago_arm")
        for lnk_name in path_links:
            if lnk_name in link_map:
                new_robot.append(link_map[lnk_name])
        for jnt in path_joints:
            new_robot.append(jnt)

        out_path = os.path.join(tempfile.gettempdir(), "tiago_arm_only.urdf")
        ET.ElementTree(new_robot).write(
            out_path, xml_declaration=True, encoding="unicode"
        )
        print(f"[TiagoPickPlacePlanner] Arm-only URDF → '{out_path}'.")
        return out_path

    def _build_chain(self, arm_urdf_path: str) -> Optional[Chain]:
        try:
            arm_root = ET.parse(arm_urdf_path).getroot()
            all_arm_links   = {e.get("name") for e in arm_root.findall("link")}
            arm_child_links = {
                j.find("child").get("link")
                for j in arm_root.findall("joint")
                if j.find("child") is not None
            }
            root_link = next(iter(all_arm_links - arm_child_links), "base_link")
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
