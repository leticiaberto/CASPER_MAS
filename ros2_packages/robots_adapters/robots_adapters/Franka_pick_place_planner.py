"""
Pure planning module for pick-and-place tasks.

Responsibilities
----------------
- Accepts (x, y, z) for pick and place positions.
- Computes IK for each waypoint using ikpy.
- Returns a PickPlanResult (list of steps) — no ROS2, no execution.

This is the ONLY file you change when swapping to Franky on the real robot.
The PickPlanResult contract stays the same; only the internals here change.

Swap guide (future)
-------------------
When using Franky on the physical FR3:
    1. Remove the ikpy IK calls below.
    2. For each MOVE step, set only `target_pose` (not `joint_trajectory`).
    3. The FrankaAdapter will call:
           CartesianWaypointMotion([CartesianWaypoint(Affine(x, y, z, ...)), ...])
       using those poses directly.
    4. No other file changes needed.

Robot URDF
----------
    franka_description must be available. The planner loads the URDF from:
        /ros2_ws/src/franka_description/robots/fr3/fr3.urdf
    Override via FRANKA_URDF_PATH environment variable.
"""

from __future__ import annotations

import math
import os
from typing import List, Optional, Tuple

import numpy as np

# ikpy — lightweight pure-Python IK, no ROS dependency
try:
    from ikpy.chain import Chain
    from ikpy.utils import geometry
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False

from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from Franka_pick_place_result import PickPlanResult, PickPlaceStep, StepKind

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

XYZ = Tuple[float, float, float]

# Approach height above the target (metres)
_APPROACH_HEIGHT = 0.12

# Time allocated per waypoint (seconds).
_WAYPOINT_DURATION_SEC = 5

# End-effector orientation for top-down grasping (x, y, z, w).
#
# FR3 hand TCP frame convention:
#   - In the neutral/home pose the gripper points forward (+x world)
#   - For top-down grasping we need the gripper Z-axis pointing DOWN (-z world)
#
# 180° rotation around X axis achieves this:
#   (x=1, y=0, z=0, w=0)
#
# If the robot approaches from the side instead of top-down, try:
#   (x=0.707, y=0, z=0, w=0.707)  →  90° around X
#   (x=0,     y=1, z=0, w=0)      →  180° around Y
#
_TOP_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)  # (x, y, z, w) — 180° around X

# Default xacro path — override with env var FRANKA_URDF_PATH
# Accepts both .urdf and .urdf.xacro — xacro files are processed at load time.
_DEFAULT_URDF = os.environ.get(
    "FRANKA_URDF_PATH",
    "/ros2_ws/src/franka_description/robots/fr3/fr3.urdf.xacro",
)

# FR3 joint names (must match controller config exactly)
_FR3_JOINT_NAMES = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7",
]

# FR3 home configuration (used as IK seed)
_FR3_HOME_JOINTS = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

# Franka Hand gripper widths (metres per finger)
_GRIPPER_OPEN   = 0.04
_GRIPPER_CLOSED = 0.00


# ---------------------------------------------------------------------------
# PickPlacePlanner
# ---------------------------------------------------------------------------

class PickPlacePlanner:
    """
    Computes a PickPlanResult for a pick-and-place task.

    Parameters
    ----------
    robot_name : str
        Robot namespace — used to prefix joint names in the trajectory
        (e.g. "fr3_robot1" → "fr3_robot1_fr3_joint1").
        Pass "" or None to use bare joint names.
    urdf_path : str, optional
        Path to the robot URDF. Defaults to FRANKA_URDF_PATH env var or
        the franka_description install path.
    approach_height : float
        How high above pick/place targets to approach (metres).
    base_height : float
        Height of the robot base above the world origin (metres).
        For example, if spawned at z=1.03, set base_height=1.03.
        All (x, y, z) coordinates passed to plan() are in the WORLD frame.
        The planner subtracts base_height from z before solving IK, since
        the URDF kinematic chain is in the robot BASE frame.
        Default: 0.0 (robot base at world origin).
    """

    def __init__(
        self,
        robot_name:       str,
        urdf_path:        Optional[str]   = None,
        approach_height:  float           = _APPROACH_HEIGHT,
        base_height:      float           = 0.0,
        eef_orientation:  Optional[tuple] = None,
    ) -> None:
        self._robot_name      = robot_name
        self._approach_height = approach_height
        self._base_height     = base_height
        self._urdf_path       = urdf_path or _DEFAULT_URDF
        # End-effector orientation for IK — overridable per instance.
        # Defaults to top-down (180° around X). Pass a (x,y,z,w) tuple
        # to change the approach angle.
        self._eef_orientation = eef_orientation or _TOP_DOWN_QUAT

        # Joint names with robot namespace prefix
        prefix = f"{robot_name}_" if robot_name else ""
        self._joint_names = [f"{prefix}{j}" for j in _FR3_JOINT_NAMES]

        # Load IK chain from URDF
        self._chain: Optional[Chain] = None
        if IKPY_AVAILABLE:
            self._chain = self._load_chain()
        else:
            print(
                "[PickPlacePlanner] WARNING: ikpy not installed. "
                "IK will use home pose for all targets (not suitable for real use)."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        pick_xyz:  XYZ,
        place_xyz: XYZ,
    ) -> PickPlanResult:
        """
        Compute a full pick-and-place plan.

        Returns a PickPlanResult with 10 steps:
            1. open_gripper
            2. move  → pick approach
            3. move  → pick pose
            4. close_gripper
            5. move  → pick approach (retreat)
            6. move  → place approach
            7. move  → place pose
            8. open_gripper
            9. move  → place approach (retreat)
           10. move  → home position

        Each MOVE step contains both:
            - target_pose       (Cartesian, for Franky / future real-robot use)
            - joint_trajectory  (IK-solved, for simulation JointTrajectoryController)
        """
        pick_approach  = _offset_z(pick_xyz,  self._approach_height)
        place_approach = _offset_z(place_xyz, self._approach_height)

        waypoints = [
            ("approach_pick",   pick_approach),
            ("pick",            pick_xyz),
            ("retreat_pick",    pick_approach),
            ("approach_place",  place_approach),
            ("place",           place_xyz),
            ("retreat_place",   place_approach),
        ]

        steps: List[PickPlaceStep] = []

        # Step 1 — open gripper
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_pre_pick",
            gripper_width = _GRIPPER_OPEN,
        ))

        # Steps 2-3 — approach and pick
        for label, xyz in waypoints[:2]:
            step = self._make_move_step(label, xyz)
            if step is None:
                return PickPlanResult.failure(
                    f"IK failed for waypoint '{label}' at xyz={xyz}"
                )
            steps.append(step)

        # Step 4 — close gripper
        steps.append(PickPlaceStep(
            kind          = StepKind.CLOSE_GRIPPER,
            label         = "close_gripper_grasp",
            gripper_width = _GRIPPER_CLOSED,
        ))

        # Steps 5-7 — retreat, approach place, descend
        for label, xyz in waypoints[2:5]:
            step = self._make_move_step(label, xyz)
            if step is None:
                return PickPlanResult.failure(
                    f"IK failed for waypoint '{label}' at xyz={xyz}"
                )
            steps.append(step)

        # Step 8 — open gripper (release)
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_release",
            gripper_width = _GRIPPER_OPEN,
        ))

        # Step 9 — retreat from place
        step = self._make_move_step(*waypoints[5])
        if step is None:
            return PickPlanResult.failure(
                f"IK failed for waypoint 'retreat_place' at xyz={waypoints[5][1]}"
            )
        steps.append(step)

        # Step 10 — return to home position
        # Uses home joint positions directly — no IK needed.
        home_trajectory = self._make_trajectory(
            _FR3_HOME_JOINTS[:],
            current_positions=_FR3_HOME_JOINTS[:],  # seed from home itself
        )
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "return_home",
            target_pose      = None,   # Franky handles home natively
            joint_trajectory = home_trajectory,
        ))

        return PickPlanResult(
            steps   = steps,
            success = True,
            message = f"Plan ready: {len(steps)} steps.",
        )

    # ------------------------------------------------------------------
    # Internal — step construction
    # ------------------------------------------------------------------

    def _make_move_step(self, label: str, xyz: XYZ) -> Optional[PickPlaceStep]:
        """
        Build a MOVE step for a given Cartesian target.

        Populates both:
        - target_pose       → for Franky (real robot, future)
        - joint_trajectory  → for simulation (ikpy IK → JointTrajectory)
        """
        pose = _make_pose(xyz, self._eef_orientation)

        # IK solve
        joint_positions = self._solve_ik(xyz)
        if joint_positions is None:
            return None

        trajectory = self._make_trajectory(joint_positions)

        return PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = label,
            target_pose      = pose,        # Franky will use this
            joint_trajectory = trajectory,  # Sim adapter will use this
        )

    def _solve_ik(self, xyz: XYZ) -> Optional[List[float]]:
        """
        Solve IK for a Cartesian target (top-down orientation).

        xyz is in the WORLD frame. We subtract base_height from z to convert
        to the robot BASE frame before solving, since the URDF chain origin
        is the robot base (link0), not the world origin.

        Returns a list of 7 joint positions [rad], or None if IK failed.
        Falls back to home pose if ikpy is not available.
        """
        if not IKPY_AVAILABLE or self._chain is None:
            print(
                f"[PickPlacePlanner] ikpy unavailable — "
                f"using home joints for {xyz} (simulation only, not accurate)"
            )
            return _FR3_HOME_JOINTS[:]

        # Convert world-frame target to robot-base frame
        xyz_base = (xyz[0], xyz[1], xyz[2] - self._base_height)
        print(
            f"[PickPlacePlanner] IK target: world={xyz} → base={xyz_base}"
        )

        # Build target transform in robot base frame
        target_matrix = _xyz_to_matrix(xyz_base, self._eef_orientation)

        # Seed must exactly match chain length
        n = len(self._chain.links)
        seed = [0.0] * n
        # Find revolute joint indices and seed them with home values
        revolute_indices = [
            i for i, l in enumerate(self._chain.links)
            if getattr(l, "joint_type", "fixed") == "revolute"
        ]
        for k, idx in enumerate(revolute_indices):
            if k < len(_FR3_HOME_JOINTS):
                seed[idx] = _FR3_HOME_JOINTS[k]

        try:
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                ik_result = self._chain.inverse_kinematics_frame(
                    target           = target_matrix,
                    initial_position = seed,
                    orientation_mode = "Y",
                )
        except Exception as exc:
            print(f"[PickPlacePlanner] IK error for xyz={xyz}: {exc}")
            return None

        # Extract only the revolute joint positions in order
        joint_positions = [ik_result[i] for i in revolute_indices]

        if len(joint_positions) != 7:
            print(
                f"[PickPlacePlanner] Expected 7 joint positions, "
                f"got {len(joint_positions)}. IK chain may be wrong."
            )
            return None

        # Sanity check: FK position error must be under 2 cm (in base frame)
        fk = self._chain.forward_kinematics(ik_result)
        error = np.linalg.norm(fk[:3, 3] - np.array(xyz_base))
        if error > 0.02:
            print(
                f"[PickPlacePlanner] IK position error too large: "
                f"{error*100:.1f}cm for base-frame target {xyz_base}"
            )
            return None

        return joint_positions

    def _make_trajectory(
        self,
        joint_positions:   List[float],
        current_positions: Optional[List[float]] = None,  # kept for API compatibility
    ) -> JointTrajectory:
        """
        Build a single-point JointTrajectory to the target joint positions.

        The controller uses spline interpolation internally, so a single
        target point with zero end velocity produces a smooth motion.
        The previous two-point approach caused backtracking because the
        midpoint was always biased toward home joints.
        """
        traj = JointTrajectory()
        traj.joint_names = self._joint_names

        pt = JointTrajectoryPoint()
        pt.positions      = joint_positions
        pt.velocities     = [0.0] * 7
        pt.accelerations  = [0.0] * 7
        pt.time_from_start = Duration(sec=_WAYPOINT_DURATION_SEC, nanosec=0)

        traj.points = [pt]
        return traj

    # ------------------------------------------------------------------
    # Internal — URDF / chain loading
    # ------------------------------------------------------------------

    def _load_chain(self) -> Optional[Chain]:
        """
        Load the ikpy kinematic chain for the FR3 arm.

        Strategy
        --------
        ikpy always follows the first child branch from the root, which picks
        up accelerometer/sensor branches instead of the arm joints. No amount
        of base_elements or active_links_mask fixes this when the sensor branch
        comes before the arm branch alphabetically.

        Instead we extract just the arm links and joints from the full URDF
        into a minimal stand-alone URDF (9 links, 8 joints) and load that.
        This gives ikpy a linear chain with no branches to get confused by.

        Arm path: base → link0 → link1 → ... → link7 → hand
        """
        if not os.path.exists(self._urdf_path):
            print(
                f"[PickPlacePlanner] URDF/xacro not found at '{self._urdf_path}'. "
                "Set FRANKA_URDF_PATH env var to the correct path."
            )
            return None

        urdf_path = self._urdf_path
        if urdf_path.endswith(".xacro"):
            urdf_path = self._process_xacro(urdf_path)
            if urdf_path is None:
                return None

        arm_urdf_path = self._extract_arm_urdf(urdf_path)
        if arm_urdf_path is None:
            return None

        # Find the actual root link name in the arm URDF (no parent joint)
        import xml.etree.ElementTree as ET
        import warnings
        try:
            arm_root = ET.parse(arm_urdf_path).getroot()
            all_arm_links  = {e.get("name") for e in arm_root.findall("link")}
            arm_child_links = {j.find("child").get("link")
                               for j in arm_root.findall("joint")
                               if j.find("child") is not None}
            root_candidates = list(all_arm_links - arm_child_links)
            arm_root_link   = root_candidates[0] if root_candidates else "base"
            print(f"[PickPlacePlanner] Arm URDF root link: '{arm_root_link}'")
        except Exception:
            arm_root_link = "base"

        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                chain = Chain.from_urdf_file(
                    arm_urdf_path,
                    base_elements=[arm_root_link],  # explicit root — avoids "base_link not found"
                    last_link_vector=[0, 0, 0.1],
                )

            revolute_idx = [
                i for i, l in enumerate(chain.links)
                if getattr(l, "joint_type", "fixed") == "revolute"
            ]
            print(
                f"[PickPlacePlanner] IK chain ready: {len(chain.links)} links, "
                f"{len(revolute_idx)} revolute joints at indices {revolute_idx}."
            )
            print(
                f"[PickPlacePlanner] Chain links: "
                + ", ".join(f"[{i}]{l.name}({getattr(l,'joint_type','?')})"
                            for i, l in enumerate(chain.links))
            )
            if len(revolute_idx) != 7:
                print(
                    f"[PickPlacePlanner] WARNING: expected 7 revolute joints, "
                    f"got {len(revolute_idx)}."
                )
            return chain

        except Exception as exc:
            print(f"[PickPlacePlanner] Failed to load IK chain: {exc}")
            return None

    def _extract_arm_urdf(self, urdf_path: str) -> Optional[str]:
        """
        Parse the full URDF and write a minimal arm-only URDF by walking
        the kinematic tree from root → fr3_hand, keeping only the links and
        joints on the direct path to the hand.

        This works regardless of what extra links are added (mount anchors,
        sensors, etc.) and regardless of naming conventions.
        """
        import xml.etree.ElementTree as ET
        import tempfile

        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
        except Exception as exc:
            print(f"[PickPlacePlanner] Failed to parse URDF: {exc}")
            return None

        prefix = f"{self._robot_name}_" if self._robot_name else ""

        # The tip link we want to reach — try with prefix first, then bare
        tip_candidates = [
            f"{prefix}fr3_hand",
            "fr3_hand",
            f"{prefix}fr3_link7",
            "fr3_link7",
        ]
        all_link_names = {e.get("name") for e in root.findall("link")}
        tip_link = next((t for t in tip_candidates if t in all_link_names), None)
        if tip_link is None:
            print(
                f"[PickPlacePlanner] Could not find hand/tip link in URDF.\n"
                f"  Tried: {tip_candidates}\n"
                f"  Available: {sorted(all_link_names)}"
            )
            return None

        # Build parent→child map and child→parent map from joints
        # joint_map: child_link → (parent_link, joint_element)
        joint_map: dict = {}
        for joint in root.findall("joint"):
            p = joint.find("parent")
            c = joint.find("child")
            if p is None or c is None:
                continue
            joint_map[c.get("link")] = (p.get("link"), joint)

        # Walk backwards from tip to root, collecting the path
        path_links  = []
        path_joints = []
        current = tip_link
        while current in joint_map:
            parent, joint_elem = joint_map[current]
            path_links.append(current)
            path_joints.append(joint_elem)
            current = parent
        path_links.append(current)  # add the root link

        # Reverse so order is root → tip
        path_links.reverse()
        path_joints.reverse()

        print(
            f"[PickPlacePlanner] Arm path ({len(path_links)} links, "
            f"{len(path_joints)} joints):\n"
            f"  " + " → ".join(path_links)
        )

        # Collect the actual <link> elements for each link on the path
        link_map = {e.get("name"): e for e in root.findall("link")}
        arm_links  = [link_map[l] for l in path_links if l in link_map]
        arm_joints = path_joints

        if len(arm_links) != len(path_links):
            missing = [l for l in path_links if l not in link_map]
            print(f"[PickPlacePlanner] Missing link elements: {missing}")
            return None

        # Count revolute joints for sanity
        revolute_count = sum(
            1 for j in arm_joints if j.get("type") == "revolute"
        )
        print(
            f"[PickPlacePlanner] Arm URDF: {len(arm_links)} links, "
            f"{len(arm_joints)} joints ({revolute_count} revolute)."
        )
        if revolute_count != 7:
            print(
                f"[PickPlacePlanner] WARNING: expected 7 revolute joints "
                f"on path to hand, got {revolute_count}."
            )

        # Write minimal arm-only URDF
        new_robot = ET.Element("robot")
        new_robot.set("name", "fr3_arm")
        for link in arm_links:
            new_robot.append(link)
        for joint in arm_joints:
            new_robot.append(joint)

        out_path = os.path.join(tempfile.gettempdir(), "fr3_arm_only.urdf")
        ET.ElementTree(new_robot).write(
            out_path, xml_declaration=True, encoding="unicode"
        )
        print(f"[PickPlacePlanner] Arm-only URDF → '{out_path}'.")
        return out_path

    def _process_xacro(self, xacro_path: str) -> Optional[str]:
        """
        Run xacro on the given file, passing arm_prefix so the full robot
        URDF is generated with correctly namespaced link/joint names.
        Returns the path to the generated URDF, or None on failure.
        """
        import subprocess
        import tempfile

        out_path = os.path.join(tempfile.gettempdir(), "fr3_processed.urdf")

        # Pass the same arm_prefix used by the launch file so joint names
        # in the URDF match what the controllers expect.
        prefix = self._robot_name if self._robot_name else "fr3"
        cmd = [
            "xacro", xacro_path,
            f"arm_prefix:={prefix}",
            f"robot_namespace:={prefix}",
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True,
            )
            with open(out_path, "w") as f:
                f.write(result.stdout)
            print(f"[PickPlacePlanner] xacro processed → '{out_path}'.")
            return out_path

        except subprocess.CalledProcessError as exc:
            print(
                f"[PickPlacePlanner] xacro processing failed:\n{exc.stderr}"
            )
            return None
        except FileNotFoundError:
            print(
                "[PickPlacePlanner] 'xacro' command not found. "
                "Install with: sudo apt install ros-humble-xacro"
            )
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
    """Build a 4x4 homogeneous transform from position + quaternion."""
    from scipy.spatial.transform import Rotation
    mat = np.eye(4)
    mat[:3, 3] = xyz
    mat[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    return mat