"""
Pure planning module for Tiago pick-and-place tasks.

Grasp strategy
--------------
Objects are grasped at their **centre** (Gazebo world_pose z) using a
**horizontal front approach** reached from an **elbow-up raised pre-grasp pose**:

1. The arm is tucked and the base navigates to a safe table-standoff position
   facing the object.
2. Pre-grasp staging raises the arm to an elbow-up configuration
   (arm_2 ≈ +0.50 rad above horizontal, arm_4 ≈ 1.50 rad bent) with
   a level (horizontal) gripper orientation.  This matches the first frame
   of the reference pick sequence: arm elevated, elbow bent forward.
3. The gripper opens.  The arm descends diagonally in three waypoints:
       pre-approach  → approach (hover)  → grasp (object centre)
   WP 0 is PRE_APPROACH_DIST behind the object AND PRE_APPROACH_Z_LIFT
   above object z — starting from the physically elevated pre-grasp pose
   so the first IK waypoint is close to the robot's actual arm state.
   WP 1 / WP 2 drop to object height along the diagonal.
4. The gripper closes.
5. The arm retraces the approach in reverse before navigating to place.

Place follows the same extend / retract pattern at the destination surface
height.

Coordinate modes
----------------
World-frame mode  (robot_frame=False, default)
    pick_xyz / place_xyz are in the WORLD frame.
    The planner computes navigation poses and transforms targets internally.

Robot-frame mode  (robot_frame=True)
    pick_xyz / place_xyz are already in the ROBOT BASE frame, as returned by
    object_to_robot.ObjectToRobot.get_pose()["robot_pose"].position.
    Navigation steps are suppressed; only arm and gripper move.

Arm-frame convention
--------------------
After the robot is at pose (nav_x, nav_y, nav_theta), a world point P is in
the arm-base frame as:

    dx, dy = P.xy - (nav_x, nav_y)
    x_arm  =  dx * cos(theta) + dy * sin(theta)
    y_arm  = -dx * sin(theta) + dy * cos(theta)
    z_arm  =  P.z - arm_base_z

where arm_base_z ≈ 0.83 m (arm_1_link height above the floor at default torso).
In robot-frame mode the transform collapses to x_arm=x, y_arm=y, z_arm=z-arm_base_z.

Approach direction
------------------
The planner keeps the robot facing the object (nav pose ensures this), so the
approach direction is always +x in the arm frame.  Waypoints are spaced along
this axis:

    pre_approach_xyz = grasp_xyz - PRE_APPROACH_DIST * heading   (world frame)
    approach_xyz     = grasp_xyz - APPROACH_DIST     * heading

In robot-frame mode the heading is simply +x, so the offset is (-dist, 0, 0).

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
from sdf_table_resolver import SdfTableResolver

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
_PREFERRED_REACH =  0.70  # m — comfortable arm reach, keeps base clear of table
_N_APPROACH_ANGLES = 12

# ---------------------------------------------------------------------------
# Motion constants
# ---------------------------------------------------------------------------

# Horizontal distances along the approach direction (robot heading → object).
# WP layout for pick/place motion (3 waypoints):
#   WP 0 — pre_approach: PRE_APPROACH_DIST behind the object AND PRE_APPROACH_Z_LIFT
#           above object z.  The arm descends diagonally from the elevated pre-grasp
#           staging pose.  Starting above the table plane eliminates table-edge
#           collisions during the approach.
#   WP 1 — approach:     APPROACH_DIST behind the object AT object z.  Hover
#           and decelerate before contact.
#   WP 2 — grasp:        object centre.
_PRE_APPROACH_DIST   = 0.25   # m — horizontal offset behind object at WP 0
_APPROACH_DIST       = 0.10   # m — horizontal offset behind object at WP 1
_PRE_APPROACH_Z_LIFT = 0.15   # m — WP 0 is this far ABOVE object z.
                               #     15 cm provides clearance above the
                               #     object during horizontal motion while
                               #     keeping the arm comfortably within
                               #     reach.  Larger lifts (e.g. 0.25) push
                               #     the IK target near the edge of the
                               #     workspace where ikpy returns FK
                               #     errors and the controller cannot
                               #     converge, causing 'pick_motion'
                               #     timeouts.
                               #     The closed-loop arm_2 descent in the
                               #     adapter handles the final drop from
                               #     this height to grasp position.

# ---------------------------------------------------------------------------
# Grasp geometry
# ---------------------------------------------------------------------------
#
# Where should gripper_grasping_frame end up when we close the fingers?
#
# We express the target as a single calibrated offset from the object's
# Z centre to where gripper_grasping_frame should sit:
#
#     gripper_grasping_frame_z = object_centre_z + _GRASP_FRAME_OFFSET_Z
#
# This offset is a calibration value that accounts for:
#   - the geometric relationship between gripper_grasping_frame and the
#     actual fingertip pinch point (depends on URDF convention),
#   - the typical droop of ikpy's IK solutions (where the real frame
#     lands vs the commanded IK target),
#   - any extra clearance wanted so fingers grip the upper body of the
#     object rather than grazing the table.
#
# All three effects are kinematically lumped together in this one number,
# which is tuned empirically: put the gripper at a position that grasps
# the object correctly, read the gripper_grasping_frame Z via TF, and
# set _GRASP_FRAME_OFFSET_Z = (measured_frame_z - object_centre_z).
#
# Measured on PAL Tiago + standard gripper + ~12 cm can: 0.170 m.
#
# --- IK tip vs gripper_grasping_frame offset -------------------------------
# ikpy builds its kinematic chain with an internal last_link_vector that
# extends 10 cm past arm_7_link along that link's local +Z axis.  Under
# _FRONT_GRASP_QUAT, that local +Z ends up along arm +X, so ikpy's
# "virtual tip" lands ~10 cm ahead of the real gripper_grasping_frame.
# _IK_TIP_X_BIAS compensates for this in the X direction.
_GRASP_FRAME_OFFSET_Z    = 0.100  # m - calibrated offset: object centre to
                                    # gripper_grasping_frame at correct grasp.
_PLACE_CLEARANCE         = 0.000  # m - extra clearance above bench surface.
_IK_TIP_X_BIAS           = 0.10   # m - compensate ikpy tip vs gripper_grasping_frame

# Common SDF/URDF extensions to probe when locating a model directory.
_SDF_MODEL_FILENAMES = ("model.sdf", "model.urdf", "model.xml")


def _parse_object_size_from_sdf(sdf_path: str) -> Optional[Tuple[float, float, float]]:
    """
    Parse an SDF file and return the (x, y, z) extent of the first non-
    ground collision/visual geometry found, or None if nothing matches.

    Supports the three most common primitives: box (<size>), cylinder
    (<radius>, <length>) and sphere (<radius>).  Mesh primitives are not
    supported because the .obj / .dae / .stl bounds would need loading.
    For those objects the caller should pass explicit dimensions.
    """
    try:
        tree = ET.parse(sdf_path)
    except (ET.ParseError, OSError):
        return None
    root = tree.getroot()

    # Prefer <collision> geometry because it is always authoritative for
    # physics; visuals sometimes use decorative meshes with no size tag.
    for geom_parent in ("collision", "visual"):
        for parent in root.iter(geom_parent):
            for geom in parent.iter("geometry"):
                # <box><size>x y z</size></box>
                box = geom.find("box")
                if box is not None:
                    size_node = box.find("size")
                    if size_node is not None and size_node.text:
                        try:
                            sx, sy, sz = (float(v) for v in size_node.text.split())
                            return (sx, sy, sz)
                        except ValueError:
                            pass
                # <cylinder><radius>r</radius><length>l</length></cylinder>
                cyl = geom.find("cylinder")
                if cyl is not None:
                    r_node = cyl.find("radius")
                    l_node = cyl.find("length")
                    if r_node is not None and l_node is not None:
                        try:
                            r = float(r_node.text); l = float(l_node.text)
                            return (2 * r, 2 * r, l)
                        except (ValueError, TypeError):
                            pass
                # <sphere><radius>r</radius></sphere>
                sph = geom.find("sphere")
                if sph is not None:
                    r_node = sph.find("radius")
                    if r_node is not None:
                        try:
                            r = float(r_node.text)
                            return (2 * r, 2 * r, 2 * r)
                        except (ValueError, TypeError):
                            pass
    return None


def _find_object_size(
    object_name: Optional[str],
    models_base_dir: Optional[str],
) -> Optional[Tuple[float, float, float]]:
    """
    Search models_base_dir for a subdirectory matching object_name and,
    if found, parse the first model.sdf/model.urdf inside it to extract
    its physical extents.
    """
    object_name = object_name.split("_")[0] if object_name else None

    if not object_name or not models_base_dir:
        return None
    import os
    candidate_dirs = [
        os.path.join(models_base_dir, object_name),
        os.path.join(models_base_dir, object_name.lower()),
    ]
    for cdir in candidate_dirs:
        if not os.path.isdir(cdir):
            continue
        for fname in _SDF_MODEL_FILENAMES:
            sdf_path = os.path.join(cdir, fname)
            if os.path.isfile(sdf_path):
                size = _parse_object_size_from_sdf(sdf_path)
                if size is not None:
                    return size
    return None


# Trajectory timing (seconds per waypoint).  Kept intentionally short so the
# simulation does not stall waiting for slow joint trajectories.
_WAYPOINT_DURATION_SEC = 2      # s — time to reach the FIRST waypoint
_DESCENT_EXTRA_SEC     = 2      # s added per additional waypoint (extend / retrace)
_TUCK_STAGE1_SEC       = 2
_TUCK_STAGE2_SEC       = 3
_TUCK_STAGE3_SEC       = 5

# ---------------------------------------------------------------------------
# Front / horizontal grasp orientation
# ---------------------------------------------------------------------------
#
# PAL Tiago gripper_grasping_frame convention
# -------------------------------------------
# The PAL gripper_grasping_frame's **Z-axis** is the approach direction.
# We want that Z-axis to point in the arm-base +X direction (toward the object).
#
# Rotation to achieve Z → +X:  +90° about the arm-base Y-axis.
#   R_y(+90°) applied to [0,0,1] → [1,0,0]  ✓
#   scipy [x, y, z, w] quaternion: [0, sin(45°), 0, cos(45°)]
#
# This is the IK target orientation.  Using the correct approach axis lets
# the solver converge to forward-facing wrist configurations instead of
# wrist-flipped (gripper-toward-robot) solutions.
_FRONT_GRASP_QUAT = (0.0, 0.7071, 0.0, 0.7071)   # 90° around Y → Z points toward +X (object)

# FK orientation validation threshold.
# After IK the X-axis (col 0 of the FK rotation matrix) must have a positive
# x-component exceeding this value (pointing forward, toward the object).
# Range [0, 1] — 0.55 ≈ within ~55° of straight forward.
_FRONT_APPROACH_X_THRESH: float = 0.55

# ---------------------------------------------------------------------------
# Pre-grasp staging pose constants
# ---------------------------------------------------------------------------
#
# Two-stage motion from tuck/home to the elbow-up, approach-ready pose.
# This matches the first frame of the reference pick sequence: arm raised
# above the table, elbow bent forward, wrist neutral.
#
# Stage 1 — LIFT (t = 3 s)
#   Raise the arm partway from the home position.  arm_5 / arm_6 begin
#   unwinding toward neutral.
#       arm_2 =  0.25  (arm partially raised)
#       arm_4 =  1.00  (elbow moderately bent for clearance)
#       arm_5 = -0.50  (wrist roll halfway to neutral)
#       arm_6 =  0.40  (wrist pitch halfway to neutral)
#
# Stage 2 — SETTLE (t = 6 s, matches _ARM_PRE_GRASP_JOINTS exactly)
#   Arm reaches the final elevated, wrist-neutral configuration ready for
#   the IK-planned diagonal descent.
#       arm_2 =  0.50  arm raised ~29° above horizontal — well above table ✓
#       arm_4 =  1.50  elbow bent, forearm points forward-up ✓
#       arm_5 =  0.00  forearm roll neutral
#       arm_6 =  0.00  wrist pitch neutral → gripper level
#
#   Joint limits (from PAL Tiago spec):
#     arm_1: [0,      +2.749] rad   arm_2: [-1.571, +1.091] rad
#     arm_3: [-3.534, +1.571] rad   arm_4: [-0.393, +2.356] rad
#     arm_5: [-2.094, +2.094] rad   arm_6: [-1.571, +1.571] rad
#
#   arm_2 = 0.50 ∈ [-1.571, +1.091] ✓
#   arm_4 = 1.50 ∈ [-0.393, +2.356] ✓
# Right-bend pre-grasp pose: arm rotated to neutral shoulder (arm_1=0),
# raised above table (arm_2=0.7), upper-arm rolled inward (arm_3=-1.5),
# elbow bent (arm_4=1.0), wrist set for side approach (arm_6=1.39, arm_7=1.8).
# From this pose the pick motion sweeps arm_1 positive toward the object.
_ARM_PRE_GRASP_JOINTS = [0.0, 0.7, -1.5, 1.0, 0.0, 1.39, 1.8]

# IK seeds — span the arm_2 range from elevated pre-grasp (arm_2 ≈ +0.50)
# down to the grasp position (arm_2 ≈ 0.00 to -0.20).
#
# Index 0 is the pre-grasp pose (added explicitly at runtime before this list
# is consulted); indices 1–5 are the generic seeds used in _solve_ik([1:]).
# Keeping only 5 generic seeds (vs the previous 10) cuts IK time roughly in half.
# Seeds reflect the right-bend approach: arm_1 sweeps from 0 up to ~1.8 rad
# (positive = toward object on the right), arm_2 adjusts height (0.3–0.7),
# arm_3 / arm_6 / arm_7 track the right-bend wrist configuration.
_FRONT_APPROACH_SEEDS: List[List[float]] = [
    # Index 0: right-bend pre-grasp pose (also added separately at runtime)
    [0.0,  0.7, -1.5,  1.0,  0.0,  1.39, 1.8],
    # Indices 1–5: arm_1 sweep 0 → ~1.8, arm_2 lowering to match table height
    [0.5,  0.6, -1.5,  1.0,  0.0,  1.39, 1.8],   # arm_1 partial sweep
    [1.0,  0.5, -1.5,  1.0,  0.0,  1.39, 1.8],   # arm_1 mid sweep
    [1.5,  0.4, -1.5,  1.0,  0.0,  1.39, 1.8],   # arm_1 near full sweep
    [1.0,  0.3, -1.5,  0.8,  0.0,  1.0,  1.5],   # lower height variant
    [1.5,  0.3, -1.5,  0.8,  0.0,  1.0,  1.5],   # lower height + full sweep
]

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
_ARM_TUCK_STAGE2  = [0.20,  0.80, -0.20, 2.35,  0.00,  0.00, 0.0]
_ARM_TUCK_STAGE3  = [0.20, -1.34, -0.20, 1.94, -1.57,  1.37, 0.0]

_GRIPPER_OPEN   = 0.044
_GRIPPER_CLOSED = 0.002
_ARM_TIP_LINK   = "gripper_grasping_frame"


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
    pre_approach_dist : float
        Horizontal distance (m) the arm is retracted behind the object at
        the start of the extension move.  Default 0.25.
    approach_dist : float
        Horizontal distance (m) of the intermediate hover waypoint — the
        last stop before the gripper reaches the object.  Default 0.10.
    arm_base_z : float
        Height of arm_1_link above the floor (m). Default 0.83.
    preferred_reach : float
        Preferred horizontal reach when computing nav poses (m). Default 0.70.
    """

    def __init__(
        self,
        robot_name:        str,
        robot_frame:       bool           = False,
        urdf_path:         Optional[str]  = None,
        pre_approach_dist: float          = _PRE_APPROACH_DIST,
        approach_dist:     float          = _APPROACH_DIST,
        arm_base_z:        float          = 0.83,
        preferred_reach:   float          = _PREFERRED_REACH,
        world_sdf_path:    Optional[str]  = None,
        models_base_dir:   Optional[str]  = None,
        table_standoff:    float          = 0.10,
    ) -> None:
        self._robot_name        = robot_name
        self._robot_frame       = robot_frame
        self._pre_approach_dist = pre_approach_dist
        self._approach_dist     = approach_dist
        self._arm_base_z        = arm_base_z
        # _gripper_z_offset: 0.0 when the IK chain reaches
        # gripper_grasping_frame, otherwise 0.143.  In practice ikpy
        # collapses fixed-joint segments past the last revolute joint
        # (arm_7), so this stays 0.0 by default and the ground-truth
        # alignment loop in the adapter closes the loop correctly.
        self._gripper_z_offset   = 0.0
        self._preferred_reach   = preferred_reach
        self._urdf_path         = urdf_path or self._find_urdf()
        self._models_base_dir   = models_base_dir

        # Per-plan offset from object centre to where gripper_grasping_frame
        # should end up.  Stable across plan() calls -- this is a fixed
        # kinematic / URDF calibration, not object-dependent.
        self._current_grasp_z_offset = _GRASP_FRAME_OFFSET_Z
        # Vertical extent of the object being picked (set in plan() from SDF).
        # Used by the adapter to compute the correct place-descent target
        # (gripper needs to descend until object bottom reaches place surface).
        self._current_object_sz      = 0.0

        # Expose the lift height so TiagoAdapter can stay in sync when
        # computing ascent targets without hardcoding the same value.
        self._pre_approach_z_lift = _PRE_APPROACH_Z_LIFT

        self._table_resolver: Optional[SdfTableResolver] = None
        if world_sdf_path:
            self._table_resolver = SdfTableResolver(
                sdf_path        = world_sdf_path,
                base_standoff   = table_standoff,
                models_base_dir = models_base_dir,
            )
            print(
                f"[TiagoPickPlacePlanner] Table resolver loaded "
                f"({len(self._table_resolver.known_tables())} models)."
            )

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
        pick_object_name:   Optional[str]                       = None,
        pick_object_size:   Optional[Tuple[float, float, float]] = None,
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
        current_arm_joints : dict, optional
            Current joint positions keyed by joint name.
        pick_object_name : str, optional
            Name of the pick object (directory name under models_base_dir).
            If provided AND models_base_dir was set on the planner, the SDF
            is parsed to extract the object's Z extent.  Used to compute a
            safe approach/grasp clearance so the gripper fingers hover just
            above the object top rather than at object centre.
        pick_object_size : (sx, sy, sz), optional
            Explicit object extent, overrides anything parsed from SDF.
            Use this for mesh-based models (where SDF has no <size> tag).

        Returns a PickPlanResult.
        """
        # ---- Determine grasp Z offset for this plan ----------------------
        # Target position for gripper_grasping_frame:
        #     object_centre_z + _GRASP_FRAME_OFFSET_Z
        #
        # This is a single calibrated offset that accounts for the URDF
        # convention, ikpy's FK-vs-real-TF offset at the typical grasp
        # pose, and the desired grip height on the object.  See the block
        # comment at the top of this module for calibration details.
        self._current_grasp_z_offset = _GRASP_FRAME_OFFSET_Z
        self._current_object_sz      = 0.0   # overwritten below if size known

        size = pick_object_size
        if size is None:
            size = _find_object_size(pick_object_name, self._models_base_dir)
        if size is not None:
            sz = max(0.0, float(size[2]))
            self._current_object_sz = sz
            print(
                f"[TiagoPickPlacePlanner] Grasp target: object "
                f"'{pick_object_name or '?'}' sz={sz:.3f} m.  "
                f"gripper_grasping_frame offset = "
                f"{self._current_grasp_z_offset:.3f} m above object centre."
            )
        else:
            if pick_object_name:
                print(
                    f"[TiagoPickPlacePlanner] Grasp target: no size found "
                    f"for '{pick_object_name}'.  "
                    f"gripper_grasping_frame offset = "
                    f"{self._current_grasp_z_offset:.3f} m above object centre."
                )

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
            pose_at_pick = robot_pose
            print(
                "[TiagoPickPlacePlanner] robot_frame=True — "
                "navigation suppressed for pick. Caller must be in position."
            )
        else:
            pick_nav = self._compute_nav_pose(pick_xyz, robot_pose)
            pose_at_pick = pick_nav if pick_nav is not None else robot_pose
            if pick_nav is not None:
                pre_wp = self._compute_pre_approach_waypoint(
                    (pick_xyz[0], pick_xyz[1]), (robot_pose[0], robot_pose[1]), pick_nav
                )
                if pre_wp is not None:
                    steps.append(PickPlaceStep(
                        kind     = StepKind.NAVIGATE,
                        label    = "navigate_pre_approach_pick",
                        nav_goal = pre_wp,
                    ))
                    print(
                        f"[TiagoPickPlacePlanner] Pre-approach waypoint (pick): "
                        f"({pre_wp[0]:.2f}, {pre_wp[1]:.2f}, "
                        f"{math.degrees(pre_wp[2]):.1f}°)"
                    )
                steps.append(PickPlaceStep(
                    kind          = StepKind.NAVIGATE,
                    label         = "navigate_to_pick",
                    nav_goal      = pick_nav,
                    nav_target_xy = (pick_xyz[0], pick_xyz[1]),
                ))
                print(
                    f"[TiagoPickPlacePlanner] Navigate to pick: "
                    f"({pick_nav[0]:.2f}, {pick_nav[1]:.2f}, "
                    f"{math.degrees(pick_nav[2]):.1f}°)"
                )
            else:
                print("[TiagoPickPlacePlanner] Pick reachable from current pose.")

        # ── Open gripper early (while arm is still tucked) ───────────
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_pre_pick",
            gripper_width = _GRIPPER_OPEN,
        ))

        # ── Pre-grasp staging (pick) — 2-stage arm motion ─────────────
        # Stage 1: raise arm partway (arm_2 → 0.25, arm_4 → 1.00).
        # Stage 2: settle to elbow-up pose (arm_2 → 0.50, arm_4 → 1.50),
        #          wrist neutral (arm_5 = arm_6 = 0).
        # The arm ends at _ARM_PRE_GRASP_JOINTS — the IK seed for WP 0
        # of the pick_motion trajectory (elevated by PRE_APPROACH_Z_LIFT).
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "pre_grasp_pick",
            joint_trajectory = self._make_pre_grasp_trajectory(),
        ))
        print("[TiagoPickPlacePlanner] Pre-grasp staging step (2 stages) added for pick.")

        # ── Diagonal approach trajectory (3 waypoints) ───────────────
        # WP 0 — pre_approach: PRE_APPROACH_DIST behind object AND
        #         PRE_APPROACH_Z_LIFT above object z.  The arm descends
        #         diagonally from the elevated staging pose (arm_2 ≈ +0.50).
        #         Starting above the table plane eliminates table-edge
        #         collisions during the forward extension.
        # WP 1 — approach:     APPROACH_DIST behind object AT object z.
        # WP 2 — grasp:        object centre.
        # Solved bottom-up (WP 2 first) so all waypoints share the same
        # elbow configuration.
        #
        # All three waypoints are at the ELEVATED Z (pick_xyz[2] + Z_LIFT).
        # The descent to the actual grasp height is NOT done here as a
        # Cartesian joint-space interpolation -- that was prone to making
        # the forearm/wrist droop into the table when the IK branch chose
        # to compensate for the height change by bending other joints.
        # Instead, the adapter performs a closed-loop GT-based descent
        # (descend_to_grasp step, below) by commanding arm_2 in small
        # increments while monitoring gripper_grasping_frame via TF.
        _pick_pre_xy = self._offset_approach(pick_xyz, pose_at_pick, self._pre_approach_dist)
        pick_pre_approach = (_pick_pre_xy[0], _pick_pre_xy[1],
                             _pick_pre_xy[2] + _PRE_APPROACH_Z_LIFT)
        # Keep the approach and grasp waypoints at the SAME elevated Z as
        # pick_pre_approach.  Only the XY changes between waypoints.
        _pick_approach_xy = self._offset_approach(pick_xyz, pose_at_pick, self._approach_dist)
        pick_approach = (_pick_approach_xy[0], _pick_approach_xy[1],
                         _pick_approach_xy[2] + _PRE_APPROACH_Z_LIFT)
        # pick_xyz_high: XY over the object, Z still elevated.  The real
        # descent happens later in descend_to_grasp (adapter-side).
        pick_xyz_high = (pick_xyz[0], pick_xyz[1],
                         pick_xyz[2] + _PRE_APPROACH_Z_LIFT)
        step = self._make_arm_trajectory(
            "pick_motion",
            [pick_pre_approach, pick_approach, pick_xyz_high],
            pose_at_pick,
        )
        if step is None:
            return PickPlanResult.failure("IK failed for 'pick_motion'")
        steps.append(step)

        # ── Closed-loop Cartesian descent to grasp height ───────────
        # Sentinel step: no joint trajectory, just a label.  The adapter
        # detects the label and runs a GT-feedback descent via arm_2
        # (see _descend_to_grasp in TiagoAdapter).  The target Z is
        # derived by the adapter from its own copy of pick_xyz.
        steps.append(PickPlaceStep(
            kind  = StepKind.MOVE,
            label = "descend_to_grasp",
        ))

        # ── Close gripper ────────────────────────────────────────────
        steps.append(PickPlaceStep(
            kind          = StepKind.CLOSE_GRIPPER,
            label         = "close_gripper_grasp",
            gripper_width = _GRIPPER_CLOSED,
        ))

        # ── Tuck arm to carry position ───────────────────────────────
        # Go directly from the grasp pose to the tuck (home) position.
        # This replaces the old ascend_after_grasp + retrace_pick
        # sequence, which required arm_2 headroom that was often
        # unavailable after grasping.  The tuck trajectory is defined
        # in joint-space stages and works from any arm configuration.
        # The object is held safely in the tucked position during
        # navigation to the place pose.
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "tuck_after_grasp",
            joint_trajectory = self._make_tuck_trajectory(),
        ))

        # ── Navigation to place ──────────────────────────────────────
        if self._robot_frame:
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
                pre_wp = self._compute_pre_approach_waypoint(
                    (place_xyz[0], place_xyz[1]), (pose_at_pick[0], pose_at_pick[1]), place_nav
                )
                if pre_wp is not None:
                    steps.append(PickPlaceStep(
                        kind     = StepKind.NAVIGATE,
                        label    = "navigate_pre_approach_place",
                        nav_goal = pre_wp,
                    ))
                    print(
                        f"[TiagoPickPlacePlanner] Pre-approach waypoint (place): "
                        f"({pre_wp[0]:.2f}, {pre_wp[1]:.2f}, "
                        f"{math.degrees(pre_wp[2]):.1f}°)"
                    )
                steps.append(PickPlaceStep(
                    kind          = StepKind.NAVIGATE,
                    label         = "navigate_to_place",
                    nav_goal      = place_nav,
                    nav_target_xy = (place_xyz[0], place_xyz[1]),
                ))
                print(
                    f"[TiagoPickPlacePlanner] Navigate to place: "
                    f"({place_nav[0]:.2f}, {place_nav[1]:.2f}, "
                    f"{math.degrees(place_nav[2]):.1f}°)"
                )
            else:
                print("[TiagoPickPlacePlanner] Place reachable from pick pose.")

        # ── Pre-grasp staging (place) — same 2-stage motion as pick ──
        steps.append(PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = "pre_grasp_place",
            joint_trajectory = self._make_pre_grasp_trajectory(),
        ))
        print("[TiagoPickPlacePlanner] Pre-grasp staging step (2 stages) added for place.")

        # ── Place motion ─────────────────────────────────────────────
        # Waypoint Z values are computed explicitly so _make_arm_trajectory
        # can use grasp_z_override=0.0 (no auto-adding of grasp offset).
        #
        # Approach waypoints: surface_z + Z_LIFT + grasp_offset gives the
        # IK target enough height that the arm swings above the bench
        # surface during the horizontal approach (same logic as pick).
        #
        # Final waypoint (place target):
        #     surface_z + sz/2             → object bottom on surface
        #     + _GRASP_FRAME_OFFSET_Z      → frame above object centre
        #     + _PLACE_CLEARANCE           → gentle landing clearance
        #     + _GRASP_FRAME_OFFSET_Z      → compensate IK droop at the
        #                                    place arm config (~15-20 cm
        #                                    observed; same constant used
        #                                    as pick since it was tuned
        #                                    for this arm's droop)
        # The double _GRASP_FRAME_OFFSET_Z is intentional: the first
        # instance is the hardware geometry (frame above pinch point),
        # the second compensates for the IK branch droop so the real
        # gripper lands where the second instance accounts for.
        sz = self._current_object_sz
        _approach_z  = place_xyz[2] + _PRE_APPROACH_Z_LIFT + _GRASP_FRAME_OFFSET_Z
        _place_target_z = (place_xyz[2]
                           + sz / 2.0
                           + _GRASP_FRAME_OFFSET_Z
                           + _PLACE_CLEARANCE)

        _place_pre_xy      = self._offset_approach(place_xyz, pose_at_place, self._pre_approach_dist)
        place_pre_approach = (_place_pre_xy[0], _place_pre_xy[1], _approach_z)
        _place_approach_xy = self._offset_approach(place_xyz, pose_at_place, self._approach_dist)
        place_approach     = (_place_approach_xy[0], _place_approach_xy[1], _approach_z)
        place_xyz_target   = (place_xyz[0], place_xyz[1], _place_target_z)
        print(
            f"[TiagoPickPlacePlanner] Place target Z: surface {place_xyz[2]:.3f}"
            f" + sz/2 {sz/2:.3f} + offset {_GRASP_FRAME_OFFSET_Z:.3f}"
            f" + clearance {_PLACE_CLEARANCE:.3f} = {_place_target_z:.3f}"
            f"  (approach Z={_approach_z:.3f})"
        )
        step = self._make_arm_trajectory(
            "place_motion",
            [place_pre_approach, place_approach, place_xyz_target],
            pose_at_place,
            grasp_z_override=0.0,
        )
        if step is None:
            return PickPlanResult.failure("IK failed for 'place_motion'")
        steps.append(step)

        # ── Open gripper (release) ───────────────────────────────────
        # place_motion already brought the gripper to the correct Z.
        steps.append(PickPlaceStep(
            kind          = StepKind.OPEN_GRIPPER,
            label         = "open_gripper_release",
            gripper_width = _GRIPPER_OPEN,
        ))

        # ── Tuck arm after place ─────────────────────────────────────
        # Same rationale as tuck_after_grasp: go directly to the carry/
        # home position rather than attempting an ascent that may exceed
        # arm_2's joint range.
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
    # Post-navigation replan
    # ------------------------------------------------------------------

    def replan_step(self, step: PickPlaceStep, robot_pose: NavPose) -> bool:
        """
        Recompute the joint trajectory for a single MOVE step using the
        robot's actual pose after navigation.

        Returns True if the trajectory was updated, False if IK failed
        (original trajectory is kept as a fallback).
        """
        if step.kind != StepKind.MOVE or step.target_pose is None:
            return True
        if step.label in {"tuck_arm_initial", "tuck_arm_pre_place_nav", "return_home",
                          "pre_grasp_pick", "pre_grasp_place"}:
            return True   # joint-space moves; world position is irrelevant

        xyz_last = (
            step.target_pose.position.x,
            step.target_pose.position.y,
            step.target_pose.position.z,
        )

        # ── 3-waypoint pick / place motion steps ─────────────────────
        # target_pose = grasp/place position (last WP, at object z).
        # WP 0 = pre_approach: behind object AND Z_LIFT above object z.
        # WP 1 = approach: behind object AT object z.
        # WP 2 = grasp: object centre.
        if step.label in {"pick_motion", "place_motion"}:
            xyz_grasp = xyz_last
            xyz_hover = self._offset_approach(xyz_grasp, robot_pose, self._approach_dist)
            _pre_xy   = self._offset_approach(xyz_grasp, robot_pose, self._pre_approach_dist)
            xyz_pre   = (_pre_xy[0], _pre_xy[1], _pre_xy[2] + _PRE_APPROACH_Z_LIFT)
            # place_motion waypoints already include the full place-Z geometry
            # so _current_grasp_z_offset must NOT be added again.
            z_override = 0.0 if step.label == "place_motion" else None
            new_step = self._make_arm_trajectory(
                step.label,
                [xyz_pre, xyz_hover, xyz_grasp],
                robot_pose,
                grasp_z_override=z_override,
            )
            if new_step is None:
                print(f"[TiagoPickPlacePlanner] replan_step: replan failed for "
                      f"'{step.label}' — keeping original.")
                return False
            step.joint_trajectory = new_step.joint_trajectory
            print(f"[TiagoPickPlacePlanner] replan_step: '{step.label}' replanned (3 WPs).")
            return True

        # ── 2-waypoint retrace steps ──────────────────────────────────
        # target_pose = pre_approach position (last WP, elevated by Z_LIFT).
        # Reconstruct approach (hover) z = pre_approach z - Z_LIFT (= object z).
        # Reconstruct approach x/y by moving toward object by (pre_dist - approach_dist).
        if step.label in {"retrace_pick", "retrace_place"}:
            xyz_pre = xyz_last  # elevated: z = object_z + Z_LIFT
            delta   = self._pre_approach_dist - self._approach_dist
            hover_z = xyz_pre[2] - _PRE_APPROACH_Z_LIFT   # object z
            if self._robot_frame:
                xyz_hover = (xyz_pre[0] + delta, xyz_pre[1], hover_z)
            else:
                theta = robot_pose[2]
                xyz_hover = (
                    xyz_pre[0] + delta * math.cos(theta),
                    xyz_pre[1] + delta * math.sin(theta),
                    hover_z,
                )
            new_step = self._make_arm_trajectory(
                step.label,
                [xyz_hover, xyz_pre],
                robot_pose,
            )
            if new_step is None:
                print(f"[TiagoPickPlacePlanner] replan_step: replan failed for "
                      f"'{step.label}' — keeping original.")
                return False
            step.joint_trajectory = new_step.joint_trajectory
            print(f"[TiagoPickPlacePlanner] replan_step: '{step.label}' replanned (2 WPs).")
            return True

        return True

    # ------------------------------------------------------------------
    # Reachability
    # ------------------------------------------------------------------

    def is_reachable(self, target_xyz: XYZ, robot_pose: NavPose) -> bool:
        """Return True if target_xyz is within the arm's reach envelope."""
        x_arm, y_arm, z_arm = self._to_arm_frame(target_xyz, robot_pose)
        horiz    = math.sqrt(x_arm ** 2 + y_arm ** 2)
        in_horiz = _REACH_MIN_HORIZ <= horiz <= _REACH_MAX_HORIZ
        in_z     = _Z_ARM_MIN <= z_arm <= _Z_ARM_MAX
        in_front = x_arm > 0
        return in_horiz and in_z and in_front

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def _offset_approach(self, xyz: XYZ, robot_pose: NavPose, dist: float) -> XYZ:
        """
        Return xyz shifted backward along the robot→object heading by dist.

        In robot_frame mode the heading is always +x, so the offset is simply
        (-dist, 0, 0) relative to xyz.  In world_frame mode robot_pose.theta
        is used to compute the direction vector.
        """
        if self._robot_frame:
            return (xyz[0] - dist, xyz[1], xyz[2])
        theta = robot_pose[2]
        return (
            xyz[0] - dist * math.cos(theta),
            xyz[1] - dist * math.sin(theta),
            xyz[2],
        )

    def _to_arm_frame(self, xyz: XYZ, robot_pose: NavPose) -> XYZ:
        if self._robot_frame:
            return (xyz[0], xyz[1], xyz[2] - self._arm_base_z)
        return self._world_to_arm(xyz, robot_pose)

    def _world_to_arm(self, xyz_world: XYZ, robot_pose: NavPose) -> XYZ:
        nav_x, nav_y, nav_theta = robot_pose
        dx = xyz_world[0] - nav_x
        dy = xyz_world[1] - nav_y
        cos_t =  math.cos(nav_theta)
        sin_t =  math.sin(nav_theta)
        x_arm =  dx * cos_t + dy * sin_t
        y_arm = -dx * sin_t + dy * cos_t
        z_arm =  xyz_world[2] - self._arm_base_z
        return (x_arm, y_arm, z_arm)

    def _arm_to_world(self, xyz_arm: XYZ, robot_pose: NavPose) -> XYZ:
        nav_x, nav_y, nav_theta = robot_pose
        cos_t = math.cos(nav_theta)
        sin_t = math.sin(nav_theta)
        world_x = nav_x + xyz_arm[0] * cos_t - xyz_arm[1] * sin_t
        world_y = nav_y + xyz_arm[0] * sin_t + xyz_arm[1] * cos_t
        world_z = xyz_arm[2] + self._arm_base_z
        return (world_x, world_y, world_z)

    # ------------------------------------------------------------------
    # Nav pose computation  (world_frame mode only)
    # ------------------------------------------------------------------

    def _compute_nav_pose(
        self,
        target_xyz: XYZ,
        robot_pose: NavPose,
    ) -> Optional[NavPose]:
        """
        Return the best nav pose from which target_xyz is reachable, or None
        if already reachable from the current pose.
        """
        if self.is_reachable(target_xyz, robot_pose):
            return None

        tx, ty, _   = target_xyz
        rx, ry, rth = robot_pose

        # ── Table-aware primary candidate ─────────────────────────────
        if self._table_resolver is not None:
            table_name = self._table_resolver.find_table_for_object((tx, ty))
            if table_name is not None:
                standoff_pose = self._table_resolver.approach_pose_for_object(
                    object_xy  = (tx, ty),
                    robot_xy   = (rx, ry),
                    table_name = table_name,
                )
                if standoff_pose is not None:
                    if self.is_reachable(target_xyz, standoff_pose):
                        print(
                            f"[TiagoPickPlacePlanner] Table-aware nav pose for "
                            f"'{table_name}': "
                            f"({standoff_pose[0]:.2f}, {standoff_pose[1]:.2f}, "
                            f"{math.degrees(standoff_pose[2]):.1f}°)"
                        )
                        return standoff_pose
                    else:
                        print(
                            f"[TiagoPickPlacePlanner] Table-aware pose for "
                            f"'{table_name}' not reachable — using angle-sweep fallback."
                        )

        # ── Angle-sweep fallback ──────────────────────────────────────
        base_angle = math.atan2(ry - ty, rx - tx)
        best: Optional[NavPose] = None
        best_cost = float("inf")

        for i in range(_N_APPROACH_ANGLES):
            angle     = base_angle + i * (2.0 * math.pi / _N_APPROACH_ANGLES)
            nav_x     = tx + self._preferred_reach * math.cos(angle)
            nav_y     = ty + self._preferred_reach * math.sin(angle)
            nav_theta = math.atan2(ty - nav_y, tx - nav_x)
            candidate = (nav_x, nav_y, nav_theta)

            if not self.is_reachable(target_xyz, candidate):
                continue

            d             = math.hypot(nav_x - rx, nav_y - ry)
            drive_heading = math.atan2(nav_y - ry, nav_x - rx) if d > 1e-3 else nav_theta
            final_align   = abs(math.atan2(
                math.sin(nav_theta - drive_heading),
                math.cos(nav_theta - drive_heading),
            ))
            cost = 10.0 * final_align + d

            if cost < best_cost:
                best_cost = cost
                best      = candidate

        if best is not None:
            return best

        # Geometric fallback — no reachability check
        print("[TiagoPickPlacePlanner] WARNING: no reachable nav pose — geometric fallback.")
        nav_x     = tx + self._preferred_reach * math.cos(base_angle)
        nav_y     = ty + self._preferred_reach * math.sin(base_angle)
        nav_theta = math.atan2(ty - nav_y, tx - nav_x)
        return (nav_x, nav_y, nav_theta)

    # ------------------------------------------------------------------
    # Pre-approach waypoint  (world_frame mode only)
    # ------------------------------------------------------------------

    def _compute_pre_approach_waypoint(
        self,
        object_xy:  Tuple[float, float],
        from_xy:    Tuple[float, float],
        final_pose: NavPose,
    ) -> Optional[NavPose]:
        """
        Return a pre-approach waypoint if the straight-line path clips a
        known table, otherwise None.
        """
        if self._table_resolver is None:
            return None

        all_tables = self._table_resolver.known_tables()
        fp_x, fp_y, _ = final_pose

        path_blocked = False
        for name in all_tables:
            fp = self._table_resolver.get_footprint(name)
            if fp is None:
                continue
            if fp.segment_intersects(from_xy, (fp_x, fp_y), margin=0.30):
                path_blocked = True
                break

        if not path_blocked:
            return None

        table_name = self._table_resolver.find_table_for_object(object_xy)
        if table_name is None:
            return None

        waypoints = self._table_resolver.approach_waypoints(
            object_xy  = object_xy,
            robot_xy   = from_xy,
            table_name = table_name,
        )
        if len(waypoints) >= 2:
            return waypoints[0]
        return None

    # ------------------------------------------------------------------
    # Step construction
    # ------------------------------------------------------------------

    def _make_arm_trajectory(
        self,
        label:          str,
        xyz_world_list: List[XYZ],   # waypoints in execution order
        robot_pose:     NavPose,
        grasp_z_override: Optional[float] = None,
    ) -> Optional[PickPlaceStep]:
        """
        Build a single MOVE step whose JointTrajectory passes through every
        point in *xyz_world_list* in order.

        grasp_z_override
            If provided, replaces self._current_grasp_z_offset in the IK
            target Z formula.  Pass 0.0 when the waypoint Z values already
            include the full place geometry (so the offset is not
            double-counted).

        Bottom-up seeded IK
        -------------------
        Waypoints are solved in reverse execution order (last = the grasp /
        deepest reach).  Each solution seeds the next shallower waypoint so
        the entire trajectory stays in the same elbow branch — no flips, no
        unexpected joint-space shortcuts during execution.

        Timing: waypoint i (0-based, execution order) is assigned
            t_i = WAYPOINT_DURATION_SEC + i * DESCENT_EXTRA_SEC  seconds.

        target_pose is set to the last waypoint so replan_step() can
        reconstruct intermediate positions from it.
        """
        world_pose = _make_pose(xyz_world_list[-1], _FRONT_GRASP_QUAT)

        solved: List[Optional[List[float]]] = [None] * len(xyz_world_list)
        prev_joints: Optional[List[float]] = None

        for i in range(len(xyz_world_list) - 1, -1, -1):
            xyz_world = xyz_world_list[i]
            xyz_arm   = self._to_arm_frame(xyz_world, robot_pose)

            # IK target in arm frame.  Two static biases are baked in:
            #
            #   +X: _IK_TIP_X_BIAS compensates for ikpy's internal
            #       last_link_vector = [0,0,0.1] extension past arm_7_link.
            #       Under _FRONT_GRASP_QUAT the last link's +Z points
            #       forward (along arm +X), so ikpy's virtual tip ends up
            #       ~10 cm ahead of the real gripper_grasping_frame.
            #       Adding 10 cm to IK target X makes the real grasp frame
            #       land on the intended point.  Empirically consistent to
            #       within 2 mm across runs.
            #
            #   +Z: _current_grasp_z_offset = _GRASP_FRAME_OFFSET_Z.
            #       Places gripper_grasping_frame at the calibrated
            #       distance above object centre where the fingers grip
            #       the object body correctly.
            z_offset = self._current_grasp_z_offset if grasp_z_override is None else grasp_z_override
            xyz_ik = (
                xyz_arm[0] + _IK_TIP_X_BIAS,
                xyz_arm[1],
                xyz_arm[2]
                + self._arm_base_z
                + self._gripper_z_offset
                + z_offset,
            )

            seed   = prev_joints if prev_joints is not None else _ARM_PRE_GRASP_JOINTS
            joints = self._solve_ik(xyz_ik, initial_joints=seed)
            if joints is None:
                print(
                    f"[TiagoPickPlacePlanner] _make_arm_trajectory: IK failed "
                    f"at waypoint {i} of '{label}' ({_fmt(xyz_arm)})."
                )
                return None
            solved[i]   = joints
            prev_joints = joints

        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS

        for i, joints in enumerate(solved):
            pt = JointTrajectoryPoint()
            pt.positions       = joints
            pt.time_from_start = Duration(
                sec     = _WAYPOINT_DURATION_SEC + i * _DESCENT_EXTRA_SEC,
                nanosec = 0,
            )
            traj.points.append(pt)

        return PickPlaceStep(
            kind             = StepKind.MOVE,
            label            = label,
            target_pose      = world_pose,
            joint_trajectory = traj,
        )

    def _solve_ik(
        self,
        xyz_arm:        XYZ,
        initial_joints: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        """
        Solve IK for target position xyz_arm (in the arm base frame, i.e. the
        coordinate frame whose origin is at arm_1_link).

        Strategy
        --------
        1. initial_joints seed (if provided) — almost always correct for WPs 0/1
           since the chain is seeded bottom-up from the grasp solution.
        2. _ARM_PRE_GRASP_JOINTS explicitly — physically expected arm state at
           the start of the approach (elbow-up: arm_2 ≈ +0.50).
        3. Up to 5 generic front-approach seeds covering arm_2 from +0.40 to -0.15
           and one arm_1 lateral variant.
        4. Orientation modes: "all" first (full 3-axis, most likely to honour the
           Z-forward approach quaternion), then None (position-only last resort,
           accepted only when the FK orientation check still passes).
           "Y"-only mode is intentionally skipped — it ignores two axes and often
           converges to wrist-flipped solutions that face the gripper backwards.
        5. max_iter=200 per IK call (vs ikpy default of 1000) — the seeded solver
           converges in <100 iterations when the seed is close; capping at 200
           eliminates long tail iterations that gain nothing.
        6. FK acceptance threshold 0.15 m.
        7. FK orientation check via _check_front_approach() — rejects solutions
           where the gripper Z-axis does not point toward the object (+X arm frame).
        8. Early exit when a forward-facing solution with FK error < 0.06 m found.
        """
        if not IKPY_AVAILABLE or self._chain is None:
            print(f"[TiagoPickPlacePlanner] ikpy unavailable — home joints for {_fmt(xyz_arm)}.")
            return _ARM_HOME_JOINTS[:]

        target_matrix = _xyz_to_matrix(xyz_arm, _FRONT_GRASP_QUAT)
        n = len(self._chain.links)
        revolute_idx = [
            i for i, lnk in enumerate(self._chain.links)
            if getattr(lnk, "joint_type", "fixed") == "revolute"
        ]

        def _make_seed(joints7: List[float]) -> List[float]:
            s = [0.0] * n
            for k, idx in enumerate(revolute_idx):
                if k < len(joints7):
                    s[idx] = joints7[k]
            return s

        # Seed order: caller's seed (bottom-up chain) → pre-grasp pose → 5 generics
        seeds: List[List[float]] = []
        if initial_joints is not None:
            seeds.append(_make_seed(initial_joints))
        if initial_joints != _ARM_PRE_GRASP_JOINTS:
            seeds.append(_make_seed(_ARM_PRE_GRASP_JOINTS))
        for j in _FRONT_APPROACH_SEEDS[1:6]:   # 5 generic descent seeds
            seeds.append(_make_seed(j))

        import warnings

        best_joints: Optional[List[float]] = None
        best_error   = float("inf")
        best_is_fwd  = False

        # Two orientation modes:
        #   "all" — full 3-axis constraint; correctly enforces Z-forward orientation.
        #           Skipping "Y" (single-axis) avoids wrist-flipped solutions that
        #           pass the partial constraint but face the gripper toward the robot.
        #   None  — position-only last resort; accepted only when FK check passes.
        for orientation_mode in ("all", None):
            if orientation_mode is None and best_is_fwd:
                break

            for seed in seeds:
                try:
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore")
                        ik_result = self._chain.inverse_kinematics_frame(
                            target           = target_matrix,
                            initial_position = seed,
                            orientation_mode = orientation_mode,
                            max_iter         = 200,
                        )
                except Exception:
                    continue

                joints = [ik_result[i] for i in revolute_idx]
                if len(joints) != 7:
                    continue

                fk    = self._chain.forward_kinematics(ik_result)
                error = np.linalg.norm(fk[:3, 3] - np.array(xyz_arm))
                if error > 0.15:
                    continue

                is_fwd = self._check_front_approach(fk)

                if (is_fwd and not best_is_fwd) or (is_fwd == best_is_fwd and error < best_error):
                    best_error  = error
                    best_joints = joints
                    best_is_fwd = is_fwd

                if best_is_fwd and best_error < 0.06:
                    print(
                        f"[TiagoPickPlacePlanner] IK {_fmt(xyz_arm)}: "
                        f"FK error {best_error*100:.1f} cm, front-approach ✓ (fast exit)."
                    )
                    return best_joints

        if best_joints is None:
            print(f"[TiagoPickPlacePlanner] IK failed for {_fmt(xyz_arm)}.")
            return None

        if not best_is_fwd:
            print(
                f"[TiagoPickPlacePlanner] WARNING: IK solution for {_fmt(xyz_arm)} "
                f"did not pass front-approach orientation check "
                f"(FK error {best_error*100:.1f} cm). "
                "Arm may not approach horizontally — verify in simulation."
            )
        else:
            print(
                f"[TiagoPickPlacePlanner] IK {_fmt(xyz_arm)}: "
                f"FK error {best_error*100:.1f} cm, front-approach ✓."
            )
        return best_joints

    def _check_front_approach(self, fk_matrix: np.ndarray) -> bool:
        """
        Return True if the FK result represents a horizontal front approach.

        The PAL Tiago gripper_grasping_frame uses its **Z-axis** as the approach
        direction.  After IK with _FRONT_GRASP_QUAT (90° around Y), the FK
        rotation column 2 (Z-axis) must have a positive x-component in the arm
        base frame — pointing forward toward the object — exceeding the threshold.

        Column 0 (X-axis) is checked as a fallback for URDF variants where X is
        the approach axis.
        """
        try:
            R = fk_matrix[:3, :3]
            # Primary: Z-axis forward (PAL Tiago convention with 90°-Y quaternion)
            if R[0, 2] > _FRONT_APPROACH_X_THRESH:
                return True
            # Fallback: X-axis forward (alternative URDF conventions)
            if R[0, 0] > _FRONT_APPROACH_X_THRESH:
                return True
        except Exception:
            pass
        return False

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

    def _make_pre_grasp_trajectory(self) -> JointTrajectory:
        """
        Two-stage pre-grasp motion from the tuck/home pose to the right-bend
        approach-ready configuration.

        Stage 1 — INTERMEDIATE (t = 3 s)
            Lift arm_2 from home (-1.34) toward raised position (0.0),
            begin rolling arm_3 inward (-0.8), bend elbow moderately (arm_4=1.2),
            start winding wrist toward target values.
                arm_1 =  0.0   shoulder neutral (same as target)
                arm_2 =  0.0   arm at horizontal — clear of table
                arm_3 = -0.8   upper-arm roll partway to -1.5
                arm_4 =  1.2   elbow moderately bent
                arm_5 =  0.0   forearm roll neutral
                arm_6 =  0.8   wrist pitch partway to 1.39
                arm_7 =  0.9   wrist roll partway to 1.8

        Stage 2 — RIGHT-BEND READY (t = 6 s) — matches _ARM_PRE_GRASP_JOINTS
            Arm settles to the right-bend configuration from which the IK-planned
            pick sweep begins.  arm_1 stays at 0; the pick motion increases arm_1
            to sweep toward the object.
                arm_1 =  0.0   shoulder neutral
                arm_2 =  0.7   arm raised ~40° above horizontal ✓
                arm_3 = -1.5   upper-arm rolled inward ✓
                arm_4 =  1.0   elbow bent ✓
                arm_5 =  0.0   forearm roll neutral
                arm_6 =  1.39  wrist pitch for side approach ✓
                arm_7 =  1.8   wrist roll for side approach ✓
        """
        stage1 = [0.0,  0.0, -0.8,  1.2,  0.0,  0.80, 0.9]   # intermediate lift
        stage2 = _ARM_PRE_GRASP_JOINTS[:]                       # right-bend ready

        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS

        def _pt(positions: List[float], sec: int) -> JointTrajectoryPoint:
            p = JointTrajectoryPoint()
            p.positions       = positions
            p.time_from_start = Duration(sec=sec, nanosec=0)
            return p

        traj.points = [
            _pt(stage1, 3),   # lift to intermediate (3 s)
            _pt(stage2, 6),   # settle to right-bend ready (6 s total)
        ]
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
        chain = self._build_chain(arm_urdf)
        if chain is not None and arm_urdf:
            import xml.etree.ElementTree as _ET
            try:
                root = _ET.parse(arm_urdf).getroot()
                tip_links = {e.get("name") for e in root.findall("link")}
                if "gripper_grasping_frame" in tip_links:
                    self._gripper_z_offset = 0.0
                else:
                    self._gripper_z_offset = 0.143
                    print(
                        f"[TiagoPickPlacePlanner] gripper_grasping_frame not in chain; "
                        f"applying gripper_z_offset={self._gripper_z_offset:.3f} m."
                    )
            except Exception:
                self._gripper_z_offset = 0.0
        return chain

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
        tip_link  = next(
            (t for t in [_ARM_TIP_LINK, "arm_tool_link", "arm_7_link"]
             if t in all_links),
            None,
        )
        if tip_link is None:
            print(f"[TiagoPickPlacePlanner] Tip link not found. Available: {sorted(all_links)}")
            return None
        print(f"[TiagoPickPlacePlanner] IK tip link: '{tip_link}'.")

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
