"""
SdfTableResolver
================
Parses a Gazebo world SDF (and external model SDFs) at startup and builds,
for every *static* model, an axis-aligned bounding footprint in the XY plane.

This is used by TiagoPickPlacePlanner and TiagoNavigator to compute
**table-aware approach poses**: instead of placing the robot at a fixed
`preferred_reach` distance from the *object*, the planner now stops at

    table_edge  +  base_standoff

where `table_edge` is the nearest edge of the table's XY footprint in the
direction of the robot, and `base_standoff` is a tunable clearance that
keeps the robot base from colliding with the table legs/apron while still
allowing the arm to reach the object.

Key concepts
------------
TableFootprint
    An axis-aligned rectangle in world XY derived from the model's collision
    geometry (box or cylinder).  For composite models the union of all link
    footprints is used.

  centre_x, centre_y  — world XY of the model origin
  half_x, half_y      — half-extents in X and Y (m)
  yaw                 — model yaw (used to rotate the box before taking AABB)

approach_pose_for_object(object_xy, robot_xy, table_name, ...)
    Given the object and the robot's current XY, returns the (x, y, theta)
    nav pose that places the robot at `base_standoff` beyond the nearest
    table edge, facing the object.

    If the straight-line path from the robot to that approach pose clips
    *another* table, a pre-approach waypoint is inserted so the robot first
    moves to the aisle before turning towards the table.

Usage
-----
    resolver = SdfTableResolver("/path/to/world.sdf")

    # Simple pose lookup (used by TiagoPickPlacePlanner):
    nav_pose = resolver.approach_pose_for_object(
        object_xy  = (1.0, 5.3),
        robot_xy   = (0.0, 0.0),
        table_name = "prep",
    )

    # With pre-approach waypoint (used by TiagoNavigator.drive_to_table):
    waypoints = resolver.approach_waypoints(
        object_xy  = (1.0, 5.3),
        robot_xy   = (0.0, 0.0),
        table_name = "prep",
    )
    # Returns list of 1 or 2 (x, y, theta) tuples.
    # Length 2 means: drive to waypoints[0] first, then waypoints[1].

Supported geometry
------------------
    box      → half_x = size_x/2,  half_y = size_y/2
    cylinder → half_x = half_y = radius
    sphere   → half_x = half_y = radius
    plane    → skipped (infinite ground plane)
    mesh     → skipped (unknown shape, conservative)

Model search paths
------------------
Same order as SdfSurfaceResolver:
    1. GAZEBO_MODEL_PATH environment variable
    2. ~/.gazebo/models
    3. /usr/share/gazebo-*/models
    4. /ros2_ws/src/*/models
"""

from __future__ import annotations

import glob
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XY      = Tuple[float, float]
NavPose = Tuple[float, float, float]   # (x, y, theta)


# ---------------------------------------------------------------------------
# TableFootprint
# ---------------------------------------------------------------------------

@dataclass
class TableFootprint:
    """
    Axis-aligned bounding footprint of a model in world XY.

    All values are in the world frame (model pose already applied).
    """
    name:     str
    centre_x: float
    centre_y: float
    half_x:   float    # half-extent along world X
    half_y:   float    # half-extent along world Y

    # Convenience ─────────────────────────────────────────────────────

    def nearest_face_normal(self, from_xy: XY) -> XY:
        """
        Return the axis-aligned outward unit normal of the table face nearest
        to `from_xy`.  Always one of (1,0), (-1,0), (0,1), (0,-1).

        Using the face normal (not the nearest boundary point) ensures the
        robot always approaches perpendicular to a flat side — never at a
        corner angle.  A perpendicular approach means the arm sweeps parallel
        to the table edge during the upward/downward pick arc, so it never
        clips a corner.
        """
        cx, cy = self.centre_x, self.centre_y
        hx, hy = self.half_x,   self.half_y
        fx, fy = from_xy
        # Four (face_midpoint, outward_normal) pairs
        candidates = [
            ((cx,      cy + hy), ( 0.0,  1.0)),   # +Y face
            ((cx,      cy - hy), ( 0.0, -1.0)),   # -Y face
            ((cx + hx, cy     ), ( 1.0,  0.0)),   # +X face
            ((cx - hx, cy     ), (-1.0,  0.0)),   # -X face
        ]
        # Pick the face whose midpoint is closest to the robot
        best = min(candidates, key=lambda c: math.hypot(c[0][0] - fx, c[0][1] - fy))
        return best[1]   # outward normal

    def standoff_pose(
        self,
        from_xy:    XY,
        object_xy:  XY,
        standoff:   float,
        reach_pref: float = 0.55,
    ) -> NavPose:
        """
        Return (x, y, theta) placing the robot exactly ``reach_pref`` metres
        from the object along the outward normal of the nearest table face.

        This is the correct approach: the nav goal is measured from the OBJECT,
        not from the table face.  The table face is used only to determine the
        approach direction (perpendicular to the face).

        nav_goal = object_xy + outward_normal * reach_pref

        This guarantees arm_horiz = reach_pref at the planned nav goal, and
        arm_horiz ≤ reach_pref + nav_xy_tolerance at the actual stopped position.
        With reach_pref=0.55 and XY_TOL=0.15: worst-case reach = 0.70 m ✓

        Parameters
        ----------
        from_xy : (x, y)
            Current robot XY — used only to determine which face is nearest.
        object_xy : (x, y)
            World-frame XY of the object to pick/place.
        standoff : float
            Unused distance parameter (kept for API compatibility).
            The robot may end up closer to the face than this value for deep
            objects; this is safe since the prep table has no side walls.
        reach_pref : float
            Desired arm horizontal reach at the nav goal (default 0.55 m).
        """
        normal = self.nearest_face_normal(from_xy)
        nx, ny = normal

        # Nav goal: step BACK from the object by reach_pref along the face normal.
        # outward normal points AWAY from the table, so stepping in that direction
        # moves the robot away from the object.
        sp_x = object_xy[0] + nx * reach_pref
        sp_y = object_xy[1] + ny * reach_pref

        # Lateral alignment: the nav goal X (or Y for X-facing face) is the
        # object's lateral coordinate so the robot is directly in front.
        # This is already satisfied because we step from object_xy directly.

        # Theta: from nav goal toward object (inward = -normal direction)
        theta = math.atan2(object_xy[1] - sp_y, object_xy[0] - sp_x)
        return (sp_x, sp_y, theta)

    def segment_intersects(self, p1: XY, p2: XY, margin: float = 0.0) -> bool:
        """
        True if the line segment p1→p2 passes through (or within `margin`
        of) this AABB.  Used for pre-approach waypoint detection.
        """
        # Expand AABB by margin
        min_x = self.centre_x - self.half_x - margin
        max_x = self.centre_x + self.half_x + margin
        min_y = self.centre_y - self.half_y - margin
        max_y = self.centre_y + self.half_y + margin

        # Cohen-Sutherland clip: parametric segment vs AABB
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        t_min, t_max = 0.0, 1.0

        for (p_val, q_val) in [
            (-dx, x1 - min_x),
            ( dx, max_x - x1),
            (-dy, y1 - min_y),
            ( dy, max_y - y1),
        ]:
            if p_val == 0.0:
                if q_val < 0.0:
                    return False
            elif p_val < 0.0:
                t_min = max(t_min, q_val / p_val)
            else:
                t_max = min(t_max, q_val / p_val)
        return t_min <= t_max

    def _contains(self, x: float, y: float) -> bool:
        return (
            self.centre_x - self.half_x <= x <= self.centre_x + self.half_x and
            self.centre_y - self.half_y <= y <= self.centre_y + self.half_y
        )


# ---------------------------------------------------------------------------
# SdfTableResolver
# ---------------------------------------------------------------------------

# Default standoff: robot base stops this far from the nearest table edge.
# At 0.50 m the Tiago arm (reach ~0.75 m) comfortably reaches objects at
# the centre of a 1.1 m-wide table without the base colliding with legs.
_DEFAULT_STANDOFF = 0.10   # m — min clearance from table face (no side walls; robot fits beside slab)

# Robot footprint half-width used for segment-intersection margin checks
# (conservative for Tiago base ~0.27 m radius).
_ROBOT_HALF_WIDTH = 0.30   # m


class SdfTableResolver:
    """
    Parameters
    ----------
    sdf_path : str
        Absolute path to the Gazebo world .sdf file.
    base_standoff : float
        Metres the robot base stops short of the table edge. Default 0.50.
    models_base_dir : str, optional
        Root directory that contains ``<model_name>/model.sdf`` files
        (e.g. ``/ros2_ws/src/my_pkg/simulation/models``).
        Prepended to the model search path so that your custom models are
        found before system Gazebo models.
    verbose : bool
        Print a summary of parsed footprints on load.
    """

    def __init__(
        self,
        sdf_path:       str,
        base_standoff:  float          = _DEFAULT_STANDOFF,
        models_base_dir: Optional[str] = None,
        verbose:        bool           = False,
    ) -> None:
        self._standoff  = base_standoff
        self._verbose   = verbose
        # { model_name: TableFootprint }
        self._footprints: Dict[str, TableFootprint] = {}

        self._search_paths = self._build_search_paths(models_base_dir)

        if not os.path.exists(sdf_path):
            print(
                f"[SdfTableResolver] WARNING: SDF not found at '{sdf_path}'. "
                "Approach poses will fall back to preferred_reach only."
            )
            return

        self._parse_world(sdf_path)

        if verbose:
            print(
                f"[SdfTableResolver] Loaded {len(self._footprints)} "
                f"model footprints from '{sdf_path}':"
            )
            for name, fp in sorted(self._footprints.items()):
                print(
                    f"    {name:<25}  centre=({fp.centre_x:.2f}, {fp.centre_y:.2f})"
                    f"  half=({fp.half_x:.2f}, {fp.half_y:.2f})"
                )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_footprint(self, model_name: str) -> Optional[TableFootprint]:
        """Return the TableFootprint for `model_name`, or None if unknown."""
        return self._footprints.get(model_name)

    def known_tables(self) -> List[str]:
        """Return list of model names for which footprints were parsed."""
        return list(self._footprints.keys())

    def approach_pose_for_object(
        self,
        object_xy:  XY,
        robot_xy:   XY,
        table_name: str,
        standoff:   Optional[float] = None,
    ) -> Optional[NavPose]:
        """
        Return a single (x, y, theta) nav pose that stops the robot at
        `standoff` metres beyond the nearest table edge, facing the object.

        Returns None if the table name is unknown (caller falls back to
        the old preferred_reach approach).
        """
        fp = self._footprints.get(table_name)
        if fp is None:
            return None
        s = standoff if standoff is not None else self._standoff
        return fp.standoff_pose(robot_xy, object_xy, s)

    def approach_waypoints(
        self,
        object_xy:   XY,
        robot_xy:    XY,
        table_name:  str,
        standoff:    Optional[float] = None,
        all_tables:  Optional[List[str]] = None,
    ) -> List[NavPose]:
        """
        Return an ordered list of 1 or 2 nav poses.

        If the straight line from `robot_xy` to the approach pose clips
        another table (or this table's body), a **pre-approach waypoint**
        is prepended:

            [pre_approach_waypoint, final_approach_pose]

        Otherwise returns:

            [final_approach_pose]

        Parameters
        ----------
        object_xy : (x, y)
            World-frame XY of the target object.
        robot_xy : (x, y)
            Current world-frame XY of the robot base.
        table_name : str
            Gazebo model name of the table holding the object.
        standoff : float, optional
            Override the default standoff distance.
        all_tables : list of str, optional
            Names of ALL tables to check for path clipping.
            Defaults to all known tables.
        """
        fp = self._footprints.get(table_name)
        if fp is None:
            return []   # unknown table → caller uses fallback

        s = standoff if standoff is not None else self._standoff
        final_pose = fp.standoff_pose(robot_xy, object_xy, s)

        # Tables to check for path clipping
        check_names = all_tables if all_tables is not None else list(self._footprints.keys())

        if self._path_is_clear(robot_xy, (final_pose[0], final_pose[1]), check_names):
            return [final_pose]

        # Path is blocked — compute a pre-approach waypoint.
        # The waypoint is placed in the "aisle" beside the table:
        # same approach direction but shifted laterally to clear all
        # obstacles between the robot and the approach pose.
        pre_wp = self._compute_pre_approach(robot_xy, final_pose, fp, check_names, s)
        if pre_wp is not None:
            return [pre_wp, final_pose]

        # Fallback: return just the final pose even if path is unclear.
        # The P-controller will still get there; worst case it bumps a leg.
        return [final_pose]

    def find_table_for_object(
        self,
        object_xy: XY,
        tolerance: float = 0.30,
    ) -> Optional[str]:
        """
        Return the name of the table whose footprint (expanded by `tolerance`)
        contains `object_xy`, or None if no table matches.

        Useful when the caller knows the object XY but not which table it's on.
        """
        for name, fp in self._footprints.items():
            if (
                fp.centre_x - fp.half_x - tolerance <= object_xy[0] <= fp.centre_x + fp.half_x + tolerance and
                fp.centre_y - fp.half_y - tolerance <= object_xy[1] <= fp.centre_y + fp.half_y + tolerance
            ):
                return name
        return None

    # ------------------------------------------------------------------
    # Path clearance helpers
    # ------------------------------------------------------------------

    def _path_is_clear(
        self,
        p1:          XY,
        p2:          XY,
        table_names: List[str],
    ) -> bool:
        """True if the segment p1→p2 does not clip any table AABB (+ robot margin)."""
        for name in table_names:
            fp = self._footprints.get(name)
            if fp is None:
                continue
            if fp.segment_intersects(p1, p2, margin=_ROBOT_HALF_WIDTH):
                return False
        return True

    def _compute_pre_approach(
        self,
        robot_xy:    XY,
        final_pose:  NavPose,
        target_fp:   TableFootprint,
        check_names: List[str],
        standoff:    float,
    ) -> Optional[NavPose]:
        """
        Find a pre-approach waypoint in the aisle that is:
          - reachable from robot_xy without clipping any table
          - from which the final_pose is directly reachable without clipping

        Strategy: sweep candidate positions along the aisle parallel to the
        table face at the same standoff distance, and pick the closest one to
        the robot that is feasible.  The sweep axis is perpendicular to the
        face normal (i.e. along the face itself).
        """
        ap_x, ap_y, ap_theta = final_pose

        # Lateral sweep axis = direction perpendicular to the outward normal
        # (i.e. parallel to the table face the robot is approaching from).
        normal = target_fp.nearest_face_normal(robot_xy)
        nx, ny = normal
        # Perpendicular to normal: rotate 90°
        perp_x = -ny
        perp_y =  nx

        best_wp: Optional[NavPose] = None
        best_d = float("inf")

        for lateral_offset in [i * 0.4 for i in range(-8, 9)]:
            cand_x = ap_x + perp_x * lateral_offset
            cand_y = ap_y + perp_y * lateral_offset
            cand_xy = (cand_x, cand_y)

            # Must be reachable from robot AND allow reaching the final pose
            if not self._path_is_clear(robot_xy, cand_xy, check_names):
                continue
            if not self._path_is_clear(cand_xy, (ap_x, ap_y), check_names):
                continue

            d = math.hypot(cand_x - robot_xy[0], cand_y - robot_xy[1])
            if d < best_d:
                best_d = d
                # Face toward the final approach pose
                theta = math.atan2(ap_y - cand_y, ap_x - cand_x)
                best_wp = (cand_x, cand_y, theta)

        return best_wp

    # ------------------------------------------------------------------
    # SDF parsing
    # ------------------------------------------------------------------

    def _parse_world(self, sdf_path: str) -> None:
        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
        except ET.ParseError as exc:
            print(f"[SdfTableResolver] XML parse error in '{sdf_path}': {exc}")
            return

        world = root.find("world") or root

        # ── Inline <model> elements ────────────────────────────────────
        for model_el in world.findall("model"):
            name = model_el.get("name", "")
            if not name:
                continue
            fp = self._footprint_from_model_el(name, model_el, world_pose_z=None)
            if fp is not None:
                self._footprints[name] = fp

        # ── <include> elements  (external model URIs) ─────────────────
        for include_el in world.findall("include"):
            name_el = include_el.find("name")
            uri_el  = include_el.find("uri")
            if name_el is None or uri_el is None:
                continue
            name = (name_el.text or "").strip()
            uri  = (uri_el.text  or "").strip()
            if not name or not uri:
                continue
            pose_el = include_el.find("pose")
            wx, wy  = _parse_pose_xy(pose_el)
            fp = self._footprint_from_uri(name, uri, wx, wy)
            if fp is not None:
                self._footprints[name] = fp

    def _footprint_from_model_el(
        self,
        name:         str,
        model_el:     ET.Element,
        world_pose_z: Optional[float],
    ) -> Optional[TableFootprint]:
        """Extract footprint for an inline <model> element."""
        pose_el = model_el.find("pose")
        wx, wy  = _parse_pose_xy(pose_el)

        max_hx = 0.0
        max_hy = 0.0

        for link_el in model_el.findall("link"):
            link_px, link_py = _parse_pose_xy(link_el.find("pose"))
            for tag in ("collision", "visual"):
                for shape_el in link_el.findall(tag):
                    geom_el = shape_el.find("geometry")
                    if geom_el is None:
                        continue
                    geom_px, geom_py = _parse_pose_xy(shape_el.find("pose"))
                    hx, hy = _geometry_half_xy(geom_el)
                    # Local offset of this geometry within the model
                    local_cx = link_px + geom_px
                    local_cy = link_py + geom_py
                    # AABB of this geometry in model-local frame
                    ext_x = abs(local_cx) + hx
                    ext_y = abs(local_cy) + hy
                    max_hx = max(max_hx, ext_x)
                    max_hy = max(max_hy, ext_y)

        if max_hx < 1e-3 and max_hy < 1e-3:
            return None   # no collision geometry found (visual-only model)

        return TableFootprint(
            name     = name,
            centre_x = wx,
            centre_y = wy,
            half_x   = max_hx,
            half_y   = max_hy,
        )

    def _footprint_from_uri(
        self,
        name: str,
        uri:  str,
        wx:   float,
        wy:   float,
    ) -> Optional[TableFootprint]:
        """Resolve an external model URI and extract its footprint."""
        model_dir = self._find_model_dir(uri)
        if model_dir is None:
            return None
        for candidate in ("model.sdf", "model-1_4.sdf", "model-1_5.sdf"):
            sdf_file = os.path.join(model_dir, candidate)
            if not os.path.exists(sdf_file):
                continue
            try:
                tree  = ET.parse(sdf_file)
                root  = tree.getroot()
                model_el = root.find("model") or (root if root.tag == "model" else None)
                if model_el is None:
                    continue
                fp = self._footprint_from_model_el(name, model_el, world_pose_z=None)
                if fp is not None:
                    # Shift to world origin
                    fp.centre_x += wx
                    fp.centre_y += wy
                    return fp
            except ET.ParseError:
                continue
        return None

    def _find_model_dir(self, uri: str) -> Optional[str]:
        model_name = uri.replace("model://", "").strip("/")
        for search_dir in self._search_paths:
            candidate = os.path.join(search_dir, model_name)
            if os.path.isdir(candidate):
                return candidate
        return None

    @staticmethod
    def _build_search_paths(models_base_dir: Optional[str]) -> List[str]:
        paths: List[str] = []

        # Caller-supplied directory first (highest priority)
        if models_base_dir and os.path.isdir(models_base_dir):
            paths.append(models_base_dir)

        # GAZEBO_MODEL_PATH
        for p in os.environ.get("GAZEBO_MODEL_PATH", "").split(":"):
            p = p.strip()
            if p and os.path.isdir(p):
                paths.append(p)

        # ~/.gazebo/models
        home_models = os.path.expanduser("~/.gazebo/models")
        if os.path.isdir(home_models):
            paths.append(home_models)

        # System paths
        for pattern in (
            "/usr/share/gazebo-*/models",
            "/usr/share/ignition/*/models",
            "/opt/ros/*/share/*/models",
        ):
            for p in glob.glob(pattern):
                if os.path.isdir(p):
                    paths.append(p)

        # ROS2 workspace models — covers the package convention
        # simulation/models/<model_name>/model.sdf
        for pattern in (
            "/ros2_ws/src/*/models",
            "/ros2_ws/src/*/simulation/models",
            "/ros2_ws/install/*/share/*/models",
        ):
            for p in glob.glob(pattern):
                if os.path.isdir(p):
                    paths.append(p)

        return paths


# ---------------------------------------------------------------------------
# SDF geometry helpers
# ---------------------------------------------------------------------------

def _parse_pose_xy(pose_el: Optional[ET.Element]) -> Tuple[float, float]:
    """Extract (x, y) from a <pose>x y z r p y</pose> element."""
    if pose_el is None or not pose_el.text:
        return (0.0, 0.0)
    parts = pose_el.text.strip().split()
    if len(parts) >= 2:
        try:
            return (float(parts[0]), float(parts[1]))
        except ValueError:
            pass
    return (0.0, 0.0)


def _geometry_half_xy(geom_el: ET.Element) -> Tuple[float, float]:
    """
    Return (half_x, half_y) of a <geometry> element.

    box      → (size_x/2, size_y/2)
    cylinder → (radius, radius)
    sphere   → (radius, radius)
    plane    → (0, 0)
    mesh     → (0, 0)
    """
    # Box
    box = geom_el.find("box")
    if box is not None:
        size_el = box.find("size")
        if size_el is not None and size_el.text:
            parts = size_el.text.strip().split()
            if len(parts) >= 2:
                try:
                    return (float(parts[0]) / 2.0, float(parts[1]) / 2.0)
                except ValueError:
                    pass

    # Cylinder
    cyl = geom_el.find("cylinder")
    if cyl is not None:
        radius_el = cyl.find("radius")
        if radius_el is not None and radius_el.text:
            try:
                r = float(radius_el.text.strip())
                return (r, r)
            except ValueError:
                pass

    # Sphere
    sph = geom_el.find("sphere")
    if sph is not None:
        radius_el = sph.find("radius")
        if radius_el is not None and radius_el.text:
            try:
                r = float(radius_el.text.strip())
                return (r, r)
            except ValueError:
                pass

    return (0.0, 0.0)
