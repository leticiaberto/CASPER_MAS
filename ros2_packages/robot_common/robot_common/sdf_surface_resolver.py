"""
SdfSurfaceResolver
==================
Parses a Gazebo world SDF file once at startup and computes, for every model,
the z offset from the model's world origin to its top surface.

    offset = max over all links of (link_pose_z + geometry_half_height)

This offset is added to the world_pose.z returned by ObjectToRobot to get
the world z coordinate of the model's top surface — the correct z for placing
objects on top of furniture or stacking items.

Usage
-----
    resolver = SdfSurfaceResolver("/path/to/world.sdf")

    # For a table whose OTR world_pose.z = 0.80:
    offset = resolver.top_surface_offset("prep")   # → 0.05
    place_z = otr_world_z + offset                 # → 0.85

    # For a tomato (spherical object, origin at centre):
    offset = resolver.top_surface_offset("tomato_1")  # → 0.025
    # place_z = top of tomato, e.g. for stacking

    # Or use the convenience wrapper:
    place_xyz = resolver.place_xyz(otr_result, "bowl_1")
    # Returns (x, y, top_surface_z)

Supported geometry types
------------------------
    box          → half_z = size_z / 2
    cylinder     → half_z = length / 2
    sphere       → half_z = radius
    plane        → half_z = 0  (infinite ground plane)
    mesh         → half_z = 0  (unknown, conservative fallback)

Composite models (multiple links/geometries)
--------------------------------------------
The top surface is the maximum over all links and all collision/visual
geometries within each link:

    offset = max(link_pose_z + geometry_half_z)

for every (link, geometry) pair in the model.

Included models (<include> tags)
---------------------------------
<include> blocks reference external model URIs (e.g. model://tomato) whose
SDF files are NOT embedded in the world SDF.  For these, the resolver tries
to locate the model SDF in the following search paths (in order):

    1. GAZEBO_MODEL_PATH environment variable (colon-separated directories)
    2. ~/.gazebo/models
    3. /usr/share/gazebo-*/models
    4. /ros2_ws/src/*/models

If the external SDF is found, it is parsed with the same logic.
If not found, the fallback offset (default 0.0) is used.

Fallback
--------
If a model is not found in the SDF or its geometry cannot be determined,
top_surface_offset() returns `fallback` (default 0.0) so that behaviour
is conservative (place at model origin z) rather than crashing.
"""

from __future__ import annotations

import glob
import os
import xml.etree.ElementTree as ET
from typing import Dict, Optional, Tuple


# ---------------------------------------------------------------------------
# SdfSurfaceResolver
# ---------------------------------------------------------------------------

class SdfSurfaceResolver:
    """
    Parameters
    ----------
    sdf_path : str
        Absolute path to the world .sdf file.
    fallback : float
        Offset returned for unknown models. Default 0.0.
    verbose : bool
        Print per-model offset summary on load. Default False.
    """

    def __init__(
        self,
        sdf_path: str,
        fallback: float = 0.0,
        verbose: bool   = False,
    ) -> None:
        self._fallback = fallback
        self._verbose  = verbose
        # { model_name: z_offset_to_top_surface }
        self._offsets: Dict[str, float] = {}

        if not os.path.exists(sdf_path):
            print(f"[SdfSurfaceResolver] WARNING: SDF not found at '{sdf_path}'. "
                  "All offsets will use fallback.")
            return

        self._model_search_paths = self._build_search_paths()
        self._parse_world(sdf_path)

        if verbose:
            print(f"[SdfSurfaceResolver] Loaded {len(self._offsets)} models "
                  f"from '{sdf_path}':")
            for name, off in sorted(self._offsets.items()):
                print(f"    {name:<30} top_offset = {off:.4f} m")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def top_surface_offset(self, model_name: str) -> float:
        """
        Return the z offset from model world origin to its top surface (m).

        Falls back to `self._fallback` (default 0.0) if unknown.
        """
        return self._offsets.get(model_name, self._fallback)

    def top_surface_world_z(self, model_name: str, otr_world_z: float) -> float:
        """
        Return the world z coordinate of the model's top surface.

        Parameters
        ----------
        model_name : str
            Gazebo model name.
        otr_world_z : float
            world_pose.position.z from ObjectToRobot.get_pose().
        """
        return otr_world_z + self.top_surface_offset(model_name)

    def place_xyz(
        self,
        otr_result: dict,
        model_name: str,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Convenience: extract (x, y, top_surface_z) from an OTR result dict.

        Parameters
        ----------
        otr_result : dict
            Return value of ObjectToRobot.get_pose(model_name).
        model_name : str
            The model name (used to look up the surface offset).

        Returns None if the OTR result contains an error.
        """
        if "error" in otr_result:
            return None
        pos = otr_result["world_pose"].position
        return (
            pos.x,
            pos.y,
            pos.z + self.top_surface_offset(model_name),
        )

    def known_models(self) -> list:
        """Return the list of model names for which offsets were computed."""
        return list(self._offsets.keys())

    # ------------------------------------------------------------------
    # SDF parsing
    # ------------------------------------------------------------------

    def _parse_world(self, sdf_path: str) -> None:
        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
        except ET.ParseError as exc:
            print(f"[SdfSurfaceResolver] XML parse error in '{sdf_path}': {exc}")
            return

        world = root.find("world") or root  # handle <sdf><world> and bare <world>

        # ── Inline <model> elements ───────────────────────────────────
        for model_el in world.findall("model"):
            name = model_el.get("name", "")
            if not name:
                continue
            offset = self._model_top_offset(model_el)
            self._offsets[name] = offset

        # ── <include> elements  (external model URIs) ─────────────────
        for include_el in world.findall("include"):
            # Gazebo SDF spec uses <name>, but some world files use the
            # shorthand <n> tag — check both.
            # ET elements are falsy when childless — must use 'is not None',
            # never bare 'or', to check existence.
            name_el = include_el.find("name")
            if name_el is None:
                name_el = include_el.find("n")
            uri_el  = include_el.find("uri")
            if name_el is None or uri_el is None:
                continue
            name = name_el.text.strip() if name_el.text else ""
            uri  = uri_el.text.strip()  if uri_el.text  else ""
            if not name or not uri:
                continue

            # Include pose adjusts the model origin in world frame.
            # The offset is RELATIVE to the model origin, so the include
            # pose doesn't affect the top-surface offset calculation.
            ext_offset = self._resolve_include_offset(uri)
            self._offsets[name] = ext_offset

    def _model_top_offset(self, model_el: ET.Element) -> float:
        """
        Compute top_surface_offset for an inline <model> element.
        = max over all links of (link_pose_z + geometry_half_z)
        """
        max_top = 0.0
        for link_el in model_el.findall("link"):
            link_z = _parse_pose_z(link_el.find("pose"))
            # Check both collision and visual geometry
            for tag in ("collision", "visual"):
                for shape_el in link_el.findall(tag):
                    geom_el = shape_el.find("geometry")
                    if geom_el is None:
                        continue
                    child_z = _parse_pose_z(shape_el.find("pose"))
                    half    = _geometry_half_z(geom_el)
                    top     = link_z + child_z + half
                    if top > max_top:
                        max_top = top
        return max_top

    def _resolve_include_offset(self, uri: str) -> float:
        """
        Find the external model SDF for a URI like 'model://tomato'
        and compute its top_surface_offset.
        """
        model_dir = self._find_model_dir(uri)
        if model_dir is None:
            return self._fallback

        # Look for model.sdf or model-1_4.sdf
        for candidate in ("model.sdf", "model-1_4.sdf", "model-1_5.sdf"):
            sdf_file = os.path.join(model_dir, candidate)
            if os.path.exists(sdf_file):
                return self._parse_model_sdf(sdf_file)

        return self._fallback

    def _parse_model_sdf(self, sdf_path: str) -> float:
        """Parse a standalone model SDF and return its top_surface_offset."""
        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
        except ET.ParseError:
            return self._fallback

        # Handle <sdf><model> and bare <model>
        model_el = root.find("model") or (root if root.tag == "model" else None)
        if model_el is None:
            return self._fallback

        return self._model_top_offset(model_el)

    def _find_model_dir(self, uri: str) -> Optional[str]:
        """
        Resolve a 'model://name' URI to a filesystem directory.
        Searches GAZEBO_MODEL_PATH, ~/.gazebo/models, and common system paths.
        """
        # Strip 'model://' prefix
        model_name = uri.replace("model://", "").strip("/")

        for search_dir in self._model_search_paths:
            candidate = os.path.join(search_dir, model_name)
            if os.path.isdir(candidate):
                return candidate

        return None

    @staticmethod
    def _build_search_paths() -> list:
        paths = []

        # GAZEBO_MODEL_PATH env var (colon-separated)
        env_paths = os.environ.get("GAZEBO_MODEL_PATH", "")
        for p in env_paths.split(":"):
            p = p.strip()
            if p and os.path.isdir(p):
                paths.append(p)

        # ~/.gazebo/models
        home_models = os.path.expanduser("~/.gazebo/models")
        if os.path.isdir(home_models):
            paths.append(home_models)

        # System Gazebo models
        for pattern in (
            "/usr/share/gazebo-*/models",
            "/usr/share/ignition/*/models",
            "/opt/ros/*/share/*/models",
        ):
            for p in glob.glob(pattern):
                if os.path.isdir(p):
                    paths.append(p)

        # ROS2 workspace models
        for pattern in (
            "/ros2_ws/src/*/models",
            "/ros2_ws/install/*/share/*/models",
        ):
            for p in glob.glob(pattern):
                if os.path.isdir(p):
                    paths.append(p)

        return paths


# ---------------------------------------------------------------------------
# SDF geometry helpers
# ---------------------------------------------------------------------------

def _parse_pose_z(pose_el: Optional[ET.Element]) -> float:
    """Extract z from a <pose>x y z r p y</pose> element. Returns 0.0 if absent."""
    if pose_el is None or not pose_el.text:
        return 0.0
    parts = pose_el.text.strip().split()
    if len(parts) >= 3:
        try:
            return float(parts[2])
        except ValueError:
            pass
    return 0.0


def _geometry_half_z(geom_el: ET.Element) -> float:
    """
    Return the half-extent in the Z direction for a <geometry> element.

    <box>      → size_z / 2
    <cylinder> → length / 2
    <sphere>   → radius
    <plane>    → 0  (ground plane, no height)
    <mesh>     → 0  (unknown, conservative)
    """
    # Box
    box = geom_el.find("box")
    if box is not None:
        size_el = box.find("size")
        if size_el is not None and size_el.text:
            parts = size_el.text.strip().split()
            if len(parts) >= 3:
                try:
                    return float(parts[2]) / 2.0
                except ValueError:
                    pass

    # Cylinder
    cyl = geom_el.find("cylinder")
    if cyl is not None:
        length_el = cyl.find("length")
        if length_el is not None and length_el.text:
            try:
                return float(length_el.text.strip()) / 2.0
            except ValueError:
                pass

    # Sphere
    sph = geom_el.find("sphere")
    if sph is not None:
        radius_el = sph.find("radius")
        if radius_el is not None and radius_el.text:
            try:
                return float(radius_el.text.strip())
            except ValueError:
                pass

    # Plane, mesh, or unknown → conservative zero
    return 0.0