import colorsys
import hashlib
from enum import Enum
import os
from typing import Optional

# -----------------------------
# Palette classes
# -----------------------------
class PastelPalette:
    """
    Deterministic pastel palette with better hue separation
    """

    def __init__(self, saturation=0.55, value=0.95):
        self.sat = saturation
        self.val = value
        self.map = {}

    def get_color(self, label):
        if label is None:
            return "white"
        if label in self.map:
            return self.map[label]

        # Stable hash → convert to 0..1
        h_bytes = hashlib.md5(label.encode("utf-8")).digest()
        hue = int.from_bytes(h_bytes[:2], "little") / 65535.0  # 0..1

        r, g, b = colorsys.hsv_to_rgb(hue, self.sat, self.val)
        color_hex = "#%02x%02x%02x" % (int(r*255), int(g*255), int(b*255))
        self.map[label] = color_hex
        return color_hex
    
class DistinctPalette:
    """Evenly spaced colors for deterministic distinct palette"""
    def __init__(self, labels, saturation=0.8, value=0.95):
        self.labels = sorted(set(labels))  # deterministic order
        self.map = {}
        n = len(self.labels)
        for i, label in enumerate(self.labels):
            hue = i / max(n, 1)
            r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
            color_hex = "#%02x%02x%02x" % (int(r*255), int(g*255), int(b*255))
            self.map[label] = color_hex

    def get_color(self, label):
        return self.map.get(label, "white")
    
class Roles(Enum):
    SUPERVISOR = "supervisor"
    MEMBER = "member"


class ROSUtils:
    def _get_sdf_path(world_name) -> str:
        if world_name is None:
            world_name = "backyard"   # default world name for GT subscription
        candidates = [
            f"/ros2_ws/install/simulation/share/simulation/worlds/{world_name}.sdf",
            os.path.join(os.path.dirname(__file__), "worlds", f"{world_name}.sdf"),
        ]
        for p in candidates:
            if os.path.exists(p):
                return p
        raise FileNotFoundError(f"{world_name}.sdf not found. Searched:\n" + "\n".join(candidates))


    def _get_models_dir() -> Optional[str]:
        candidates = [
            "/ros2_ws/install/simulation/share/simulation/models",
            "/ros2_ws/src/simulation/models",
            os.path.join(os.path.dirname(__file__), "..", "simulation", "models"),
        ]
        for p in candidates:
            if os.path.isdir(p):
                return p
        return None