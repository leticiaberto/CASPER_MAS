import colorsys
import hashlib
from enum import Enum
import os
from typing import Optional

# -----------------------------
# Palette classes
# -----------------------------
# These are deliberately chosen to not clash with:
# lightcoral (red), palegoldenrod (yellow), lightskyblue (blue),
# palegreen (green), lightgray, snow (white)
AGENT_COLORS = [
    "#c8a8e9",  # pastel purple
    "#f4a9d0",  # pastel pink
    "#a8d8d8",  # pastel teal
    "#f4c9a0",  # pastel orange
    "#b8c9f0",  # pastel periwinkle  
    "#e8d5a8",  # pastel tan/khaki
    "#d0e8a8",  # pastel lime (lighter/yellower than palegreen)
    "#f0b8b8",  # pastel salmon (lighter/more pink than lightcoral)
    "#a8c8e8",  # pastel steel blue (more muted than lightskyblue)
    "#e8b8d8",  # pastel mauve
    "#b8e8c8",  # pastel mint
    "#e8c8a8",  # pastel peach
]

class PastelPalette:
    """
    Assigns a fixed pastel color to each agent, in registration order.
    Colors are chosen to be visually distinct from task status colors.
    Falls back to cycling if more agents than palette entries.
    """

    def __init__(self, colors: list[str] = AGENT_COLORS):
        self.colors = colors
        self.map: dict[str, str] = {}
        self._counter = 0

    def get_color(self, agent_name: str | None) -> str:
        if agent_name is None:
            return "white"
        if agent_name not in self.map:
            self.map[agent_name] = self.colors[self._counter % len(self.colors)]
            self._counter += 1
        return self.map[agent_name]
    
class PastelPaletteHash:
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