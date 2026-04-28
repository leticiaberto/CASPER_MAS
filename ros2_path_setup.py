"""
Adds all local ROS2 adapter packages to sys.path so they can be imported
without being installed or sourced. Import this module once at the entry
point (Robot.py, teste.py, etc.) before any ros2_packages imports.
"""
import os
import sys

_ROS2_PKGS_ROOT = os.path.join(os.path.dirname(__file__), "ros2_packages")

_PACKAGES = [
    "tiago_adapters",
    "fr3_adapters",
    "robot_common",
    "human_adapters",
]

for _pkg in _PACKAGES:
    _path = os.path.join(_ROS2_PKGS_ROOT, _pkg, _pkg)
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)
