from launch import LaunchDescription
import os

def generate_launch_description():
    """
    Launch file for real robot.
    USE_SIM is read from environment variable set in Robot.py
    """
    if "USE_SIM" not in os.environ:
        os.environ["USE_SIM"] = "false"  # default to real robot
    print(f"[LAUNCH] USE_SIM={os.environ['USE_SIM']}")

    return LaunchDescription()