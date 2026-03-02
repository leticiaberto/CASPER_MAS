from launch import LaunchDescription
import os

def generate_launch_description():
    """
    This launch file sets USE_SIM based on the environment variable.
    It does not start any ROS nodes itself — the nodes (adapters) will
    read USE_SIM directly from os.environ.
    """
    # Ensure USE_SIM is set for adapters
    if "USE_SIM" not in os.environ:
        os.environ["USE_SIM"] = "true"  # default to simulation
    print(f"[LAUNCH] USE_SIM={os.environ['USE_SIM']}")

    return LaunchDescription()