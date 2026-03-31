"""
Generates the controller configuration YAML file for a given robot name and writes it to both the install share path and the src path of the package. 
This ensures that the configuration is available at runtime and also serves as a source of truth for development.
The expected robot is FR3
"""
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = "fr3_adapters"


def get_package_src_directory(package_name: str) -> str:
    """Find the src directory by searching common workspace locations."""
    search_roots = [
        "/ros2_ws/src",
        os.path.expanduser("~/ros2_ws/src"),
        os.path.join(os.getcwd(), "src"),
    ]

    for root in search_roots:
        candidate = os.path.join(root, package_name)
        if os.path.isdir(candidate):
            return candidate

    raise FileNotFoundError(
        f"Could not find src directory for package '{package_name}'. "
        f"Searched in: {search_roots}"
    )


def build_config(robot_name: str) -> dict:
    return {
        "/**": {
            "ros__parameters": {
                "update_rate": 1000,
                "use_sim_time": True,
            }
        },
        robot_name: {
            "controller_manager": {
                "ros__parameters": {
                    "update_rate": 1000,
                    "use_sim_time": True,
                    "joint_state_broadcaster": {
                        "type": "joint_state_broadcaster/JointStateBroadcaster"
                    },
                    "arm_controller": {
                        "type": "joint_trajectory_controller/JointTrajectoryController"
                    },
                    "gripper_controller": {
                        "type": "joint_trajectory_controller/JointTrajectoryController"
                    }
                }
            },
            "joint_state_broadcaster": {
                "ros__parameters": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                }
            },
            "arm_controller": {
                "ros__parameters": {
                   "type": "joint_trajectory_controller/JointTrajectoryController",
                    "joints": [f"{robot_name}_fr3_joint{i}" for i in range(1, 8)],
                    "command_interfaces": ["position"],
                    "state_interfaces": ["position", "velocity"],
                    "state_publish_rate": 100.0,
                    "action_monitor_rate": 20.0,
                    "allow_partial_joints_goal": False,
                }
            },
            "gripper_controller": {
                "ros__parameters": {
                    "type": "joint_trajectory_controller/JointTrajectoryController",
                    "joints": [
                        f"{robot_name}_fr3_finger_joint1",
                        f"{robot_name}_fr3_finger_joint2"
                    ],
                    "command_interfaces": ["position"],
                    "state_interfaces": ["position", "velocity"],
                    "state_publish_rate": 100.0,
                    "action_monitor_rate": 20.0,
                    "allow_partial_joints_goal": False,
                }
            },
        },
    }


def write_yaml(path: str, config: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    print(f"  ✔ Written to {path}")


def generate_controller_config(robot_name: str) -> None:
    print(f"\nGenerating config for: {robot_name}")

    config = build_config(robot_name)
    filename = f"{robot_name}_controllers.yaml"
    errors = []

    # 1. Write to install share path (what ROS 2 reads at runtime)
    try:
        install_dir = get_package_share_directory(PACKAGE_NAME)
        install_path = os.path.join(install_dir, "config", filename)
        write_yaml(install_path, config)
    except Exception as e:
        errors.append(f"Install path failed: {e}")
        print(f"  ✘ Install path failed: {e}")

    # 2. Write to src path (source of truth)
    try:
        src_dir = get_package_src_directory(PACKAGE_NAME)
        src_path = os.path.join(src_dir, "config", filename)
        write_yaml(src_path, config)
    except Exception as e:
        errors.append(f"Src path failed: {e}")
        print(f"  ✘ Src path failed: {e}")

    if len(errors) == 2:
        raise RuntimeError(
            f"Failed to write config to both locations:\n" + "\n".join(errors)
        )

    print(f"\nDone! Config ready for robot: {robot_name}\n")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 generate_config.py <robot_name>")
        print("Example: python3 generate_config.py robot2")
        sys.exit(1)

    robot_name = sys.argv[1]
    generate_controller_config(robot_name)