import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder

def launch_robot(context):
    robot_id = os.environ.get("ROBOT_ID", "robot1")
    robot_model = os.environ.get("ROBOT_MODEL", "virtual")

    actions = []

    # ------------------------------------------------
    # FRANKA ROBOT
    # ------------------------------------------------
    if robot_model == "frankaresearch3":
        moveit_config = (
            MoveItConfigsBuilder(
                "panda",
                package_name="moveit_resources_panda_moveit_config",
            )
            .to_moveit_configs()
        )

        actions.append(PushRosNamespace(robot_id))

        actions.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        )

        actions.append(
            Node(
                package="robots_adapters",
                executable="franka_adapter",
                output="screen",
                namespace=robot_id,
            )
        )

    # ------------------------------------------------
    # VIRTUAL ROBOT
    # ------------------------------------------------
    elif robot_model == "virtual":
        actions.append(
            Node(
                package="robots_adapters",
                executable="virtual_robot_node",
                namespace=robot_id,
                output="screen",
            )
        )

    # ------------------------------------------------
    # PEPPER ROBOT
    # ------------------------------------------------
    elif robot_model == "pepper":
        actions.append(
            Node(
                package="robots_adapters",
                executable="pepper_node",
                namespace=robot_id,
                output="screen",
            )
        )

    else:
        raise RuntimeError(f"Unknown robot type: {robot_model}")

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_robot)
    ])