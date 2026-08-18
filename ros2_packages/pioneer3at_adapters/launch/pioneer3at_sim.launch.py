"""
Spawn a Pioneer 3AT into a running Gazebo Fortress world.

The Pioneer 3AT model is stored locally in:
    pioneer3at_adapters/model/pioneer3at/model.sdf

Examples
--------
# Spawn with defaults
ros2 launch pioneer3at_adapters pioneer3at.launch.py

# Spawn at a specific position
ros2 launch pioneer3at_adapters pioneer3at.launch.py \
    robot_name:=pioneer3at_1 \
    x:=2.0 \
    y:=1.0 \
    z:=0.35 \
    yaw:=1.57

# Spawn into a different world
ros2 launch pioneer3at_adapters pioneer3at.launch.py \
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_pioneer(context, *args, **kwargs):

    robot_name = LaunchConfiguration('robot_name').perform(context)

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    # ---------------------------------------------------------------
    # Locate the locally installed Pioneer 3AT model
    # ---------------------------------------------------------------

    pkg_dir = get_package_share_directory('pioneer3at_adapters')

    sdf_file = os.path.join(pkg_dir, 'model', 'pioneer3at', 'model.sdf')

    if not os.path.isfile(sdf_file):
        raise FileNotFoundError(
            f"Pioneer 3AT model not found at:\n"
            f"  {sdf_file}\n\n"
            f"Make sure the model is installed in:\n"
            f"  pioneer3at_adapters/model/pioneer3at/\n\n"
            f"and that model.sdf exists."
        )

    # ---------------------------------------------------------------
    # Spawn Pioneer 3AT
    # ---------------------------------------------------------------

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-yaw', yaw,
        ],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/pioneer3at/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/pioneer3at/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/pioneer3at/cmd_vel', '/cmd_vel'),
            ('/model/pioneer3at/odometry', '/odom'),
        ],
        output='screen',
    )

    # ---------------------------------------------------------------
    # Return launch actions
    # ---------------------------------------------------------------
    return [
        TimerAction(period=5.0, actions=[bridge_node, spawn_node]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='pioneer3at', description='Name of the Pioneer 3AT model in Gazebo'),
        DeclareLaunchArgument('x',          default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('y',          default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('z',          default_value='0.35', description='Initial Z position'),
        DeclareLaunchArgument('yaw',        default_value='0.0', description='Initial yaw orientation'),
        OpaqueFunction(function=spawn_pioneer),
    ])