from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Arguments: which robots to launch
    franka_enabled = LaunchConfiguration('franka_enabled', default='false')
    pepper_enabled = LaunchConfiguration('pepper_enabled', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('franka_enabled', default_value='false'),
        DeclareLaunchArgument('pepper_enabled', default_value='false'),

        Node(
            package='robots_adapters',
            executable='franka_adapter',
            name='franka',
            namespace='franka',
            condition=IfCondition(franka_enabled)
        ),

        Node(
            package='robots_adapters',
            executable='pepper_adapter',
            name='pepper',
            namespace='pepper',
            condition=IfCondition(pepper_enabled)
        ),
    ])