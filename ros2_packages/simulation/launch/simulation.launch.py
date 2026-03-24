"""
Launch file to start the Gazebo world and spawn multiple Franka robots at specified locations with delays
Used more for testing and development. 
For actual multi-agent experiments, use the top-level launch file (franka_sim.launch.py + world.launch.py).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # world launch still comes from simulation
    world_launch = os.path.join(get_package_share_directory('simulation'), 'launch', 'world.launch.py')

    # robot launch now comes from robots_adapters
    robot_launch = os.path.join(get_package_share_directory('robots_adapters'), 'launch', 'franka_sim.launch.py')

    # Define your robots here
    robots = [
        {'robot_name': 'fr3_robot1', 'x_pos': '-0.3', 'y_pos': '0.0', 'z_pos': '1.1', 'spawn_delay': '3.0'},
        {'robot_name': 'fr3_robot2', 'x_pos':  '0.5', 'y_pos': '0.0', 'z_pos': '1.1', 'spawn_delay': '4.0'},
    ]

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={'world_name': 'franka'}.items()
    )

    robot_includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments=robot.items()
        )
        for robot in robots
    ]

    return LaunchDescription([
        world,
        *robot_includes,
    ])