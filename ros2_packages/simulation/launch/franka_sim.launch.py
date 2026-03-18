from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_sim = get_package_share_directory('simulation')
    world_path = os.path.join(pkg_sim, 'worlds', 'franka.sdf')
    models_path = os.path.join(pkg_sim, 'models')

    return LaunchDescription([
        # Set environment variable so it's visible to Gazebo subprocess
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=models_path
        ),

        # Include the ros_gz_sim launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': f'-r {world_path}'  # -r = run the world
            }.items()
        ),
    ])