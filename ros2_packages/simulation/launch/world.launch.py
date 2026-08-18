"""
Launch just the world without any robots
ros2 launch simulation world.launch.py world_name:=franka
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)

    pkg_sim = get_package_share_directory('simulation')
    world_path = os.path.join(pkg_sim, 'worlds', f'{world_name}.sdf')

    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        # --headless-rendering: tells ogre2 to use EGL device mode (NVIDIA GPU)
        # instead of GLX (Intel display GPU). The GUI window still opens normally.
        # This is what fixes the ~1/70 real-time speed caused by software rendering.
        launch_arguments={'gz_args': f'-r --headless-rendering {world_path}'}.items()
    )

    return [gz_sim]


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    models_path = os.path.join(pkg_sim, 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='backyard',
            description='Name of the world file (without .sdf extension) inside worlds/'
        ),

        
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH', # GZ_SIM_RESOURCE_PATH
            value=os.pathsep.join([
                os.path.dirname(get_package_share_directory('franka_description')),
                models_path,
                os.path.dirname(get_package_share_directory('tiago_description')),
                os.path.dirname(get_package_share_directory('pmb2_description')),
                os.path.dirname(get_package_share_directory('pal_gripper_description')),
                os.path.dirname(get_package_share_directory('pal_urdf_utils')),
                os.path.join(get_package_share_directory('human_adapters'), 'models'), 
                os.path.join(get_package_share_directory('pioneer3at_adapters'), 'model'),
                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''), # GZ_SIM_RESOURCE_PATH
            ])
        ),

        OpaqueFunction(function=launch_setup),
    ])