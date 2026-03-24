"""
Add robots individually (in separate terminals or scripts)
ros2 launch simulation franka_sim.launch.py robot_name:=robot1 x_pos:=-0.3 spawn_delay:=3.0
ros2 launch simulation franka_sim.launch.py robot_name:=robot2 x_pos:=0.5  spawn_delay:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    x_pos       = LaunchConfiguration('x_pos').perform(context)
    y_pos       = LaunchConfiguration('y_pos').perform(context)
    z_pos       = LaunchConfiguration('z_pos').perform(context)
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

    pkg_robots = get_package_share_directory('robots_adapters')

    controllers_file = os.path.join(
        pkg_robots, 'config', f'{robot_name}_controllers.yaml'
    )

    if not os.path.exists(controllers_file):
        raise FileNotFoundError(f"Controllers file not found: {controllers_file}")

    franka_xacro = PathJoinSubstitution([
        FindPackageShare('robots_adapters'),
        'urdf', 'fr3_gz.urdf.xacro'
    ])

    # --- Robot State Publisher ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', franka_xacro,
                    ' arm_prefix:=',       robot_name,
                    ' robot_namespace:=',  robot_name,
                    ' controllers_file:=', controllers_file,
                ]),
                value_type=str
            ),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # --- Spawn into Gazebo world ---
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  robot_name,
            '-topic', f'/{robot_name}/robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
        ],
        output='screen'
    )

    # --- Controllers ---
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_name}/controller_manager',
        ],
        output='screen'
    )

    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--param-file', controllers_file,
        ],
        output='screen'
    )

    # --- Event chain: spawn → jsb → arm ---
    start_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb])
    )
    start_arm = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[arm])
    )

    return [
        rsp,
        TimerAction(period=spawn_delay, actions=[spawn]),
        start_jsb,
        start_arm,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1',
            description='Robot name — used for namespace, arm_prefix, topic and controllers file'
        ),
        DeclareLaunchArgument('x_pos',       default_value='0.0'),
        DeclareLaunchArgument('y_pos',       default_value='0.0'),
        DeclareLaunchArgument('z_pos',       default_value='1.1'),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='3.0',
            description='Seconds to wait before spawning, to give Gazebo time to load'
        ),

        OpaqueFunction(function=launch_setup),
    ])