from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (TimerAction, IncludeLaunchDescription,
                             SetEnvironmentVariable, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    world_path = os.path.join(pkg_sim, 'worlds', 'franka.sdf')
    models_path = os.path.join(pkg_sim, 'models')

    controllers_file_r1 = os.path.join(pkg_sim, 'config', 'fr3_controllers_robot1.yaml')
    controllers_file_r2 = os.path.join(pkg_sim, 'config', 'fr3_controllers_robot2.yaml')

    franka_xacro = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'urdf', 'fr3_gz.urdf.xacro'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # ── Robot 1 ──────────────────────────────────────────────────────────────

    robot1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', franka_xacro,
                    ' arm_prefix:=robot1',
                    ' robot_namespace:=robot1',
                    ' controllers_file:=', controllers_file_r1,
                ]),
                value_type=str
            ),
            'use_sim_time': True,
        }],
        output='screen'
    )

    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fr3_1',
            '-topic', '/robot1/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen'
    )

    robot1_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/robot1/controller_manager'
        ],
        output='screen'
    )

    robot1_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/robot1/controller_manager',
            '--param-file', controllers_file_r1,
        ],
        output='screen'
    )

    # ── Robot 2 ──────────────────────────────────────────────────────────────

    robot2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', franka_xacro,
                    ' arm_prefix:=robot2',
                    ' robot_namespace:=robot2',
                    ' controllers_file:=', controllers_file_r2,
                ]),
                value_type=str
            ),
            'use_sim_time': True,
        }],
        output='screen'
    )

    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fr3_2',
            '-topic', '/robot2/robot_description',
            '-x', '1.5', '-y', '0.0', '-z', '0.0',
        ],
        output='screen'
    )

    robot2_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/robot2/controller_manager'
        ],
        output='screen'
    )

    robot2_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/robot2/controller_manager',
            '--param-file', controllers_file_r2,
        ],
        output='screen'
    )

    # ── Event-driven controller startup ──────────────────────────────────────

    start_r1_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot1, on_exit=[robot1_jsb])
    )
    start_r1_arm = RegisterEventHandler(
        OnProcessExit(target_action=robot1_jsb, on_exit=[robot1_arm])
    )
    start_r2_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot2, on_exit=[robot2_jsb])
    )
    start_r2_arm = RegisterEventHandler(
        OnProcessExit(target_action=robot2_jsb, on_exit=[robot2_arm])
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.pathsep.join([
                os.path.dirname(get_package_share_directory('franka_description')),
                models_path
            ])
        ),

        gz_sim,

        robot1_rsp,
        robot2_rsp,

        # Give Gazebo time to fully load before spawning
        TimerAction(period=3.0, actions=[spawn_robot1]),
        TimerAction(period=4.0, actions=[spawn_robot2]),

        start_r1_jsb,
        start_r1_arm,
        start_r2_jsb,
        start_r2_arm,
    ])