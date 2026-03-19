from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
import os


def generate_launch_description():
    # Paths
    pkg_sim = get_package_share_directory('simulation')
    world_path = os.path.join(pkg_sim, 'worlds', 'franka.sdf')
    models_path = os.path.join(pkg_sim, 'models')

    # Xacro path (FR3)
    franka_xacro = PathJoinSubstitution([
        FindPackageShare('franka_description'),
        'robots',
        'fr3',
        'fr3.urdf.xacro'
    ])

    # =========================
    # ROBOT 1
    # =========================
    robot1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    franka_xacro,
                    ' prefix:=robot1_',
                    ' use_gazebo:=true'
                ]),
                value_type=str  # Important! Treat as string, not YAML
            )
        }],
        output='screen'
    )

    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'franka_1',
            '-topic', '/robot1/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # =========================
    # ROBOT 2
    # =========================
    robot2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    franka_xacro,
                    ' prefix:=robot2_',
                    ' use_gazebo:=true'
                ]),
                value_type=str
            )
        }],
        output='screen'
    )

    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'franka_2',
            '-topic', '/robot2/robot_description',
            '-x', '1.5', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([

        # Gazebo resource path
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.pathsep.join([
                os.path.join(get_package_share_directory('franka_description'), 'share'),
                models_path
            ])
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': f'-r {world_path}'
            }.items()
        ),

        # Robot state publishers
        robot1_rsp,
        robot2_rsp,

        # Delay spawn to ensure robot_description is ready
        TimerAction(period=2.0, actions=[spawn_robot1]),
        TimerAction(period=3.0, actions=[spawn_robot2]),
    ])