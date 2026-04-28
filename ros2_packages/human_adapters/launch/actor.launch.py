import os
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_actor(context, *args, **kwargs):
    namespace  = LaunchConfiguration('namespace').perform(context)
    actor_name = LaunchConfiguration('actor_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    x          = LaunchConfiguration('x').perform(context)
    y          = LaunchConfiguration('y').perform(context)
    z          = LaunchConfiguration('z').perform(context)

    pkg_dir  = get_package_share_directory('human_adapters')
    sdf_path = os.path.join(pkg_dir, 'models', 'WalkingActor', 'model.sdf')

    with open(sdf_path, 'r') as f:
        sdf = f.read()

    sdf = sdf.replace('human_actor', actor_name)
    sdf = sdf.replace('/human_1/cmd_pose', f'/{namespace}/cmd_pose')

    # Write SDF to temp file to avoid shell quoting issues
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
        f.write(sdf)
        sdf_file = f.name

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', world_name,
            '-file',  sdf_file,   # -file instead of -string
            '-name',  actor_name,
            '-x', x, '-y', y, '-z', z,
        ],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='actor_bridge',
        arguments=[
            f'/{namespace}/cmd_pose@geometry_msgs/msg/Pose]ignition.msgs.Pose',
        ],
        output='screen',
    )

    controller_node = Node(
        package='human_adapters',
        executable='actor_controller',
        namespace=namespace,
        name='actor_controller',
        parameters=[{
            'actor_name': actor_name,
            'world_name': world_name,
        }],
        output='screen',
    )

    return [
        bridge_node,                                        # starts immediately
        TimerAction(period=5.0, actions=[spawn_node]),      # waits for Gazebo
        TimerAction(period=7.0, actions=[controller_node]), # waits for actor to be spawned
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace',  default_value='human_1'),
        DeclareLaunchArgument('actor_name', default_value='human_1'),
        DeclareLaunchArgument('world_name', default_value='backyard'),
        DeclareLaunchArgument('x',          default_value='0.0'),
        DeclareLaunchArgument('y',          default_value='0.0'),
        DeclareLaunchArgument('z',          default_value='0.0'),

        OpaqueFunction(function=spawn_actor),
    ])