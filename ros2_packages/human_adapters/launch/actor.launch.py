"""
Spawn a human actor and its controller. The controller waits for commands
via a ROS 2 topic pub --once — it does NOT run any trajectory automatically.

Available topic commands  (send via ros2 topic pub):
  goto   – move to (x, y) then face final_yaw
  stop   – freeze in place
  test   – run the built-in square-loop trajectory (original behaviour)
  finish – execute the current action then shut the node down cleanly

Examples
--------
# Spawn
ros2 launch human_adapters actor.launch.py actor_type:=WalkingActor color:=red  actor_name:=w1

ros2 launch human_adapters actor.launch.py actor_type:=CasualFemale  color:=orange actor_name:=c1

ros2 launch human_adapters actor.launch.py actor_type:=FemaleVisitor color:=yellow actor_name:=v1


"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_actor(context, *args, **kwargs):
    actor_name = LaunchConfiguration('actor_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    actor_type = LaunchConfiguration('actor_type').perform(context)
    color      = LaunchConfiguration('color').perform(context)
    x          = LaunchConfiguration('x').perform(context)
    y          = LaunchConfiguration('y').perform(context)
    yaw        = LaunchConfiguration('yaw').perform(context)

    # Model folder is simply <ActorType>_<color> — pre-built by patch_actor.py
    if color:
        model_variant = f"{actor_type}_{color}"
    else:
        model_variant = actor_type

    pkg_dir  = get_package_share_directory('human_adapters')
    sdf_file = os.path.join(pkg_dir, 'models', model_variant, 'model.sdf')

    if not os.path.isfile(sdf_file):
        raise FileNotFoundError(
            f"Model variant '{model_variant}' not found at {sdf_file}.\n"
            f"Run:  python3 patch_actor.py --models-dir <src>/models "
            f"--actor-type {actor_type} --color {color}\n"
            f"Then rebuild:  colcon build"
        )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', world_name,
            '-file',  sdf_file,
            '-name',  actor_name,
            '-x', x, '-y', y, '-yaw', yaw
        ],
        output='screen',
    )

    controller_node = Node(
        package='human_adapters',
        executable='actor_controller',
        namespace=actor_name,
        name='actor_controller',
        parameters=[{
            'actor_name':  actor_name,
            'world_name':  world_name,
            'initial_x':   float(x),
            'initial_y':   float(y),
            'initial_yaw': float(yaw),
        }],
        output='screen',
    )

    return [
        TimerAction(period=5.0,  actions=[spawn_node]),
        TimerAction(period=10.0, actions=[controller_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('actor_name',  default_value='human_host_1'),
        DeclareLaunchArgument('world_name',  default_value='backyard'),
        DeclareLaunchArgument('actor_type',  default_value='WalkingActor',
                              description='WalkingActor | CasualFemale | FemaleVisitor'),
        DeclareLaunchArgument('color',       default_value='',
                              description='Optional colour variant (leave empty to use default model)'),
        DeclareLaunchArgument('x',           default_value='0.0'),
        DeclareLaunchArgument('y',           default_value='0.0'),
        DeclareLaunchArgument('yaw',         default_value='0.0'),

        OpaqueFunction(function=spawn_actor),
    ])