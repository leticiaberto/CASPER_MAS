import os
import tempfile
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_actor(context, *args, **kwargs):
    actor_name = LaunchConfiguration('actor_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    x          = LaunchConfiguration('x').perform(context)
    y          = LaunchConfiguration('y').perform(context)
    shirt_r    = LaunchConfiguration('shirt_r').perform(context)
    shirt_g    = LaunchConfiguration('shirt_g').perform(context)
    shirt_b    = LaunchConfiguration('shirt_b').perform(context)

    namespace = actor_name  # for controller node

    pkg_dir = get_package_share_directory('human_adapters')

    sdf_path   = os.path.join(pkg_dir, 'models', model_name, 'model.sdf')
    source_dae = os.path.join(pkg_dir, 'models', model_name, 'meshes', 'walk.dae')
    patch_script = os.path.join(pkg_dir, 'scripts', 'patch_actor.py')

    # ── 1. Patch the .dae ────────────────────────────────────────────────────
    patched_dae = tempfile.NamedTemporaryFile(
        mode='w', suffix='.dae', delete=False, prefix=f'{actor_name}_'
    )
    patched_dae_path = patched_dae.name
    patched_dae.close()

    subprocess.run([
        'python3', patch_script,
        '--input',  source_dae,
        '--output', patched_dae_path,
        '--shirt-color', shirt_r, shirt_g, shirt_b,
    ], check=True)

    # ── 2. Patch the SDF ─────────────────────────────────────────────────────
    with open(sdf_path, 'r') as f:
        sdf = f.read()

    sdf = sdf.replace('human_actor', actor_name)
    sdf = sdf.replace(
        f'model://{model_name}/meshes/walk.dae',
        patched_dae_path
    )

    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
        f.write(sdf)
        sdf_file = f.name

    # ── 3. Spawn + controller ─────────────────────────────────────────────────
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', world_name,
            '-file',  sdf_file,
            '-name',  actor_name,
            '-x', x, '-y', y,
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
        TimerAction(period=5.0,  actions=[spawn_node]),
        TimerAction(period=10.0, actions=[controller_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('actor_name',  default_value='human_1'),
        DeclareLaunchArgument('world_name',  default_value='backyard'),
        DeclareLaunchArgument('model_name',  default_value='WalkingActor',
                              description='Subfolder name under models/'),
        DeclareLaunchArgument('x',           default_value='0.0'),
        DeclareLaunchArgument('y',           default_value='0.0'),

        # Shirt colour — defaults reproduce the original green sweater
        DeclareLaunchArgument('shirt_r', default_value='0.098'),
        DeclareLaunchArgument('shirt_g', default_value='0.255'),
        DeclareLaunchArgument('shirt_b', default_value='0.075'),

        OpaqueFunction(function=spawn_actor),
    ])