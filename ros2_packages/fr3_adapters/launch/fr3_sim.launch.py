"""
Spawns the FR3 robot into Ignition Fortress with a world-fixed base.

The world joint is baked into the SDF at spawn time (not defined in the
world SDF, which gets ignored for dynamically spawned models in Ignition Fortress).

Strategy
--------
1. Process xacro → URDF at launch time
2. Convert URDF → SDF using `ign sdf -p`
3. Inject a fixed <joint> connecting world→base inside the SDF model
4. Spawn the modified SDF directly via -file (not -topic)

This guarantees the joint exists when the model loads into the physics engine.

Usage:
    ros2 launch fr3_adapters franka_sim.launch.py robot_name:=fr3_robot1
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    x_pos       = LaunchConfiguration('x_pos').perform(context)
    y_pos       = LaunchConfiguration('y_pos').perform(context)
    z_pos       = LaunchConfiguration('z_pos').perform(context)
    yaw         = LaunchConfiguration('yaw').perform(context)
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

    pkg_robots = get_package_share_directory('fr3_adapters')
    xacro_file = os.path.join(pkg_robots, 'urdf', 'fr3_gz.urdf.xacro')
    controllers_file = os.path.join(pkg_robots, 'config', f'{robot_name}_controllers.yaml')

    if not os.path.exists(controllers_file):
        raise FileNotFoundError(f"Controllers file not found: {controllers_file}")

    # ------------------------------------------------------------------
    # Build the spawn arguments.
    # Try to convert URDF→SDF and inject world joint. Fall back to -topic.
    # ------------------------------------------------------------------
    spawn_args = _build_spawn_args(
        xacro_file, controllers_file, robot_name,
        x_pos, y_pos, z_pos, yaw,
    )

    # ------------------------------------------------------------------
    # Robot State Publisher — needed for TF and joint_states topic
    # ------------------------------------------------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ', xacro_file,
                    ' arm_prefix:=',       robot_name,
                    ' robot_namespace:=',  robot_name,
                    ' controllers_file:=', controllers_file,
                ]),
                value_type=str,
            ),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Spawn model into Gazebo
    # ------------------------------------------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=spawn_args,
        output='screen',
    )

    # ------------------------------------------------------------------
    # Controllers — chained after spawn
    # ------------------------------------------------------------------
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_name}/controller_manager',
        ],
        output='screen',
    )

    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--param-file', controllers_file,
        ],
        output='screen',
    )

    gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', f'/{robot_name}/controller_manager',
            '--param-file', controllers_file,
        ],
        output='screen',
    )

    start_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[
            TimerAction(period=2.0, actions=[jsb])
        ])
    )
    start_arm     = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm]))
    start_gripper = RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[gripper]))

    return [
        rsp,
        TimerAction(period=spawn_delay, actions=[spawn]),
        start_jsb,
        start_arm,
        start_gripper,
    ]


def _build_spawn_args(xacro_file, controllers_file, robot_name,
                      x_pos, y_pos, z_pos, yaw):
    """
    Build the argument list for ros_gz_sim/create.

    Tries to:
      1. xacro → URDF
      2. URDF → SDF (via `ign sdf -p`)
      3. Inject world-fixed joint into SDF
      4. Return -file <sdf_path> spawn args

    Falls back to -topic spawn if any step fails.
    """
    try:
        # Step 1: xacro → URDF
        urdf_str = subprocess.check_output([
            'xacro', xacro_file,
            f'arm_prefix:={robot_name}',
            f'robot_namespace:={robot_name}',
            f'controllers_file:={controllers_file}',
        ], stderr=subprocess.PIPE).decode()

        # Write URDF to temp file
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, mode='w') as f:
            f.write(urdf_str)
            urdf_path = f.name

        # Step 2: URDF → SDF
        sdf_str = subprocess.check_output(
            ['ign', 'sdf', '-p', urdf_path],
            stderr=subprocess.PIPE,
        ).decode()
        os.unlink(urdf_path)

        # Step 3: Inject world-fixed joint inside the <model> block.
        # In a standalone SDF (not world SDF), the joint child is just
        # the link name — no model:: prefix needed.
        world_joint = (
            f'\n    <joint name="{robot_name}_world_anchor" type="fixed">'
            f'\n      <parent>world</parent>'
            f'\n      <child>base</child>'
            f'\n    </joint>'
        )
        if '</model>' in sdf_str:
            sdf_str = sdf_str.replace('</model>', f'{world_joint}\n  </model>', 1)
            print(f'[franka_sim.launch] Injected world joint into SDF for {robot_name}')
        else:
            print('[franka_sim.launch] WARNING: </model> not found in SDF — world joint not injected')

        # Write final SDF to temp file (kept alive — cleaned up by OS on exit)
        with tempfile.NamedTemporaryFile(suffix='.sdf', delete=False, mode='w') as f:
            f.write(sdf_str)
            sdf_path = f.name

        print(f'[franka_sim.launch] Spawning from SDF: {sdf_path}')
        return [
            '-name', robot_name,
            '-file', sdf_path,
            '-x', x_pos, '-y', y_pos, '-z', z_pos, '-Y', yaw,
        ]

    except Exception as e:
        print(f'[franka_sim.launch] SDF generation failed ({e}), falling back to -topic spawn')
        print('[franka_sim.launch] WARNING: world joint will NOT be applied — robot may fall')
        return [
            '-name', robot_name,
            '-topic', f'/{robot_name}/robot_description',
            '-x', x_pos, '-y', y_pos, '-z', z_pos, '-Y', yaw,
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='fr3_robot1'),
        DeclareLaunchArgument('x_pos',      default_value='0.0'),
        DeclareLaunchArgument('y_pos',      default_value='0.0'),
        DeclareLaunchArgument('z_pos',      default_value='1.03'),
        DeclareLaunchArgument('yaw',        default_value='0.0'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0'),
        OpaqueFunction(function=launch_setup),
    ])
