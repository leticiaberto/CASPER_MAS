"""
Spawns Tiago (arm + pal-gripper) into Ignition Fortress.

Key insight
-----------
tiago_description/ros2_control/ros2_control.urdf.xacro already contains
the correct Ignition plugin block, gated on gazebo_version == 'gazebo':

  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>.../gazebo_controller_manager_cfg.yaml</parameters>
  </plugin>

  hardware: <plugin>gz_ros2_control/GazeboSimSystem</plugin>

Passing gazebo_version:=gazebo activates both.  No runtime patching needed.
The controller manager cfg yaml inside tiago_description also carries
update_rate and all controller type definitions, fixing those errors.

Only one minimal patch remains (patch_namespace):
  The xacro does not inject <ros><namespace> into the plugin block.
  gz_ros2_control therefore looks for robot_state_publisher globally.
  We inject the namespace after xacro runs so the plugin finds the
  namespaced RSP and registers controller_manager under /{robot_name}/.

Why no local xacro copy
-----------------------
The tiago.urdf.xacro pulls in ~20 other PAL xacros via $(find ...).
Copying it would mean owning all those transitive dependencies forever.
Passing the right args + one targeted string inject is far cleaner.

Why no world joint
------------------
Tiago has a mobile base — floor collision + diff-drive keep it upright.
Contrast with FR3 which needed a world→base fixed joint injection.

Multi-robot
-----------
  ros2 launch robots_adapters tiago_sim.launch.py robot_name:=tiago_robot1
  ros2 launch robots_adapters tiago_sim.launch.py robot_name:=tiago_robot2 x_pos:=2.0
"""

import os
import re
import subprocess
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Minimal URDF patch — namespace injection only
# ---------------------------------------------------------------------------
def patch_urdf(urdf_str: str, robot_name: str) -> str:
    """
    Two patches applied to the URDF before passing to RSP:

    1. Namespace injection
       The xacro does not set <ros><namespace> on the gz_ros2_control plugin.
       Without it the plugin looks for robot_state_publisher globally and loops.
       We inject <ros><namespace>/{robot_name}</namespace></ros>.

    2. Parameters path injection
       The xacro <parameters> tag points to gazebo_controller_manager_cfg.yaml
       which uses /controller_manager: (global node name). With our namespace
       the CM lives at /tiago_robot1/controller_manager — the global path never
       applies, so update_rate and controller types are never set.
       We inject a second <parameters> tag pointing to our temp YAML which has
       the correct /{robot_name}/controller_manager: section.
       gz_ros2_control merges multiple <parameters> tags, so both files apply.
    """
    # 1. Namespace injection
    ns_block = (
        f'\n      <ros>\n'
        f'        <namespace>/{robot_name}</namespace>\n'
        f'      </ros>'
    )
    patched = re.sub(
        r'(<plugin[^>]*GazeboSimROS2ControlPlugin[^>]*>)',
        r'\1' + ns_block,
        urdf_str,
    )
    if patched == urdf_str:
        print(f'[tiago_sim.launch] WARNING: GazeboSimROS2ControlPlugin not found.')
        return patched

    # NOTE: <parameters> tag injection is intentionally skipped.
    #
    # gz_ros2_control in Ignition Fortress has a bug: it prepends "--params-file"
    # internally and then passes the whole string "--params-file /path/file.yaml"
    # as a single token to rcl, which rejects it with:
    #   "node name must not contain characters other than alphanumerics or '_'"
    #
    # The CM update_rate defaults to 100 Hz (harmless warning, not a failure).
    # Controller types are set by the spawner's --param-file via the parameter
    # service right before load_controller is called — no URDF tag needed.

    print(f'[tiago_sim.launch] [{robot_name}] URDF patched (namespace injected)')
    return patched



# ---------------------------------------------------------------------------
# Namespaced controller YAML builder
# ---------------------------------------------------------------------------
def _build_namespaced_controllers_yaml(robot_name: str) -> str:
    """
    Build a single namespaced controller YAML for all spawners.

    Structure needed by ros2_control in humble:

        /{robot_name}:
          controller_manager:
            ros__parameters:
              update_rate: 100          ← CM needs this (plugin YAML uses /controller_manager
              joint_state_broadcaster:    which is global and never matches /robot/CM)
                type: joint_state_broadcaster/JointStateBroadcaster
              arm_controller:
                type: joint_trajectory_controller/JointTrajectoryController
              ...                       ← all types pre-registered here

          arm_controller:
            ros__parameters:            ← per-controller params (joints, interfaces, etc.)
              joints: [...]

    The CM section is what tells the controller_manager what plugin to load
    when the spawner calls load_controller. Without it the CM says
    "type param not defined" even when the YAML has types under each controller.
    """
    from ament_index_python.packages import get_package_share_directory

    # ── Controller manager section ──────────────────────────────────────
    # Registers types + update_rate under /{robot_name}/controller_manager.
    # The gazebo_controller_manager_cfg.yaml uses /controller_manager: (global)
    # so it never reaches our namespaced CM — we must supply update_rate here.
    cm_section = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 100,
                'use_sim_time': True,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster',
                },
                'arm_controller': {
                    'type': 'joint_trajectory_controller/JointTrajectoryController',
                },
                'gripper_controller': {
                    'type': 'joint_trajectory_controller/JointTrajectoryController',
                },
                'mobile_base_controller': {
                    'type': 'diff_drive_controller/DiffDriveController',
                },
                'torso_controller': {
                    'type': 'joint_trajectory_controller/JointTrajectoryController',
                },
                'head_controller': {
                    'type': 'joint_trajectory_controller/JointTrajectoryController',
                },
            }
        }
    }

    # ── Per-controller parameter sections ──────────────────────────────
    # Load individual PAL YAMLs (joint names, interfaces, gains, etc.)
    sources = []

    pkg_tiago_ctrl = get_package_share_directory('tiago_controller_configuration')
    tiago_cfg = os.path.join(pkg_tiago_ctrl, 'config')
    for name in ['joint_state_broadcaster.yaml', 'arm_controller.yaml',
                 'torso_controller.yaml', 'head_controller.yaml']:
        sources.append(os.path.join(tiago_cfg, name))

    pkg_pmb2_ctrl = get_package_share_directory('pmb2_controller_configuration')
    sources.append(os.path.join(pkg_pmb2_ctrl, 'config', 'mobile_base_controller_public_sim.yaml'))

    controllers = {}
    for path in sources:
        if not os.path.exists(path):
            print(f'[tiago_sim.launch] WARNING: not found: {path}')
            continue
        with open(path) as f:
            data = yaml.safe_load(f)
        if data:
            controllers.update(data)

    # Gripper: pal_gripper_controller_configuration/gripper_controller.yaml contains
    # unexpanded xacro template variable '${EE_SIDE_PREFIX}_controller' which means
    # there is no 'gripper_controller' key in the file. Hardcode the params directly.
    pkg_gripper_ctrl = get_package_share_directory('pal_gripper_controller_configuration')
    gripper_yaml_path = os.path.join(pkg_gripper_ctrl, 'config', 'gripper_controller.yaml')
    gripper_params_set = False
    if os.path.exists(gripper_yaml_path):
        with open(gripper_yaml_path) as f_g:
            gripper_raw = yaml.safe_load(f_g)
        if gripper_raw and 'gripper_controller' in gripper_raw:
            controllers['gripper_controller'] = gripper_raw['gripper_controller']
            gripper_params_set = True
    if not gripper_params_set:
        controllers['gripper_controller'] = {
            'ros__parameters': {
                'joints': ['gripper_left_finger_joint', 'gripper_right_finger_joint'],
                'command_interfaces': ['position'],
                'state_interfaces': ['position'],
                'open_loop_control': True,
                'constraints': {
                    'gripper_left_finger_joint': {'goal': 0.01},
                    'gripper_right_finger_joint': {'goal': 0.01},
                    'goal_time': 3.0,
                    'stopped_velocity_tolerance': 5.0,
                },
            }
        }

    # ── Build final YAML ────────────────────────────────────────────────
    #
    # ROS 2 param file format: each top-level key is a NODE NAME (full path).
    # rcl matches params to a node by its fully qualified name.
    #
    # WRONG (what we had — nests controller under CM key):
    #   /tiago_robot1:
    #     arm_controller:           ← rcl treats this as CM sub-param
    #       ros__parameters: ...
    #
    # CORRECT (each controller gets its own top-level key):
    #   /tiago_robot1/arm_controller:    ← rcl matches this node path
    #     ros__parameters: ...
    #
    # The CM also needs its own entry so update_rate and type registrations
    # reach /tiago_robot1/controller_manager (not just /controller_manager).
    #
    # Filter out bad keys from gripper YAML (unexpanded ${EE_SIDE_PREFIX}).

    namespaced = {
        f'/{robot_name}/controller_manager': cm_section['controller_manager'],
    }

    for ctrl_name, ctrl_cfg in controllers.items():
        # Skip unexpanded xacro template variables
        if '$' in ctrl_name or '{' in ctrl_name:
            continue
        namespaced[f'/{robot_name}/{ctrl_name}'] = ctrl_cfg

    with tempfile.NamedTemporaryFile(
        suffix=f'_{robot_name}_controllers.yaml',
        delete=False, mode='w'
    ) as f:
        yaml.dump(namespaced, f, default_flow_style=False)
        tmp_path = f.name

    print(f'[tiago_sim.launch] [{robot_name}] Controllers YAML: {tmp_path}')
    return tmp_path


# ---------------------------------------------------------------------------
# Launch setup
# ---------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    robot_name  = LaunchConfiguration('robot_name').perform(context)
    x_pos       = LaunchConfiguration('x_pos').perform(context)
    y_pos       = LaunchConfiguration('y_pos').perform(context)
    z_pos       = LaunchConfiguration('z_pos').perform(context)
    yaw         = LaunchConfiguration('yaw').perform(context)
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_tiago_desc = get_package_share_directory('tiago_description')
    xacro_file     = os.path.join(pkg_tiago_desc, 'robots', 'tiago.urdf.xacro')

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(
            f"tiago.urdf.xacro not found at {xacro_file}.\n"
            "Make sure tiago_robot (humble-devel) is built in your workspace."
        )



    # ------------------------------------------------------------------
    # Build namespaced controllers YAML first — path is embedded in URDF
    # so the CM reads it at Gazebo startup (before any spawner runs).
    # ------------------------------------------------------------------
    controllers_file = _build_namespaced_controllers_yaml(robot_name)

    # ------------------------------------------------------------------
    # xacro → URDF
    # gazebo_version:=gazebo  activates gz_ros2_control plugin + GazeboSimSystem
    # is_public_sim:=True     switches sensor plugins from PAL-internal to public
    # ------------------------------------------------------------------
    print(f'[tiago_sim.launch] [{robot_name}] Running xacro…')
    urdf_str = subprocess.check_output([
        'xacro', xacro_file,
        f'namespace:={robot_name}',
        'end_effector:=pal-gripper',
        'gazebo_version:=gazebo',    # ← activates Ignition plugin in xacro
        'is_public_sim:=True',
        'use_sim_time:=True',
    ], stderr=subprocess.PIPE).decode()

    # ------------------------------------------------------------------
    # Patch URDF: inject namespace + controllers YAML path into plugin block
    # ------------------------------------------------------------------
    urdf_str = patch_urdf(urdf_str, robot_name)

    # ------------------------------------------------------------------
    # Robot State Publisher
    # Publishes patched URDF to /{robot_name}/robot_description
    # ------------------------------------------------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': urdf_str,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Spawn — reads URDF from /{robot_name}/robot_description topic.
    # package:// URIs resolved via ROS ament index (no model:// problems).
    # ------------------------------------------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  robot_name,
            '-topic', f'/{robot_name}/robot_description',
            '-x', x_pos, '-y', y_pos, '-z', z_pos, '-Y', yaw,
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Controllers — all scoped to /{robot_name}/controller_manager
    # The gz_ros2_control plugin creates controller_manager there once
    # it successfully connects to RSP (which the namespace injection enables).
    #
    # Chain: spawn → jsb (2 s) → arm → gripper → mobile_base → torso
    # ------------------------------------------------------------------
    cm = f'/{robot_name}/controller_manager'

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=['joint_state_broadcaster',
                   '--controller-manager', cm,
                   '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster',
                   '--param-file', controllers_file],
        output='screen',
    )
    arm = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=['arm_controller',
                   '--controller-manager', cm,
                   '--controller-type', 'joint_trajectory_controller/JointTrajectoryController',
                   '--param-file', controllers_file],
        output='screen',
    )
    gripper = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=['gripper_controller',
                   '--controller-manager', cm,
                   '--controller-type', 'joint_trajectory_controller/JointTrajectoryController',
                   '--param-file', controllers_file],
        output='screen',
    )
    mobile_base = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=['mobile_base_controller',
                   '--controller-manager', cm,
                   '--controller-type', 'diff_drive_controller/DiffDriveController',
                   '--param-file', controllers_file],
        output='screen',
    )
    torso = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_name,
        arguments=['torso_controller',
                   '--controller-manager', cm,
                   '--controller-type', 'joint_trajectory_controller/JointTrajectoryController',
                   '--param-file', controllers_file],
        output='screen',
    )

    start_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[
            TimerAction(period=2.0, actions=[jsb])
        ])
    )
    start_arm         = RegisterEventHandler(OnProcessExit(target_action=jsb,         on_exit=[arm]))
    start_gripper     = RegisterEventHandler(OnProcessExit(target_action=arm,         on_exit=[gripper]))
    start_mobile_base = RegisterEventHandler(OnProcessExit(target_action=gripper,     on_exit=[mobile_base]))
    start_torso       = RegisterEventHandler(OnProcessExit(target_action=mobile_base, on_exit=[torso]))

    return [
        rsp,
        TimerAction(period=spawn_delay, actions=[spawn]),
        start_jsb,
        start_arm,
        start_gripper,
        start_mobile_base,
        start_torso,
    ]


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',  default_value='tiago_robot1',
                              description='Unique name — Gazebo model name and ROS namespace'),
        DeclareLaunchArgument('x_pos',       default_value='0.0'),
        DeclareLaunchArgument('y_pos',       default_value='0.0'),
        DeclareLaunchArgument('z_pos',       default_value='0.0'),
        DeclareLaunchArgument('yaw',         default_value='0.0'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0'),
        OpaqueFunction(function=launch_setup),
    ])