"""The arm's ROS stack: robot_state_publisher, controller spawners,
move_group, moveit_servo and the motion lock server.

Hardware-agnostic: the same file runs on the real arm (arm.launch.py, next to
ros2_control_node) and in Gazebo (arm_sim/arm_gz.launch.py and
indomitus_bringup/sim_rover_arm.launch.py, where controller_manager lives in
the gz_ros2_control plugin). Pushes no namespace; entry points push /arm.

Servo streams joint positions to indomitus_arm_forward_position_controller,
which is spawned inactive; arm_teleop switches JTC <-> forward around home /
start_servo. move_group, servo and the gripper spawner start once
indomitus_arm_controller is spawned.

arm_active_at_start:=false is for hardware that starts INACTIVE (the real arm:
motors read, no torque): indomitus_arm_controller is then spawned inactive too,
and arm_teleop's Activate switches the hardware and the controller on.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from moveit_configs_utils.launches import generate_move_group_launch

from arm_bringup.launch_utils import (
    ARM_CONTROLLER, END_EFFECTORS, FORWARD_CONTROLLER, build_moveit_config, servo_node,
)


def _spawner(*arguments) -> Node:
    # No --controller-manager-timeout: in sim controller_manager only appears
    # once the entry point has spawned the model.
    return Node(package='controller_manager', executable='spawner', output='screen',
                arguments=[*arguments, '--switch-timeout', '60', '--service-call-timeout', '70'])


ARM_JOINTS = [
    'arm_mount_base_joint', 'arm_base_shoulder_joint', 'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint', 'arm_wrist_1_wrist_2_joint', 'arm_wrist_2_end_effector_joint',
]
# ArmCanSystem's gain_ramp_secs (arm_macro.xacro): motion lags until it is over.
REAL_ARM_STIFFNESS_RAMP_SECS = 15.0


def _park_pose() -> list:
    """poses.json "park" as six joint values, or [] when not recorded yet."""
    path = os.path.join(get_package_share_directory('arm_teleop'), 'poses.json')
    try:
        with open(path) as f:
            park = json.load(f).get('park') or {}
        return [float(park[name]) for name in ARM_JOINTS]
    except (OSError, KeyError, TypeError, ValueError):
        return []


def _launch_setup(context):
    use_sim = LaunchConfiguration('use_sim').perform(context) == 'true'
    fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context) == 'true'
    end_effector = LaunchConfiguration('end_effector').perform(context)
    namespace = context.launch_configurations.get('ros_namespace', '')
    active_at_start = LaunchConfiguration('arm_active_at_start').perform(context) == 'true'

    moveit_config = build_moveit_config(
        LaunchConfiguration('description_file').perform(context), {
            'sim': 'true' if use_sim else 'false',
            'camera': LaunchConfiguration('camera').perform(context),
            'use_fake_hardware': 'true' if fake_hardware else 'false',
            'can_interface': LaunchConfiguration('can_interface').perform(context),
            'end_effector': end_effector,
        })

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
        # tf/tf_static stay global - see docs/software/tf_ownership.md.
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
    )

    # Separate spawners: joint_state_broadcaster only reads, so it runs even
    # while the hardware is inactive; a controller with command interfaces
    # cannot be activated on inactive hardware.
    jsb_spawner = _spawner('joint_state_broadcaster')
    arm_spawner = _spawner(ARM_CONTROLLER, *([] if active_at_start else ['--inactive']))

    started_after_arm = [
        *generate_move_group_launch(moveit_config).entities,
        servo_node(moveit_config, extra_parameters=[
            # Light smoothing only: heavy coeffs lag the command and, with
            # high MIT kp, the arm overshoots then waits (stop-go buzz).
            {'butterworth_filter_coeff': 2.0},
        ]),
    ]
    # Only the jaw has finger joints; the real arm only stubs the right one.
    if end_effector == 'jaw':
        fingers = ['gripper_right_controller']
        if use_sim or fake_hardware:
            fingers.append('gripper_left_controller')
        # On mock / sim the fingers belong to the arm's own hardware component,
        # so they cannot be activated while it is inactive either.
        started_after_arm.append(_spawner(*fingers, *([] if active_at_start else ['--inactive'])))

    def _after_arm_controller(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'{ARM_CONTROLLER} spawner failed (exit code {event.returncode}).'),
                Shutdown(reason='arm controller activation failed'),
            ]
        # Event handler actions run after the namespace push and the
        # use_sim_time parameter scope were popped, so re-apply both.
        return [GroupAction([
            *([PushRosNamespace(namespace)] if namespace else []),
            SetParameter(name='use_sim_time', value=use_sim),
            *started_after_arm,
        ])]

    return [
        SetParameter(name='use_sim_time', value=use_sim),
        robot_state_publisher,
        jsb_spawner,
        arm_spawner,
        # Streaming teleop controller, inactive until arm_teleop switches to it.
        _spawner(FORWARD_CONTROLLER, '--inactive'),
        # keyboard_servo_node and panel_align_node are its clients, possibly
        # from another host.
        Node(package='arm_teleop', executable='arm_motion_lock_server', output='screen'),
        # Owner of the motors' power: /arm/power, /arm/state (see arm_power).
        Node(package='arm_power', executable='arm_power_node', name='arm_power', output='screen',
             parameters=[{
                 'stiffness_ramp_secs':
                     0.0 if (use_sim or fake_hardware) else REAL_ARM_STIFFNESS_RAMP_SECS,
                 # [] would not type as a double array, hence the 1-element filler
                 # arm_power reads as "no park pose".
                 'park_pose': _park_pose() or [0.0],
             }]),
        # arm_tasks is not in the Jetson image, hence opt-in.
        Node(package='arm_tasks', executable='collision_link_reporter', output='screen',
             condition=IfCondition(LaunchConfiguration('report_collisions'))),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner, on_exit=_after_arm_controller)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'description_file',
            default_value=os.path.join(get_package_share_directory('arm_description'),
                                       'urdf', 'arm_standalone.urdf.xacro'),
            description='Arm xacro: arm_standalone.urdf.xacro (stand, root world) or '
                        'indomitus_description arm_on_rover.urdf.xacro (root main_body_link)',
        ),
        DeclareLaunchArgument(
            'use_sim', default_value='false', choices=['true', 'false'],
            description='Gazebo: gz_ros2_control in the URDF + use_sim_time',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='true', choices=['true', 'false'],
            description='Hardware only: mock components instead of the CAN bus',
        ),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument(
            'arm_active_at_start', default_value='true', choices=['true', 'false'],
            description="'false' when the arm hardware starts inactive (see the docstring)",
        ),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Sim only: simulate the wrist camera'),
        DeclareLaunchArgument(
            'end_effector', default_value='jaw', choices=END_EFFECTORS,
            description='Tool mounted on the arm - see arm_macro.xacro',
        ),
        DeclareLaunchArgument(
            'report_collisions', default_value='false',
            description='Run arm_tasks collision_link_reporter (logs the colliding link '
                        'pair). Needs arm_tasks built - not in the Jetson image.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
