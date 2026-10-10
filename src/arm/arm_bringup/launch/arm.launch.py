"""Real arm bringup, all under /arm: robot_state_publisher, ros2_control,
controllers, move_group, moveit_servo, the motion lock server and (real
hardware only) the CAN bridge + end-effector node.

Servo streams joint positions to indomitus_arm_forward_position_controller,
which is spawned inactive; arm_teleop switches JTC <-> forward around home /
start_servo. servo_node starts once indomitus_arm_controller is active.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils.launches import generate_move_group_launch

from arm_bringup.launch_utils import (
    ARM_CONTROLLER, END_EFFECTORS, FORWARD_CONTROLLER, build_moveit_config, include_launch,
    servo_node,
)

ARM_NAMESPACE = 'arm'


def _spawner(*arguments, **kwargs) -> Node:
    return Node(package='controller_manager', executable='spawner',
                arguments=list(arguments), output='screen', **kwargs)


def _launch_setup(context):
    fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context) == 'true'
    end_effector = LaunchConfiguration('end_effector').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)

    mappings = {
        'use_fake_hardware': 'true' if fake_hardware else 'false',
        'end_effector': end_effector,
    }
    if can_interface:
        mappings['can_interface'] = can_interface
    moveit_config = build_moveit_config(
        os.path.join(get_package_share_directory('arm_description'),
                     'urdf', 'arm_standalone.urdf.xacro'),
        mappings,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
        # tf/tf_static stay global - see docs/software/tf_ownership.md.
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('arm_moveit_config'),
                                 'config', 'ros2_controllers.yaml')],
        # URDF comes from robot_state_publisher's topic, not a parameter.
        remappings=[('~/robot_description', 'robot_description')],
    )

    arm_spawner = _spawner('joint_state_broadcaster', ARM_CONTROLLER)
    # Streaming teleop controller, inactive until arm_teleop switches to it.
    forward_spawner = _spawner(FORWARD_CONTROLLER, '--inactive')

    actions = [
        robot_state_publisher,
        ros2_control_node,
        arm_spawner,
        forward_spawner,
        *generate_move_group_launch(moveit_config).entities,
        # Lives next to controller_manager: keyboard_servo_node and
        # panel_align_node are its clients, possibly from another host.
        Node(package='arm_teleop', executable='arm_motion_lock_server', output='screen'),
        # arm_tasks is not in the Jetson image, hence opt-in.
        Node(package='arm_tasks', executable='collision_link_reporter', output='screen',
             condition=IfCondition(LaunchConfiguration('report_collisions'))),
    ]

    # Only the jaw has finger joints; the real arm only stubs the right one.
    if end_effector == 'jaw':
        fingers = ['gripper_right_controller']
        if fake_hardware:
            fingers.append('gripper_left_controller')
        actions.append(_spawner(
            *fingers,
            '--controller-manager-timeout', '60',
            '--switch-timeout', '60',
            '--service-call-timeout', '70',
        ))

    if not fake_hardware:
        # Only the end-effector cmd/ack pairs (0x1A-0x1F), not the joint IDs.
        can_args = {'receiver_filters': '1A:7FE,1C:7FE,1E:7FE'}
        if can_interface:
            can_args['interface'] = can_interface
        actions += [
            include_launch('arm_bringup', 'can.launch.py', can_args),
            include_launch('arm_peripherals', 'end_effector_can.launch.py',
                           {'end_effector': end_effector}),
        ]

    servo = servo_node(moveit_config, extra_parameters=[
        # Light smoothing only: heavy coeffs lag the command and, with high
        # MIT kp, the arm overshoots then waits (stop-go buzz).
        {'butterworth_filter_coeff': 2.0},
    ])

    def _start_servo(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f'{ARM_CONTROLLER} spawner failed '
                                f'(exit code {event.returncode}) - servo_node not started.')]
        # Event handler actions run after the namespace push was popped.
        return [GroupAction([PushRosNamespace(ARM_NAMESPACE), servo])]

    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=arm_spawner, on_exit=_start_servo)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        # Required on purpose: the operator always states whether the real
        # motors get commanded.
        DeclareLaunchArgument(
            'use_fake_hardware', choices=['true', 'false'],
            description="REQUIRED. 'true' = mock hardware, 'false' = real CAN bus",
        ),
        DeclareLaunchArgument(
            'end_effector', default_value='jaw', choices=END_EFFECTORS,
            description='Tool mounted on the arm - see arm_macro.xacro',
        ),
        DeclareLaunchArgument(
            'can_interface', default_value='',
            description="SocketCAN interface for joints + end effector; empty = each "
                        "component's own default",
        ),
        DeclareLaunchArgument(
            'report_collisions', default_value='false',
            description='Run arm_tasks collision_link_reporter (logs the colliding link '
                        'pair). Needs arm_tasks built - not in the Jetson image.',
        ),
        DeclareLaunchArgument(
            'bring_up_gamepad', default_value='true',
            description='Also start gamepad_servo_node (rover-side half of gamepad teleop; '
                        'the joystick side is arm_teleop gamepad_joy.launch.py)',
        ),
        DeclareLaunchArgument(
            'home_pose_name', default_value='',
            description="poses.json key gamepad A drives to; empty = '<end_effector>_home' "
                        "if it exists, else 'home'",
        ),
        DeclareLaunchArgument(
            'activity_indicator_pre_delay_sec', default_value='0.0',
            description='Seconds planned moves wait before moving. ERC rules '
                        '(REQ-OPS-080/090/100) require >= 5.0 at the competition.',
        ),

        GroupAction([
            PushRosNamespace(ARM_NAMESPACE),
            OpaqueFunction(function=_launch_setup),
        ]),

        # Outside the /arm push: gamepad_servo.launch.py sets namespace='arm'
        # itself (inside the push it would become /arm/arm/...).
        include_launch('arm_teleop', 'gamepad_servo.launch.py', {
            'end_effector': LaunchConfiguration('end_effector'),
            'home_pose_name': LaunchConfiguration('home_pose_name'),
            'activity_indicator_pre_delay_sec':
                LaunchConfiguration('activity_indicator_pre_delay_sec'),
        }, condition=IfCondition(LaunchConfiguration('bring_up_gamepad'))),
    ])
