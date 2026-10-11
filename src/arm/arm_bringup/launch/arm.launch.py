"""Real arm, all under /arm: ros2_control_node (CAN or mock hardware), the
CAN bridge + end-effector node (real hardware only), the arm stack
(stack.launch.py) and the on-rover half of gamepad teleop.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from arm_bringup.launch_utils import END_EFFECTORS, include_launch

ARM_NAMESPACE = 'arm'


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    end_effector = LaunchConfiguration('end_effector')
    can_interface = LaunchConfiguration('can_interface')
    real_hardware = UnlessCondition(use_fake_hardware)

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
            'can_interface', default_value='can0',
            description='SocketCAN interface for the joints and the end effector',
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

            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('arm_moveit_config'),
                                         'config', 'ros2_controllers.yaml')],
                # URDF comes from the stack's robot_state_publisher topic.
                remappings=[('~/robot_description', 'robot_description')],
            ),

            # Only the end-effector cmd/ack pairs (0x1A-0x1F), not the joint IDs.
            include_launch('arm_bringup', 'can.launch.py', {
                'interface': can_interface,
                'receiver_filters': '1A:7FE,1C:7FE,1E:7FE',
            }, condition=real_hardware),
            include_launch('arm_peripherals', 'end_effector_can.launch.py', {
                'end_effector': end_effector,
            }, condition=real_hardware),

            include_launch('arm_bringup', 'stack.launch.py', {
                'use_sim': 'false',
                'use_fake_hardware': use_fake_hardware,
                'can_interface': can_interface,
                'end_effector': end_effector,
                'report_collisions': LaunchConfiguration('report_collisions'),
            }),
        ]),

        # Outside the /arm push: gamepad_servo.launch.py sets namespace='arm'
        # itself (inside the push it would become /arm/arm/...).
        include_launch('arm_teleop', 'gamepad_servo.launch.py', {
            'end_effector': end_effector,
            'home_pose_name': LaunchConfiguration('home_pose_name'),
            'activity_indicator_pre_delay_sec':
                LaunchConfiguration('activity_indicator_pre_delay_sec'),
        }, condition=IfCondition(LaunchConfiguration('bring_up_gamepad'))),
    ])
