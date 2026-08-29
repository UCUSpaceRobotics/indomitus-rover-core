# LAUNCH THIS ON THE REAL ROVER

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    end_effector = LaunchConfiguration('end_effector')
    gripper_can_iface = LaunchConfiguration('gripper_can_iface')

    gamepad_servo_node = Node(
        package='arm_tasks',
        executable='gamepad_servo_node',
        output='screen',
        parameters=[{
            'end_effector': end_effector,
            'gripper_can_iface': gripper_can_iface,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector',
            default_value='jaw',
            description=(
                "Which tool is physically mounted right now: 'jaw', "
                "'drill_sampling', or 'astrobio'. Gates the A/B/Y mode-jump "
                'buttons in gamepad_servo_node — match this to what you '
                'actually launched the arm with (demo.launch.py '
                'end_effector:=...).'
            ),
        ),
        DeclareLaunchArgument(
            'gripper_can_iface',
            default_value='can0',
            description=(
                'SocketCAN interface the SAFE gripper firmware (jaw tool) '
                "is on. 'can0' for real hardware, 'vcan0' for sim/bench "
                'testing. Requires the CAN interface to actually be present '
                'on this machine — run this launch file on the rover.'
            ),
        ),
        gamepad_servo_node,
    ])
