# LAUNCH THIS ON THE REAL ROVER

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    end_effector = LaunchConfiguration('end_effector')

    gamepad_servo_node = Node(
        package='arm_tasks',
        executable='gamepad_servo_node',
        namespace='arm',
        output='screen',
        parameters=[{
            'end_effector': end_effector,
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
        gamepad_servo_node,
    ])
