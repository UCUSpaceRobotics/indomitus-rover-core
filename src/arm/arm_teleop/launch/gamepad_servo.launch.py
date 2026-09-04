# LAUNCH THIS ON THE REAL ROVER

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    end_effector = LaunchConfiguration('end_effector')
    home_pose_name = LaunchConfiguration('home_pose_name')

    gamepad_servo_node = Node(
        package='arm_teleop',
        executable='gamepad_servo_node',
        namespace='arm',
        output='screen',
        parameters=[{
            'end_effector': end_effector,
            'home_pose_name': home_pose_name,
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
                'actually launched the arm with (arm.launch.py '
                'end_effector:=...).'
            ),
        ),
        DeclareLaunchArgument(
            'home_pose_name',
            default_value='',
            description=(
                'poses.json key that A drives to when not in sampling/drill '
                "mode. Leave empty (default) to auto-pick '{end_effector}_home' "
                "(e.g. 'jaw_home') when that key exists in poses.json, else "
                "fall back to 'home' — see ServoController.__init__. Set "
                'explicitly only to override that auto-pick.'
            ),
        ),
        gamepad_servo_node,
    ])
