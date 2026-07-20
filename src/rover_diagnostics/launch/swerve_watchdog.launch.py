from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_angle_jump_deg_arg = DeclareLaunchArgument(
        'max_angle_jump_deg', default_value='15.0',
        description='Max allowed steering joint angle change per message, in degrees'
    )
    max_velocity_jump_arg = DeclareLaunchArgument(
        'max_velocity_jump', default_value='2.0',
        description='Max allowed drive joint velocity change per message, in rad/s'
    )

    watchdog_node = Node(
        package='rover_diagnostics',
        executable='swerve_jump_watchdog',
        name='swerve_jump_watchdog',
        output='screen',
        parameters=[{
            'max_angle_jump_deg': LaunchConfiguration('max_angle_jump_deg'),
            'max_velocity_jump': LaunchConfiguration('max_velocity_jump'),
        }],
    )

    return LaunchDescription([
        max_angle_jump_deg_arg,
        max_velocity_jump_arg,
        watchdog_node,
    ])