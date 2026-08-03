from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_dir = LaunchConfiguration('log_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_dir',
            default_value='~/.ros/rover_faults',
            description='Directory for the rotating fault event log',
        ),
        Node(
            package='rover_diagnostics',
            executable='fault_logger_node',
            name='fault_logger_node',
            output='screen',
            parameters=[{'log_dir': log_dir}],
        ),
    ])
