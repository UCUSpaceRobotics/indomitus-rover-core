from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('xacro_file'),
        DeclareLaunchArgument('xacro_args', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', LaunchConfiguration('xacro_file'), ' ', LaunchConfiguration('xacro_args'),
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
