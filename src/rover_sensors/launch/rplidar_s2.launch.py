#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('rover_sensors'), 'config', 'rplidar_s2.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to RPLIDAR S2 params'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='rplidar',
            description='Namespace to prevent topic collisions'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            respawn=True,       # Crucial: Restarts node if USB cable wiggles loose
            respawn_delay=2.0
        ),
    ])
