#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotEqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("rover_sensors"), "config", "rplidar_s2.yaml"
    )

    config_file_val = LaunchConfiguration("config_path")
    config_file_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_file,
        description="Full path to the RPLIDAR S2 parameters file",
    )

    namespace_val = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="rplidar",
        description="Namespace to prevent topic collisions",
    )

    custom_config_file_present = NotEqualsSubstitution(config_file_val, "")
    config_file = IfElseSubstitution(
        custom_config_file_present,
        if_value=config_file_val,
        else_value=default_config_file,
    )

    return LaunchDescription([
        namespace_arg,
        config_file_arg,
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="rplidar_node",
            namespace=namespace_val,
            parameters=[config_file],
            output="screen",
            respawn=True,       # Crucial: Restarts node if USB cable wiggles loose
            respawn_delay=2.0,
        ),
    ])
