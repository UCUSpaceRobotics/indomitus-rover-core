# rover_sensors/launch/scan_filter.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotEqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_sensors")    
    default_filter_config = os.path.join(pkg, "config", "scan_filter.yaml")

    config_file_val = LaunchConfiguration("config_path")
    config_file_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_filter_config,
        description="Full path to the Scan Filter parameters file",
    )

    custom_config_file_present = NotEqualsSubstitution(config_file_val, "")
    config_file = IfElseSubstitution(
        custom_config_file_present,
        if_value=config_file_val,
        else_value=default_filter_config,
    )

    return LaunchDescription([
        config_file_arg,

        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="laser_filter_node",
            output="screen",
            parameters=[config_file],
            remappings=[
                ("scan", "/rplidar/scan"),
                ("scan_filtered", "/rplidar/scan_filtered"),
            ],
        ),
    ])