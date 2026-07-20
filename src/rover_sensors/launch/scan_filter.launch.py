# rover_sensors/launch/scan_filter.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("rover_sensors")    
    filter_config = os.path.join(pkg, "config", "scan_filter.yaml")

    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="laser_filter_node",
            output="screen",
            parameters=[filter_config],
            remappings=[
                ("scan", "/rplidar/scan"),
                ("scan_filtered", "/rplidar/scan_filtered"),
            ],
        )
    ])