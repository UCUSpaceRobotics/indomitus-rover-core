from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    container_can_config = os.path.join(
        get_package_share_directory("indomitus_rover_bringup"),
        "config", "container_can.yaml"
    )

    return LaunchDescription([
        Node(
            package="indomitus_rover_peripherals",
            executable="rover_container_node",
            parameters=[container_can_config],
        ),
    ])