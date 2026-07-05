from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = "rover_localization"

    pkg_share = get_package_share_directory(package_name)

    aruco_params = os.path.join(
        pkg_share,
        "config",
        "aruco_params.yaml"
    )

    aruco_node = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[aruco_params],
    )

    return LaunchDescription([
        aruco_node,
    ])
