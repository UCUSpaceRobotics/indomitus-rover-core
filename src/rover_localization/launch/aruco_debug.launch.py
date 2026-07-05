from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = "rover_localization"

    pkg_share = get_package_share_directory(package_name)

    usb_cam_params = os.path.join(
        pkg_share,
        "config",
        "usb_cam_params.yaml"
    )

    aruco_params = os.path.join(
        pkg_share,
        "config",
        "aruco_params.yaml"
    )

    camera_info_file = os.path.join(
        pkg_share,
        "config",
        "approx_laptop_camera.yaml"
    )

    camera_info_url = "file://" + camera_info_file

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="camera",
        name="usb_cam",
        output="screen",
        parameters=[
            usb_cam_params,
            {
                "camera_info_url": camera_info_url,
            }
        ],
    )

    aruco_node = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[aruco_params],
    )

    return LaunchDescription([
        usb_cam_node,
        aruco_node,
    ])
