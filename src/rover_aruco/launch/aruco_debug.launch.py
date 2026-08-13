from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def get_parameter_from_yaml(file_path, node_name, parameter_name):
    with open(file_path, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    try:
        return str(
            config[node_name]["ros__parameters"][parameter_name]
        )
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            f"Parameter '{parameter_name}' for node '{node_name}' "
            f"was not found in '{file_path}'"
        ) from error


def generate_launch_description():
    package_name = "rover_aruco"

    pkg_share = get_package_share_directory(package_name)

    usb_cam_params = os.path.join(
        pkg_share,
        "config",
        "usb_cam_params.yaml"
    )

    aruco_params = os.path.join(
        pkg_share,
        "config",
        "aruco_debug_params.yaml"
    )

    camera_info_file = os.path.join(
        pkg_share,
        "config",
        "approx_laptop_camera.yaml"
    )

    camera_info_url = "file://" + camera_info_file

    default_video_device = get_parameter_from_yaml(
        usb_cam_params, "/**", "video_device"
    )
    video_device = LaunchConfiguration(
        "video_device",
    )

    camera_topic = LaunchConfiguration("camera_topic")

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
                "video_device": video_device,
            }
        ],
    )

    aruco_node = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[
            aruco_params,
            {
                "cam_base_topic": camera_topic,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "video_device",
            description="Webcam device used by usb_cam_node",
            default_value=default_video_device
        ),
        DeclareLaunchArgument(
            "camera_topic",
            description="Camera image topic used by the ArUco tracker",
            default_value=get_parameter_from_yaml(
                aruco_params,
                "aruco_tracker",
                "cam_base_topic"
            ),
        ),
        usb_cam_node,
        aruco_node,
    ])
