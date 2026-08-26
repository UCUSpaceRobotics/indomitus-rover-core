import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotEqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("rover_sensors"), "config", "arducam.yaml"
    )

    camera_path_arg = LaunchConfiguration("camera_path")
    camera_info_url_arg = LaunchConfiguration("camera_info_url")
    camera_frame_id_arg = LaunchConfiguration("camera_frame_id")
    config_path_val = LaunchConfiguration("config_path")
    namespace_arg = LaunchConfiguration("namespace")

    declare_camera_path = DeclareLaunchArgument(
        "camera_path",
        default_value="/dev/video0",
        description="Path to the camera device (e.g., /dev/video0 or /dev/camera_left)"
    )

    declare_camera_info_url = DeclareLaunchArgument(
        "camera_info_url",
        default_value="",
        description="URL to the camera calibration file (e.g., file:///path/to/cal.yaml)"
    )

    declare_camera_frame_id = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="camera",
        description="TF frame ID attached to the published image headers "
                     "(e.g., rear_arducam_optical_frame or mast_arducam_optical_frame)"
    )

    declare_config_path = DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="Full path to the YAML configuration file with camera parameters. "
                     "Defaults to rover_sensors' shared arducam.yaml when not set."
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="camera",
        description="Namespace for the camera node"
    )

    custom_config_path_present = NotEqualsSubstitution(config_path_val, "")
    config_file = IfElseSubstitution(
        custom_config_path_present,
        if_value=config_path_val,
        else_value=default_config_file,
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="arducam_node",
        namespace=namespace_arg,
        output="screen",
        parameters=[
            config_file,
            {
                "video_device": camera_path_arg,
                "camera_info_url": camera_info_url_arg,
                "camera_frame_id": camera_frame_id_arg,
            }
        ]
    )

    return LaunchDescription([
        declare_camera_path,
        declare_camera_info_url,
        declare_camera_frame_id,
        declare_config_path,
        declare_namespace,
        v4l2_camera_node
    ])
