from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    package_name = "rover_localization"

    pkg_share = get_package_share_directory(package_name)

    aruco_params = os.path.join(
        pkg_share,
        "config",
        "aruco_params.yaml"
    )

    camera_topic = LaunchConfiguration("camera_topic")
    

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            description="Camera image topic used by the ArUco tracker",
            default_value=get_parameter_from_yaml(
                aruco_params, 
                "aruco_tracker", 
                "cam_base_topic"
            ),
        ),
        Node(
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
        ),
    ])
