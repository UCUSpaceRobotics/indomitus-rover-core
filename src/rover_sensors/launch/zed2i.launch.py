from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

DEFAULT_COMMON_STEREO_CONFIG_NAME = "common_stereo.yaml"
DEFAULT_CAMERA_CONFIG_NAME = "zed2i.yaml"
CAMERA_MODEL = "zed2i"


def generate_launch_description():
    child_launch_file_path = PathJoinSubstitution([
        FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"
    ])
    config_common_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors"), "config", DEFAULT_COMMON_STEREO_CONFIG_NAME
    ])
    config_camera_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors"), "config", DEFAULT_CAMERA_CONFIG_NAME
    ])


    config_common_path_argument = DeclareLaunchArgument(
        name="config_common_path",
        default_value=config_common_path,
        description="Path to the common stereo config for ZED stereocamera"
    )
    config_camera_path_argument = DeclareLaunchArgument(
        name="config_camera_path",
        default_value=config_camera_path,
        description="Path to the config for the ZED 2i stereo camera"
    )


    launch_file = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(child_launch_file_path),
        launch_arguments={
            "camera_model": CAMERA_MODEL,
            "config_common_path": LaunchConfiguration("config_common_path"),
            "config_camera_path": LaunchConfiguration("config_camera_path"),
        }.items()
    )

    return LaunchDescription([
        config_common_path_argument,
        config_camera_path_argument,
        launch_file,
    ])
