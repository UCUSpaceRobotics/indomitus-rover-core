from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

DEFAULT_ZED2I_CONFIG_NAME = "zed2i.yaml"
CAMERA_MODEL = "zed2i"


def generate_launch_description():
    child_launch_file_path = PathJoinSubstitution([
        FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"
    ])
    config_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors"), "config", DEFAULT_ZED2I_CONFIG_NAME
    ])

    config_path_argument = DeclareLaunchArgument(
        name="config_path",
        default_value=config_path,
        description="Path to the config for the ZED 2i stereo camera"
    )

    launch_file = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(child_launch_file_path),
        launch_arguments={
            "camera_model": CAMERA_MODEL,
            "camera_name": CAMERA_MODEL,
            "publish_urdf": "false",
            "ros_params_override_path": LaunchConfiguration("config_path"),
        }.items()
    )

    return LaunchDescription([
        config_path_argument,
        launch_file,
    ])
