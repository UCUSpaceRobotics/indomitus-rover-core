from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    zed2i_launch_file_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors", "launch", "zed2i.launch.py")
    ])
    navigation_launch_file_path = PathJoinSubstitution([
        FindPackageShare("rover_navigation", "launch", "navigation_test.launch.py")
    ])

    zed2i_config_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors"), "config", "zed2i_test.yaml"
    ])
    nav2_config_path = PathJoinSubstitution([
        FindPackageShare("rover_navigation"), "config", "nav2_params_test.yaml"
    ])

    zed2i_config_argument = DeclareLaunchArgument(
        name="zed2i_config",
        default_value=zed2i_config_path,
        description="Path to the config for the ZED 2i stereo camera"
    )
    nav2_config_argument = DeclareLaunchArgument(
        name="nav2_config",
        default_value=nav2_config_path,
        description="Path to the config for the nav2"
    )


    zed2i_launch_file = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(zed2i_launch_file_path),
        launch_arguments={
            "config_path": LaunchConfiguration("zed2i_config"),
        }.items()
    )
    nav2_launch_file = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(navigation_launch_file_path),
        launch_arguments={
            "use_sim_time": "false",
            "config_path": LaunchConfiguration("nav2_config"),
        }.items()
    )

    return LaunchDescription([
        zed2i_config_argument,
        nav2_config_argument,
        zed2i_launch_file,
        nav2_launch_file,
    ])