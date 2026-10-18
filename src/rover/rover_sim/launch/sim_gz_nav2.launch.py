from typing import List
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from rover_bringup.launch_utils import include_launch

# List of available worlds.
SUPPORTED_WORLDS = ["mars_yard", "nav2_test_world"]

# List of worlds that utilize the dynamic resolution map loading feature.
# For any world not in this list, the map_resolution parameter is safely ignored.
SUPPORTED_RESOLUTION_WORLDS = ["mars_yard"]

# List of available mars_yard map years.
SUPPORTED_MAP_YEARS = ["2025", "2026"]

# List of available map resolutions. Only applies to the 2025 mars_yard map.
SUPPORTED_MAP_RESOLUTIONS = ["low", "medium", "high"]


def _declare_launch_arguments() -> List[Action]:
    return [
        DeclareLaunchArgument(
            "world_name",
            default_value="",
            choices=["", *SUPPORTED_WORLDS],
            description=f"Gazebo world file to load (without .sdf extension). Available options: {SUPPORTED_WORLDS}."
        ),
        DeclareLaunchArgument(
            "map_year",
            default_value="",
            choices=["", *SUPPORTED_MAP_YEARS],
            description=f"The year of the map to load. Options: {SUPPORTED_MAP_YEARS}. Dynamically applies only to: mars_yard."
        ),
        DeclareLaunchArgument(
            "map_resolution",
            default_value="",
            choices=["", *SUPPORTED_MAP_RESOLUTIONS],
            description=f"Options: {SUPPORTED_MAP_RESOLUTIONS}. Dynamically applies only to: {SUPPORTED_RESOLUTION_WORLDS}. Ignored for other worlds."
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="",
            description="The name assigned to the robot model when spawned inside the Gazebo environment.",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="",
            description="Initial X coordinate (in meters) for spawning the robot in the global world frame.",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="",
            description="Initial Y coordinate (in meters) for spawning the robot in the global world frame.",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="",
            description="Initial Z coordinate (in meters) for spawning the robot in the global world frame.",
        ),
        DeclareLaunchArgument(
            "scan_filter_params_file",
            default_value="",
            description="Full path to the Scan Filter parameters file to override default params",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value="",
            description="Full path to the Nav2 parameters file to override default params",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value="",
            description="Full path to the SLAM parameters file to override default params",
        ),
        DeclareLaunchArgument(
            "rover_namespace",
            default_value=EnvironmentVariable("ROVER_NAMESPACE", default_value="rover"),
            description="ROS namespace all rover nodes/topics are pushed under (arm excluded).",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = _declare_launch_arguments()

    namespace_val = LaunchConfiguration("rover_namespace")
    scan_filter_params_file_val = LaunchConfiguration("scan_filter_params_file")
    nav2_params_file_val = LaunchConfiguration("nav2_params_file")
    slam_params_file_val = LaunchConfiguration("slam_params_file")

    return LaunchDescription([
        *launch_arguments,

        # sim_gz.launch.py pushes `namespace` around its own nodes itself
        # (it's also runnable standalone) - it's only threaded through here,
        # not re-wrapped, so the namespace isn't pushed twice.
        include_launch("rover_sim", "sim_gz.launch.py", {
            "world_name": LaunchConfiguration("world_name"),
            "map_year": LaunchConfiguration("map_year"),
            "map_resolution": LaunchConfiguration("map_resolution"),
            "model_name": LaunchConfiguration("model_name"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "extra_xacro_args": "lidar_simulate_scan:=true stereo_camera_simulate_depth:=true",
            "rover_namespace": namespace_val,
        }),

        TimerAction(
            period=10.0,
            actions=[
                GroupAction([
                    PushRosNamespace(namespace_val),

                    include_launch("rover_sensors", "scan_filter.launch.py", {
                        "config_path": scan_filter_params_file_val
                    }),

                    include_launch("rover_localization", "slam.launch.py", {
                        "use_sim_time": "true",
                        "config_path": slam_params_file_val,
                    }),

                    include_launch("rover_navigation", "nav2.launch.py", {
                        "use_sim_time": "true",
                        "cmd_vel_topic": "cmd_vel_nav",
                        "params_file": nav2_params_file_val,
                    }),
                ]),
            ],
        ),
    ])