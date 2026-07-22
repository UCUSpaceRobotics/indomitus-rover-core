from typing import List
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from rover_bringup.launch_utils import include_launch


def _declare_launch_arguments() -> List[Action]:
    return [
        DeclareLaunchArgument(
            "world_name", 
            default_value="nav2_test_world", 
            description="Name of the Gazebo world file to load (without the .sdf extension).",
        ),
        DeclareLaunchArgument(
            "model_name", 
            default_value="indomitus_rover", 
            description="The name assigned to the robot model when spawned inside the Gazebo environment.",
        ),
        DeclareLaunchArgument(
            "spawn_x", 
            default_value="0.0", 
            description="Initial X coordinate (in meters) for spawning the robot in the global world frame.",
        ),
        DeclareLaunchArgument(
            "spawn_y", 
            default_value="0.0", 
            description="Initial Y coordinate (in meters) for spawning the robot in the global world frame.",
        ),
        DeclareLaunchArgument(
            "spawn_z", 
            default_value="1.0", 
            description="Initial Z coordinate (in meters) for spawning the robot in the global world frame. Set higher than ground level.",
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
    ]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = _declare_launch_arguments()

    scan_filter_params_file_val = LaunchConfiguration("scan_filter_params_file")
    nav2_params_file_val = LaunchConfiguration("nav2_params_file")
    slam_params_file_val = LaunchConfiguration("slam_params_file")

    return LaunchDescription([
        *launch_arguments,

        include_launch("rover_sim", "sim_gz.launch.py", {
            "world_name": LaunchConfiguration("world_name"),
            "model_name": LaunchConfiguration("model_name"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "extra_xacro_args": "use_nav:=true lidar_simulate_scan:=true stereo_camera_simulate_depth:=true",
        }),

        TimerAction(
            period=10.0,
            actions=[
                include_launch("rover_sensors", "scan_filter.launch.py", {
                    "config_path": scan_filter_params_file_val
                }),

                include_launch("rover_localization", "slam.launch.py", {
                    "use_sim_time": "true",
                    "config_path": slam_params_file_val,
                }),

                include_launch("rover_navigation", "nav2.launch.py", {
                    "use_sim": "true",
                    "cmd_vel_topic": "cmd_vel_nav",
                    "params_file": nav2_params_file_val,
                }),
            ],
        ),
    ])