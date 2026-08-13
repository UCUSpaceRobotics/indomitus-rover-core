# rover_bringup/launch/navigation.launch.py

from typing import List

from launch import LaunchDescription, Action
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from rover_bringup.launch_utils import include_launch


def _declare_launch_arguments() -> List[Action]:
    return [
        DeclareLaunchArgument(
            "zed2i_params_file",
            default_value="",
            description="Full path to the ZED2i parameters file to override default params",
        ),
        DeclareLaunchArgument(
            "rplidar_params_file",
            default_value="",
            description="Full path to the RPLIDAR S2 parameters file to override default params",
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

    zed2i_params_file_val = LaunchConfiguration("zed2i_params_file")
    rplidar_params_file_val = LaunchConfiguration("rplidar_params_file")
    scan_filter_params_file_val = LaunchConfiguration("scan_filter_params_file")
    nav2_params_file_val = LaunchConfiguration("nav2_params_file")
    slam_params_file_val = LaunchConfiguration("slam_params_file")

    stereo_camera_launch = include_launch("rover_sensors", "zed2i.launch.py", {
        "config_path": zed2i_params_file_val,
    })

    lidar_launch = include_launch("rover_sensors", "rplidar_s2.launch.py", {
        "config_path": rplidar_params_file_val,
    })

    scan_filter_launch = include_launch("rover_sensors", "scan_filter.launch.py", {
        "config_path": scan_filter_params_file_val
    })

    wait_for_lidar = ExecuteProcess(
        cmd=["/bin/bash", "-c", "until ros2 topic echo --once /rplidar/scan_filtered > /dev/null 2>&1; do sleep 1; done"],
        output="log"
    )

    wait_for_stereo_camera = ExecuteProcess(
        cmd=["/bin/bash", "-c", "until ros2 topic echo --once /zed2i/imu/data > /dev/null 2>&1; do sleep 1; done"],
        output="log"
    )

    start_nav_and_slam = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_stereo_camera,
            on_exit=[
                LogInfo(msg="Sensors are online, launching SLAM and Navigation..."),

                include_launch("rover_localization", "slam.launch.py", {
                    "use_sim_time": "false",
                    "config_path": slam_params_file_val,
                }),

                include_launch("rover_navigation", "nav2.launch.py", {
                    "use_sim": "false",
                    "cmd_vel_topic": "cmd_vel_nav",
                    "params_file": nav2_params_file_val,
                }),
            ]
        )
    )

    start_camera_wait = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_lidar,
            on_exit=[wait_for_stereo_camera]
        )
    )

    return LaunchDescription([
        *launch_arguments,

        stereo_camera_launch,
        lidar_launch,
        scan_filter_launch,

        wait_for_lidar,
        start_camera_wait,
        start_nav_and_slam,
    ])
