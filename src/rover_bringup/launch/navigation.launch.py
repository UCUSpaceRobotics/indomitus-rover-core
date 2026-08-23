# rover_bringup/launch/navigation.launch.py

from typing import List, Union

from launch import LaunchDescription, Action, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from rover_bringup.launch_utils import include_launch


def _declare_launch_arguments() -> List[Action]:
    return [
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


def _wait_for_topic_cmd(
    topic: str, waiting_message: str, interval: Union[Substitution, str],
) -> List:
    """
    Block until `topic` actually publishes a message, logging `waiting_message` every
    `interval` seconds while it waits.

    Uses `ros2 topic echo --once`, which blocks for a real message rather than just a
    registered publisher (unlike `ros2 topic info`) - both topics used below are small
    (LaserScan, CameraInfo), so this never pays to deserialize a heavy payload like
    `/zed2i/points` would.
    """
    return [
        "/bin/bash", "-c",
        [
            f'until ros2 topic echo --once {topic} > /dev/null 2>&1; do '
            f'echo "{waiting_message}"; sleep ',
            interval,
            "; done",
        ],
    ]


def generate_launch_description() -> LaunchDescription:
    launch_arguments = _declare_launch_arguments()

    rplidar_params_file_val = LaunchConfiguration("rplidar_params_file")
    scan_filter_params_file_val = LaunchConfiguration("scan_filter_params_file")
    nav2_params_file_val = LaunchConfiguration("nav2_params_file")
    slam_params_file_val = LaunchConfiguration("slam_params_file")

    lidar_launch = include_launch("rover_sensors", "rplidar_s2.launch.py", {
        "config_path": rplidar_params_file_val,
    })

    scan_filter_launch = include_launch("rover_sensors", "scan_filter.launch.py", {
        "config_path": scan_filter_params_file_val
    })

    lidar_reminder = LogInfo(
        msg="[navigation] Starting the RPLIDAR + scan filter pipeline. Waiting for scans "
            "below."
    )

    # This launch file no longer starts the ZED2i camera itself (see rover.launch.py's
    # `zed2i_mode` argument) - it only waits for the topics the camera produces in 'nav' mode.
    zed2i_reminder = LogInfo(
        msg="[navigation] Expecting the ZED2i camera to already be running in 'nav' mode "
            "(rover.launch.py zed2i_mode:=nav). Waiting for its topics below."
    )

    wait_for_lidar = ExecuteProcess(
        cmd=_wait_for_topic_cmd(
            "/rplidar/scan_filtered",
            "[navigation] Waiting for RPLIDAR scan filter (topic /rplidar/scan_filtered)...",
            "3",  # seconds between messages
        ),
        output="screen",
    )

    # `/zed2i/depth/camera_info` (not `/zed2i/imu/data`, which publishes in both modes,
    # and not the much heavier `/zed2i/points`) is only published when the camera is up
    # in 'nav' mode, from the same depth pipeline stage that feeds nav2's local costmap
    # voxel_layer (see rover_navigation/config/nav2_params.yaml) - so it's a light-weight
    # stand-in confirming that pipeline is genuinely producing data, not just advertised.
    wait_for_stereo_camera = ExecuteProcess(
        cmd=_wait_for_topic_cmd(
            "/zed2i/depth/camera_info",
            "[navigation] Waiting for ZED2i depth pipeline (topic /zed2i/depth/camera_info) "
            "- is rover.launch.py running with zed2i_mode:=nav?",
            "3",  # seconds between messages
        ),
        output="screen",
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

        lidar_launch,
        scan_filter_launch,

        lidar_reminder,
        zed2i_reminder,
        wait_for_lidar,
        start_camera_wait,
        start_nav_and_slam,
    ])
