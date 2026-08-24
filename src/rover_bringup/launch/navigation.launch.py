# rover_bringup/launch/navigation.launch.py

from typing import Callable, List, Optional, Union

from launch import LaunchContext, LaunchDescription, Action, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
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
    topics: Union[str, List[str]], waiting_message: str, interval: Union[Substitution, str],
) -> List:
    """
    Block until every topic in `topics` actually publishes a message, logging
    `waiting_message` every `interval` seconds while it waits.
    """
    if isinstance(topics, str):
        topics = [topics]

    check_parts: List[Union[str, Substitution]] = []
    for i, topic in enumerate(topics):
        if i > 0:
            check_parts.append(" && ")
        check_parts += [
            "timeout ",
            interval,
            f" ros2 topic echo --once --no-arr {topic} > /dev/null 2>&1",
        ]

    return [
        "/bin/bash", "-c",
        [
            "trap 'exit 130' INT TERM; until ",
            *check_parts,
            f'; do echo "{waiting_message}"; sleep ',
            interval,
            "; done",
        ],
    ]


def _on_success(
    description: str, next_actions: List[Action],
) -> Callable[[ProcessExited, LaunchContext], Optional[List[Action]]]:
    """
    Build an `OnProcessExit.on_exit` callback that only runs `next_actions` if the process
    exited with code 0. Without this, a wait process that gets killed or fails still fires
    `on_exit`, which would let SLAM/Nav2 start without confirmed sensor data.
    """
    def _handle(event: ProcessExited, context: LaunchContext) -> Optional[List[Action]]:
        if event.returncode == 0:
            return next_actions
        return [
            LogInfo(
                msg=f"[navigation] {description} exited with code {event.returncode} "
                    "instead of 0 - aborting startup of downstream nodes."
            )
        ]
    return _handle


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

    wait_for_stereo_camera = ExecuteProcess(
        cmd=_wait_for_topic_cmd(
            ["/zed2i/points", "/zed2i/odom"],
            "[navigation] Waiting for ZED2i topics (/zed2i/points, /zed2i/odom) "
            "- is rover.launch.py running with zed2i_mode:=nav?",
            "3",  # seconds between messages
        ),
        output="screen",
    )

    start_nav_and_slam = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_stereo_camera,
            on_exit=_on_success(
                "Wait for ZED2i topics",
                [
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
    )

    start_camera_wait = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_lidar,
            on_exit=_on_success("Wait for RPLIDAR scan filter", [wait_for_stereo_camera])
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
