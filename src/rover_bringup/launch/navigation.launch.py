from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from rover_bringup.launch_utils import include_launch

def generate_launch_description() -> LaunchDescription:
    stereo_camera_launch = include_launch("rover_sensors", "zed2i.launch.py")
    lidar_launch = include_launch("rover_sensors", "rplidar_s2.launch.py")
    scan_filter_launch = include_launch("rover_sensors", "scan_filter.launch.py")

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

                include_launch('rover_localization', 'slam.launch.py', {
                    'use_sim_time': 'false',
                }),

                include_launch('rover_navigation', 'nav2.launch.py', {
                    'use_sim': 'false',
                    'cmd_vel_topic': 'cmd_vel_nav',
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
        stereo_camera_launch,
        lidar_launch,
        scan_filter_launch,

        wait_for_lidar,
        start_camera_wait,
        start_nav_and_slam,
    ])