"""Rover cameras, separate from rover.launch.py so they restart (or switch ZED2i
mode) without taking down CAN/control/drive. systemd: rover-launch-cameras,
tmux session `cameras`.

    ros2 launch rover_bringup cameras.launch.py
    ros2 launch rover_bringup cameras.launch.py zed2i_mode:=nav
    ros2 launch rover_bringup cameras.launch.py use_cameras:=false zed2i_mode:=nav
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import PushRosNamespace
from rover_bringup.launch_utils import include_launch


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'rover_namespace',
        default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
        description='ROS namespace all rover nodes/topics are pushed under (arm excluded).',
    )

    use_cameras_arg = DeclareLaunchArgument(
        'use_cameras', default_value='true',
        description='rover_cameras: MJPEG streams to the ground station + image_raw topics',
    )

    cameras_arg = DeclareLaunchArgument(
        'cameras', default_value='',
        description="rover_cameras' cameras:= (names to start; '' = all with autostart)",
    )

    zed2i_mode_arg = DeclareLaunchArgument(
        'zed2i_mode', default_value='',
        description=(
            "ZED2i stereo camera mode: 'rgb' for the operator color feed, 'nav' for the "
            "point cloud and VIO used by navigation, 'record' for traversability-dataset recording. "
            "Leave empty to not launch the camera at all."
        ),
        choices=['', 'rgb', 'nav', 'record'],
    )
    zed2i_mode_val = LaunchConfiguration('zed2i_mode')

    return LaunchDescription([
        namespace_arg,
        use_cameras_arg,
        cameras_arg,
        zed2i_mode_arg,

        GroupAction([
            PushRosNamespace(LaunchConfiguration('rover_namespace')),

            include_launch('rover_cameras', 'cameras.launch.py', {
                'rover_namespace': '',   # already pushed by this group
                'cameras': LaunchConfiguration('cameras'),
            }, condition=IfCondition(LaunchConfiguration('use_cameras'))),

            include_launch('rover_sensors', 'zed2i.launch.py', {
                'mode': zed2i_mode_val,
            }, condition=IfCondition(NotEqualsSubstitution(zed2i_mode_val, ''))),
        ]),
    ])
