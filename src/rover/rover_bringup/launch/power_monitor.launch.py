"""PlotJuggler for the rover's power monitor. power_node itself runs in
rover.launch.py, so it is not started here.

    ros2 launch rover_bringup power_monitor.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from rover_bringup.launch_utils import include_launch


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'rover_namespace',
        default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
        description='ROS namespace all rover nodes/topics are pushed under (arm excluded).',
    )

    return LaunchDescription([
        namespace_arg,

        GroupAction([
            PushRosNamespace(LaunchConfiguration('rover_namespace')),

            include_launch('rover_viz', 'power_monitor_viz.launch.py'),
        ]),
    ])
