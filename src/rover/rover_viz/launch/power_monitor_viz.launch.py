"""PlotJuggler for the rover's power monitor. power_node itself runs in
rover.launch.py, so it is not started here.

    ros2 launch rover_viz power_monitor_viz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    layout_path = os.path.join(get_package_share_directory('rover_viz'), 'plotjuggler', 'power_monitor.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description='ROS namespace all rover nodes/topics are pushed under (arm excluded).',
        ),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('rover_namespace')),
            Node(
                package='plotjuggler',
                executable='plotjuggler',
                name='power_monitor',
                arguments=['--nosplash', '-l', layout_path],
            ),
        ]),
    ])
