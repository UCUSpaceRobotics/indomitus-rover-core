# rover_navigation/launch/slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_navigation")
    params_file = os.path.join(pkg, "config", "slam_toolbox_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from Gazebo (true) or wall clock (false)."
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])