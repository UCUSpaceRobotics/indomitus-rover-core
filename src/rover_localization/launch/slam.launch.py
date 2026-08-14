# rover_localization/launch/slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotEqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_localization")
    default_config_file = os.path.join(pkg, "config", "slam_toolbox_params.yaml")

    use_sim_time_val = LaunchConfiguration("use_sim_time")
    config_file_val = LaunchConfiguration("config_path")

    custom_config_file_present = NotEqualsSubstitution(config_file_val, "")
    config_file = IfElseSubstitution(
        custom_config_file_present,
        if_value=config_file_val,
        else_value=default_config_file,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from Gazebo (true) or wall clock (false)"
        ),

        DeclareLaunchArgument(
            "config_path",
            default_value=default_config_file,
            description="Full path to the SLAM parameters file"
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time_val}],
        ),
    ])