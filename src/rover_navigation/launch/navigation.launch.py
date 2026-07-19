# rover_navigation/launch/navigation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_navigation")

    params_file          = os.path.join(pkg, "config",          "nav2_params.yaml")
    nav_to_pose_bt       = os.path.join(pkg, "behaviour_trees", "navigate_to_pose_w_replanning.xml")
    nav_through_poses_bt = os.path.join(pkg, "behaviour_trees", "navigate_through_poses_w_replanning.xml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    lc_navigation = [
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
    ]

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock from Gazebo (true) or wall clock (false)."
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "default_nav_to_pose_bt_xml":       nav_to_pose_bt,
                    "default_nav_through_poses_bt_xml": nav_through_poses_bt,
                },
            ],
        ),

        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": lc_navigation,
            }],
        ),
    ])