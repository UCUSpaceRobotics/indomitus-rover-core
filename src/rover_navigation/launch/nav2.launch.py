# rover_navigation/launch/nav2.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotEqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rover_navigation")

    nav2_real_config = os.path.join(pkg, "config", "nav2_params.yaml")
    nav2_sim_config = os.path.join(pkg, "config", "nav2_params_sim.yaml")

    nav_to_pose_bt = os.path.join(pkg, "behaviour_trees", "navigate_to_pose_w_replanning.xml")
    nav_through_poses_bt = os.path.join(pkg, "behaviour_trees", "navigate_through_poses_w_replanning.xml")

    use_sim_val = LaunchConfiguration("use_sim")
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Enable simulation mode (uses sim clock and sim nav2 config).",
    )

    cmd_vel_topic_val = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel_nav",
        description="The topic Nav2 will publish its velocity commands to. Defaults to cmd_vel_nav for twist_mux routing.",
    )

    nav2_params_file_val = LaunchConfiguration("params_file")
    nav2_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Full path to the ROS 2 parameters file to override default params",
    )

    default_params_file = IfElseSubstitution(
        use_sim_val,
        if_value=nav2_sim_config,
        else_value=nav2_real_config,
    )

    custom_params_file_present = NotEqualsSubstitution(nav2_params_file_val, "")
    params_file = IfElseSubstitution(
        custom_params_file_present,
        if_value=nav2_params_file_val,
        else_value=default_params_file,
    )

    lc_navigation = [
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
    ]

    return LaunchDescription([
        use_sim_arg,
        cmd_vel_topic_arg,
        nav2_params_file_arg,

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_val}],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_val}],
            remappings=[('cmd_vel', cmd_vel_topic_val)],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_val}],
            remappings=[('cmd_vel', cmd_vel_topic_val)],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_val,
                    "default_nav_to_pose_bt_xml": nav_to_pose_bt,
                    "default_nav_through_poses_bt_xml": nav_through_poses_bt,
                },
            ],
        ),

        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_val}],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_val,
                "autostart": True,
                "node_names": lc_navigation,
            }],
        ),
    ])