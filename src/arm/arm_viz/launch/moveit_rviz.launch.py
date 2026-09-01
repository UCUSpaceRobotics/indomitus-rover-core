import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings={
            "use_fake_hardware": "true",
            "end_effector": LaunchConfiguration("end_effector"),
        })
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector', default_value='jaw',
            description="'jaw', 'other_tool', or 'drill_sampling' — must match what arm_bringup/arm.launch.py was actually started with"),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(
                get_package_share_directory("arm_moveit_config"), "config", "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
            ],
            # See arm_bringup/arm.launch.py's git history / this session's
            # own debugging notes for why these specific bare names need
            # remapping: several internal moveit_ros_visualization
            # sub-displays ignore moveit.rviz's "Move Group Namespace: arm"
            # entirely and use these names bare.
            remappings=[
                ("joint_states", "arm/joint_states"),
                ("monitored_planning_scene", "arm/monitored_planning_scene"),
                ("planning_scene", "arm/planning_scene"),
                ("planning_scene_world", "arm/planning_scene_world"),
                ("display_planned_path", "arm/display_planned_path"),
                ("recognized_object_array", "arm/recognized_object_array"),
            ],
            output="screen",
        ),
    ])
