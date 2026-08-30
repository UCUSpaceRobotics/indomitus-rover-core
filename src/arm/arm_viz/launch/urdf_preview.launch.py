import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_description_dir = get_package_share_directory('arm_description')
    xacro_file = os.path.join(arm_description_dir, 'urdf', 'arm_standalone.urdf.xacro')

    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_fake_hardware:=true',
            ' end_effector:=', LaunchConfiguration('end_effector'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector', default_value='jaw',
            description="'jaw', 'other_tool', or 'drill_sampling' — see arm_macro.xacro"),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('arm_viz'), 'rviz', 'arm.rviz')],
        ),
    ])
