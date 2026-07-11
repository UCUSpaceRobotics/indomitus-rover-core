import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_description_dir = get_package_share_directory('arm_description')
    arm_viz_dir = get_package_share_directory('arm_viz')
    xacro_file = os.path.join(arm_description_dir, 'urdf', 'arm_standalone.urdf.xacro')

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use mock_components/GenericSystem instead of the real CAN hardware interface'
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware')
        ]),
        value_type=str
    )

    return LaunchDescription([
        use_fake_hardware_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(arm_viz_dir, 'rviz', 'arm.rviz')]
        )
    ])
