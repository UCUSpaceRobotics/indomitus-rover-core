import os
from typing import List

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from rover_bringup.launch_utils import include_launch


def launch_nodes(context: LaunchContext,
                 **substitutions: launch.substitutions.LaunchConfiguration
                 ) -> List[Node]:
    rover_description_share = get_package_share_directory('rover_description')
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}

    use_rviz = kwargs.get('use_rviz', 'true').lower() == 'true'
    use_joint_gui = kwargs.get('use_joint_state_publisher_gui', 'true').lower() == 'true'

    nodes = []

    robot_state_publisher_node = include_launch('rover_description', 'robot_state_publisher.launch.py', {
        'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.urdf.xacro'),
        'xacro_args': f'name:={kwargs.get('name', '')}',
        'publish_frequency': '100.0',
        'log_level': 'warn',
    })
    nodes.append(robot_state_publisher_node)

    if use_joint_gui:
        joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    else:
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    nodes.append(joint_state_publisher_node)

    if use_rviz:
        rviz_config_file = os.path.join(
            get_package_share_directory('rover_viz'), 'rviz', 'robot.rviz')

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    declared_args = [
        launch.actions.DeclareLaunchArgument(
            'name',
            default_value='',
            description='Name of the robot used as a tf prefix'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui for joint control'
        ),
    ]

    all_kwargs = {
        'name': launch.substitutions.LaunchConfiguration('name'),
        'use_rviz': launch.substitutions.LaunchConfiguration('use_rviz'),
        'use_joint_state_publisher_gui': launch.substitutions.LaunchConfiguration('use_joint_state_publisher_gui'),
    }

    return LaunchDescription(
        declared_args + [
            launch.actions.OpaqueFunction(
                function=launch_nodes,
                kwargs=all_kwargs),
        ])
