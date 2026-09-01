"""Just the rviz2 window — run alongside a core stack started elsewhere."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_viz_dir = get_package_share_directory('arm_viz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(arm_viz_dir, 'rviz', 'arm.rviz')],
        ),
    ])
