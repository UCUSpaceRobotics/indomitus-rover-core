import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rover_peripherals'),
        'config',
        'lights.yaml',
    )

    lighting_node = Node(
        package='rover_peripherals',
        executable='rover_lighting_node',
        name='lights_can_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        lighting_node
    ])
