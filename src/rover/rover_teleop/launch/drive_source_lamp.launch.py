from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_teleop',
            executable='drive_source_lamp_node',
            name='drive_source_lamp_node',
            output='screen',
        ),
    ])
