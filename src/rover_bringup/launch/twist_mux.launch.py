import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rover_bringup_dir = get_package_share_directory('rover_bringup')

    twist_mux_config = os.path.join(
        rover_bringup_dir,
        'config',
        'twist_mux.yaml',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        twist_mux_node,
    ])