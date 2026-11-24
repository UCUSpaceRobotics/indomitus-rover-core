import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rover_drive_dir = get_package_share_directory('rover_drive')

    twist_mux_config = os.path.join(
        rover_drive_dir,
        'config',
        'twist_mux.yaml',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('cmd_vel_out', 'cmd_vel'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of the system clock.'),
        twist_mux_node,
    ])