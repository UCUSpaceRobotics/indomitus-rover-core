import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    layout_path = os.path.join(get_package_share_directory('rover_viz'), 'plotjuggler', 'power_monitor.xml')

    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='power_monitor',
        arguments=['-l', layout_path]
    )

    return LaunchDescription([
        plotjuggler
    ])
