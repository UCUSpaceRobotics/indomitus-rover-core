import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    power_config = os.path.join(get_package_share_directory('rover_peripherals'), 'config', 'power_node.yaml')

    power_node = Node(
        package='rover_peripherals',
        executable='rover_power_node',
        name='power_node',
        parameters=[power_config],
    )
    
    return LaunchDescription([
        power_node
    ])
