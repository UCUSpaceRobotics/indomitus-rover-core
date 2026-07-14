from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lighting_node = Node(
        package='rover_peripherals',
        executable='rover_lighting_node',
        name='lights_can_node',
        output='screen',
    )

    return LaunchDescription([
        lighting_node
    ])