from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rover_hardware_interface'),
        'config', 'chassis_driver.yaml'
    )

    return LaunchDescription([
        Node(
            package='rover_hardware_interface',
            executable='chassis_driver_node_exe',
            name='chassis_driver',
            output='screen',
            parameters=[config],
        )
    ])
