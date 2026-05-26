from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('indomitus_rover_chassis_driver'),
        'config', 'chassis_driver.yaml'
    )

    return LaunchDescription([
        Node(
            package='indomitus_rover_chassis_driver',
            executable='chassis_driver_node_exe',
            name='chassis_driver',
            output='screen',
            parameters=[config],
        )
    ])
