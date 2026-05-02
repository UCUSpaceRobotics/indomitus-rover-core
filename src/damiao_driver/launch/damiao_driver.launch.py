from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('damiao_driver'),
        'config', 'damiao_driver.yaml'
    )

    return LaunchDescription([
        Node(
            package='damiao_driver',
            executable='damiao_driver_node',
            name='damiao_driver',
            output='screen',
            parameters=[config],
        )
    ])
