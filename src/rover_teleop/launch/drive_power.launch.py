import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """The drive power owner.

    Deliberately separate from joy.launch.py and included by rover bringup:
    the ground station has to be able to power the drive with no gamepad
    plugged into the rover at all.
    """
    config = os.path.join(
        get_package_share_directory('rover_teleop'),
        'config',
        'drive_power.yaml',
    )

    drive_power_node = Node(
        package='rover_teleop',
        executable='drive_power_node',
        name='drive_power',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        drive_power_node
    ])
