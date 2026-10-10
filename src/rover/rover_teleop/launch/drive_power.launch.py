import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """The drive power owner.

    Deliberately separate from joy.launch.py and included by every bringup —
    hardware and simulation alike: the ground station has to be able to power
    the drive with no gamepad plugged into the rover, and in simulation the
    joystick's motor and compact buttons have to reach something.
    """
    config = os.path.join(
        get_package_share_directory('rover_teleop'),
        'config',
        'drive_power.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of the system clock.'),
        # Simulation can spawn either swerve controller, so bringup has to be
        # able to say which one drive/power activates.
        DeclareLaunchArgument(
            'controller_name', default_value='swerve_controller_test',
            description='Swerve controller that drive/power activates and deactivates.'),

        Node(
            package='rover_teleop',
            executable='drive_power_node',
            name='drive_power',
            parameters=[config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'controller_name': LaunchConfiguration('controller_name'),
            }],
            output='screen',
        ),
    ])
