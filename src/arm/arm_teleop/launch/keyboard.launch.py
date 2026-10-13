"""Keyboard teleop for the arm: keyboard_servo_node under /arm.

  ros2 launch arm_teleop keyboard.launch.py                     # real arm / fake hardware
  ros2 launch arm_teleop keyboard.launch.py use_sim_time:=true  # Gazebo sim
  ros2 launch arm_teleop keyboard.launch.py linear_speed:=1.0 angular_speed:=2.0

Reads the keyboard through evdev (/dev/input), not this terminal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    # Empty = keep the node's own default.
    for name in ('linear_speed', 'angular_speed', 'activity_indicator_pre_delay_sec'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            params[name] = float(value)

    return [Node(
        package='arm_teleop',
        executable='keyboard_servo_node',
        # Same namespace as the arm stack (arm_bringup/arm.launch.py, arm_sim).
        namespace='arm',
        output='screen',
        emulate_tty=True,
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when teleoperating the Gazebo sim'),
        DeclareLaunchArgument('linear_speed', default_value='',
                              description='EEF translation speed, m/s (empty: node default)'),
        DeclareLaunchArgument('angular_speed', default_value='',
                              description='EEF rotation speed, rad/s (empty: node default)'),
        DeclareLaunchArgument(
            'activity_indicator_pre_delay_sec', default_value='',
            description="Wait before r/p/f moves (empty: node default). ERC requires >= 5.0 "
                        "at competition."),
        OpaqueFunction(function=_make_node),
    ])
