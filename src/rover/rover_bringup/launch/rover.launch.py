import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, Command
)
from launch_ros.actions import PushRosNamespace
from rover_bringup.launch_utils import include_launch


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')
    rover_description_share = get_package_share_directory('rover_description')

    namespace_arg = DeclareLaunchArgument(
        'rover_namespace',
        default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
        description='ROS namespace all rover nodes/topics are pushed under (arm excluded).',
    )
    namespace_val = LaunchConfiguration('rover_namespace')

    interface_arg = DeclareLaunchArgument(
        'interface', default_value='can_rover',
        description='SocketCAN network interface name',
    )

    use_joy_arg = DeclareLaunchArgument(
        'use_joy', default_value='true',
        description='Wired gamepad plugged into the rover itself (cmd_vel_joy, '
                    'top twist_mux priority) - separate from the ground '
                    'station (cmd_vel_gs) and operator laptops (cmd_vel_remote)',
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0',
        description='Device path for the rover-side gamepad; only used when use_joy:=true',
    )

    robot_description = Command([
        'xacro ',
        os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
        ' use_sim:=false',
        ' can_interface:=', LaunchConfiguration('interface'),
    ])

    return LaunchDescription([
        namespace_arg,
        interface_arg,
        use_joy_arg,
        joy_dev_arg,

        # Cameras / ZED2i: cameras.launch.py (own tmux session, restarts without this one).
        GroupAction([
            PushRosNamespace(namespace_val),

            include_launch('rover_teleop', 'gamepad.launch.py', {
                'rover_namespace': '',   # already pushed by this group
                'source': 'rover',       # the default is a laptop gamepad
                'joy_dev': LaunchConfiguration('joy_dev'),
            }, condition=IfCondition(LaunchConfiguration('use_joy'))),

            include_launch('indomitus_can', 'can.launch.py', {
                'interface': LaunchConfiguration('interface'),
            }),

            include_launch('rover_description', 'robot_state_publisher.launch.py', {
                'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
                'xacro_args': [
                    'use_sim:=false can_interface:=', LaunchConfiguration('interface'),
                    ' rover_namespace:=', namespace_val,
                ]
            }),

            include_launch('rover_controller', 'control.launch.py', {
                'use_sim': 'false',
                'robot_description': robot_description,
                'controllers_yaml': os.path.join(rover_bringup_share, 'config', 'controllers.yaml'),
                'controllers': 'joint_state_broadcaster odometry_controller '
                               'swerve_controller',
                'inactive_controllers': 'swerve_controller',
            }),

            include_launch('rover_drive', 'twist_mux.launch.py'),

            include_launch('rover_drive', 'drive_manager.launch.py'),

            include_launch('rover_comms', 'lora.launch.py'),

            include_launch('rover_diagnostics', 'fault_logger.launch.py'),

            include_launch('rover_peripherals', 'lighting.launch.py'),

            include_launch('rover_comms', 'gs_link_lamp.launch.py'),  # Drives lights/traffic_blue off /gs/link/state.

            include_launch('rover_drive', 'drive_source_lamp.launch.py'),  # red=autonomous, green=manual, off=nothing driving.

            include_launch('rover_peripherals', 'power_monitor_node.launch.py'),

            include_launch('rover_localization', 'ekf.launch.py'),
        ]),
    ])
