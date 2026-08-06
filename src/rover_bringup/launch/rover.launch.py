import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from rover_bringup.launch_utils import include_launch


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')
    rover_description_share = get_package_share_directory('rover_description')

    interface_arg = DeclareLaunchArgument(
        'interface', default_value='can0',
        description='SocketCAN network interface name',
    )


    robot_description = Command([
        'xacro ',
        os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
        ' use_sim:=false',
        ' can_interface:=', LaunchConfiguration('interface'),
    ])

    return LaunchDescription([
        interface_arg,
        include_launch('rover_bringup', 'can.launch.py', {
            'interface': LaunchConfiguration('interface'),
        }),
        include_launch('rover_description', 'robot_state_publisher.launch.py', {
            'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
        }),
        include_launch('rover_bringup', 'control.launch.py', {
            'use_sim': 'false',
            'robot_description': robot_description,
            'controllers_yaml': os.path.join(rover_bringup_share, 'config', 'controllers.yaml'),
            # swerve_controller_test is loaded but left inactive alongside
            # swerve_controller — only one may hold the joints at a time, so
            # switching between them is a switch_controller call.
            'controllers': 'joint_state_broadcaster swerve_controller '
                           'swerve_controller_test odometry_controller',
            'inactive_controllers': 'swerve_controller swerve_controller_test',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),
        include_launch('rover_peripherals', 'lighting.launch.py'),
        include_launch('rover_peripherals', 'power_monitor_node.launch.py'),
        include_launch('rover_localization', 'ekf.launch.py')
    ])
