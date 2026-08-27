import os
from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from rover_bringup.launch_utils import include_launch


def _rover_sensors_include(launch_file, launch_arguments, label, condition=None):
    try:
        get_package_share_directory('rover_sensors')
    except PackageNotFoundError:
        return LogInfo(msg=f'rover_sensors is not built - starting without {label}.')
    return include_launch('rover_sensors', launch_file, launch_arguments, condition=condition)


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
        _rover_sensors_include('arducam.launch.py', {
            'camera_name': 'mast',
            'camera_path': '/dev/arducam-mast',
            'namespace': 'mast_arducam',
            'camera_frame_id': 'mast_arducam_optical_frame',
        }, label='the mast arducam'),
        _rover_sensors_include('arducam.launch.py', {
            'camera_name': 'rear',
            'camera_path': '/dev/arducam-rear',
            'namespace': 'rear_arducam',
            'camera_frame_id': 'rear_arducam_optical_frame',
        }, label='the rear arducam'),
        _rover_sensors_include('arducam.launch.py', {
            'camera_name': 'container',
            'camera_path': '/dev/arducam-container',
            'namespace': 'container_arducam',
            'camera_frame_id': 'container_arducam_optical_frame',
        }, label='the container arducam'),
        include_launch('rover_description', 'robot_state_publisher.launch.py', {
            'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
            'xacro_args': ['use_sim:=false can_interface:=', LaunchConfiguration('interface')]
        }),
        include_launch('rover_bringup', 'control.launch.py', {
            'use_sim': 'false',
            'robot_description': robot_description,
            'controllers_yaml': os.path.join(rover_bringup_share, 'config', 'controllers.yaml'),
            'controllers': 'joint_state_broadcaster odometry_controller '
                           'swerve_controller_test',
            'inactive_controllers': 'swerve_controller_test',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),
        include_launch('rover_teleop', 'drive_power.launch.py'),
    ])
