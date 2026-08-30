import os
from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
# from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, Command #, NotEqualsSubstitution
)
from launch_ros.actions import PushRosNamespace
from rover_bringup.launch_utils import include_launch


def _lora_fallback():
    """Include the LoRa fallback link, or log why it is missing.

    include_launch() resolves the package share directory while the launch
    description is being built, so a rover_comms that is not built raises and
    takes the whole of bringup down with it. A backup command path must never
    be the reason the rover refuses to start - the same reason the node itself
    opens its serial port in a worker thread rather than in its constructor.
    """
    try:
        get_package_share_directory('rover_comms')
    except PackageNotFoundError:
        return LogInfo(msg='rover_comms is not built - starting without the '
                           'LoRa fallback. The rover has no backup command '
                           'path if Wi-Fi drops.')
    return include_launch('rover_comms', 'lora.launch.py')


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
        'interface', default_value='can0',
        description='SocketCAN network interface name',
    )

    # zed2i_mode_arg = DeclareLaunchArgument(
    #     'zed2i_mode', default_value='',
    #     description=(
    #         "ZED2i stereo camera mode: 'rgb' for the operator color feed, 'nav' for the "
    #         "point cloud and VIO used by navigation. Leave empty to not launch the camera at all."
    #     ),
    #     choices=['', 'rgb', 'nav'],
    # )
    # zed2i_mode_val = LaunchConfiguration('zed2i_mode')

    robot_description = Command([
        'xacro ',
        os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
        ' use_sim:=false',
        ' can_interface:=', LaunchConfiguration('interface'),
    ])

    return LaunchDescription([
        namespace_arg,
        interface_arg,
        # zed2i_mode_arg,
        GroupAction([
            PushRosNamespace(namespace_val),

            include_launch('rover_bringup', 'can.launch.py', {
                'interface': LaunchConfiguration('interface'),
            }),

            # include_launch('rover_sensors', 'zed2i.launch.py', {
            #     'mode': zed2i_mode_val,
            # }, condition=IfCondition(NotEqualsSubstitution(zed2i_mode_val, ''))),

            include_launch('rover_description', 'robot_state_publisher.launch.py', {
                'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
                'xacro_args': [
                    'use_sim:=false can_interface:=', LaunchConfiguration('interface'),
                    ' rover_namespace:=', namespace_val,
                ]
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

            # Owns motors + controller activation. Not in joy.launch.py on
            # purpose: the ground station must be able to power the drive with
            # no gamepad plugged into the rover.
            include_launch('rover_teleop', 'drive_power.launch.py'),

            _lora_fallback(), # Publishes cmd_vel_lora, which twist_mux carries below cmd_vel_ext.

            include_launch('rover_diagnostics', 'fault_logger.launch.py'),

            include_launch('rover_peripherals', 'lighting.launch.py'),

            include_launch('rover_peripherals', 'power_monitor_node.launch.py'),

            # include_launch('rover_localization', 'ekf.launch.py'),
        ]),
    ])
