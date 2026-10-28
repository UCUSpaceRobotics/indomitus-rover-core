import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, Command, NotEqualsSubstitution,
    IfElseSubstitution,
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

    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Software stack only, against an already-running '
                    'rover_sim/sim_gz.launch.py: the sim owns robot_state_publisher '
                    'and the controller_manager, and the hardware-only nodes are skipped',
    )
    use_sim_val = LaunchConfiguration('use_sim')

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

    use_cameras_arg = DeclareLaunchArgument(
        'use_cameras', default_value='true',
        description='rover_cameras: MJPEG streams to the ground station + image_raw topics',
    )

    zed2i_mode_arg = DeclareLaunchArgument(
        'zed2i_mode', default_value='',
        description=(
            "ZED2i stereo camera mode: 'rgb' for the operator color feed, 'nav' for the "
            "point cloud and VIO used by navigation, 'record' for traversability-dataset recording. "
            "Leave empty to not launch the camera at all."
        ),
        choices=['', 'rgb', 'nav', 'record'],
    )
    zed2i_mode_val = LaunchConfiguration('zed2i_mode')

    # Only for ros2_control_node on hardware; skipped in sim so xacro isn't run for nothing.
    robot_description = IfElseSubstitution(use_sim_val, if_value='', else_value=Command([
        'xacro ',
        os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
        ' use_sim:=false',
        ' can_interface:=', LaunchConfiguration('interface'),
    ]))

    # Hardware starts swerve inactive until drive_manager powers the drive. The
    # sim's diff_bar_effort_controller is spawned by sim_gz.launch.py, not here.
    controllers_yaml = IfElseSubstitution(
        use_sim_val,
        if_value=os.path.join(rover_bringup_share, 'config', 'controllers_sim.yaml'),
        else_value=os.path.join(rover_bringup_share, 'config', 'controllers.yaml'),
    )
    inactive_controllers = IfElseSubstitution(
        use_sim_val, if_value='', else_value='swerve_controller')

    return LaunchDescription([
        use_sim_arg,
        namespace_arg,
        interface_arg,
        use_joy_arg,
        joy_dev_arg,
        use_cameras_arg,
        zed2i_mode_arg,

        GroupAction([
            PushRosNamespace(namespace_val),

            # Shared by hardware and sim.
            include_launch('rover_controller', 'control.launch.py', {
                'use_sim': use_sim_val,
                'robot_description': robot_description,
                'controllers_yaml': controllers_yaml,
                'controllers': 'joint_state_broadcaster odometry_controller swerve_controller',
                'inactive_controllers': inactive_controllers,
            }),

            include_launch('rover_drive', 'twist_mux.launch.py', {
                'use_sim_time': use_sim_val,
            }),

            include_launch('rover_drive', 'drive_manager.launch.py', {
                'use_sim_time': use_sim_val,
            }),

            include_launch('rover_localization', 'ekf.launch.py', {
                'use_sim_time': use_sim_val,
            }),

            # Hardware only - in sim, Gazebo stands in for all of these.
            GroupAction([
                include_launch('rover_teleop', 'gamepad.launch.py', {
                    'rover_namespace': '',   # already pushed by this group
                    'source': 'rover',       # the default is a laptop gamepad
                    'joy_dev': LaunchConfiguration('joy_dev'),
                }, condition=IfCondition(LaunchConfiguration('use_joy'))),

                include_launch('rover_cameras', 'cameras.launch.py', {
                    'rover_namespace': '',   # already pushed by this group
                }, condition=IfCondition(LaunchConfiguration('use_cameras'))),

                include_launch('rover_bringup', 'can.launch.py', {
                    'interface': LaunchConfiguration('interface'),
                }),

                include_launch('rover_sensors', 'zed2i.launch.py', {
                    'mode': zed2i_mode_val,
                }, condition=IfCondition(NotEqualsSubstitution(zed2i_mode_val, ''))),

                include_launch('rover_description', 'robot_state_publisher.launch.py', {
                    'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
                    'xacro_args': [
                        'use_sim:=false can_interface:=', LaunchConfiguration('interface'),
                        ' rover_namespace:=', namespace_val,
                    ]
                }),

                include_launch('rover_comms', 'lora.launch.py'),

                include_launch('rover_diagnostics', 'fault_logger.launch.py'),

                include_launch('rover_peripherals', 'lighting.launch.py'),

                include_launch('rover_comms', 'gs_link_lamp.launch.py'),  # Drives lights/traffic_blue off /gs/link/state.

                include_launch('rover_drive', 'drive_source_lamp.launch.py'),  # red=autonomous, green=manual, off=nothing driving.

                include_launch('rover_peripherals', 'power_monitor_node.launch.py'),
            ], condition=UnlessCondition(use_sim_val)),
        ]),
    ])
