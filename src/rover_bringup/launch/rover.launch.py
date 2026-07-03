# rover_bringup/launch/rover.launch.py
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def lifecycle_sequence(node):
    return [
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )),
    ]


def check_can_interface_up(context, *args, **kwargs):
    """
    Runs at launch time (after argument substitution).
    Aborts the launch with a clear error if the CAN interface is missing or down.
    """
    interface = LaunchConfiguration('interface').perform(context)

    try:
        output = subprocess.check_output(
            ['ip', 'link', 'show', interface],
            stderr=subprocess.STDOUT,
            text=True,
        )
    except subprocess.CalledProcessError:
        raise RuntimeError(
            f"CAN interface '{interface}' does not exist.")
    except FileNotFoundError:
        raise RuntimeError("'ip' command not found — cannot verify CAN interface state.")

    if 'state UP' not in output and ',UP,' not in output:
        raise RuntimeError(
            f"CAN interface '{interface}' exists but is DOWN.")

    return [LogInfo(msg=f"CAN interface '{interface}' is up — proceeding with launch.")]


def launch_setup(context, *args, **kwargs):

    rover_bringup_dir     = get_package_share_directory('rover_bringup')

    twist_mux_config = PathJoinSubstitution([
        rover_bringup_dir,
        'config',
        'twist_mux.yaml',
    ])

    sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace='',
        parameters=[{'interface': LaunchConfiguration('interface'), 'timeout_sec': 0.01}],
        output='screen',
    )

    receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace='',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'interval_sec': 0.01,
            # candump-синтаксис: id:mask (hex, without 0x).
            # Pass only 0x300-0x3FF (ESP),
            # all other ids are filtered out.
            # example: 'filters': '300:700,400:700',
            'filters': '300:700',
        }],
        output='screen',
    )

    robot_description = Command([
        'xacro ',
        os.path.join(rover_bringup_dir, 'urdf', 'rover_real.urdf.xacro'),
        ' can_interface:=', LaunchConfiguration('interface'),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(rover_bringup_dir, 'config', 'controllers.yaml'),
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_controller', '--inactive'],
        output='screen',
    )

    odometry_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['odometry_controller'],
        output='screen',
    )

    lighting_node = Node(
        package='rover_peripherals',
        executable='rover_lighting_node',
        name='lights_can_node',
        output='screen',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel'),
        ]
    )

    return [
        sender_node,
        receiver_node,
        *lifecycle_sequence(sender_node),
        *lifecycle_sequence(receiver_node),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        swerve_controller_spawner,
        odometry_controller_spawner,
        twist_mux_node,
        lighting_node,
    ]


def generate_launch_description():
    interface_arg = DeclareLaunchArgument(
        'interface', default_value='can0',
        description='SocketCAN network interface name',
    )

    return LaunchDescription([
        interface_arg,
        OpaqueFunction(function=check_can_interface_up),
        OpaqueFunction(function=launch_setup),
    ])
