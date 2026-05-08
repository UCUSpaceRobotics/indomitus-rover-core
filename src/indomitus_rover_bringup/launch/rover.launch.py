# indomitus_rover_bringup/launch/rover.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
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


def generate_launch_description():
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='can0',
        description='SocketCAN network interface name',
    )

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
        }],
        output='screen',
    )

    # Kinematics + driver стартують після того як CAN lifecycle ноди активувались
    deferred_nodes = TimerAction(period=1.5, actions=[
        Node(
            package='indomitus_rover_control',
            executable='rover_kinematics_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('indomitus_rover_description'),
                    'config', 'rover_geometry.yaml',
                ),
            ],
        ),
        Node(
            package='chassis_driver',
            executable='chassis_driver_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('chassis_driver'),
                'config', 'chassis_driver.yaml',
            )],
        ),
    ])

    return LaunchDescription([
        interface_arg,
        sender_node,
        receiver_node,
        *lifecycle_sequence(sender_node),
        *lifecycle_sequence(receiver_node),
        deferred_nodes,
    ])
