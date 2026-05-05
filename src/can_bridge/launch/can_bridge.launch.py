from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description() -> LaunchDescription:

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
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'timeout_sec': 0.01,
        }],
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

    configure_sender = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sender_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    configure_receiver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(receiver_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_sender = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sender_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    activate_receiver = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=receiver_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(receiver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        interface_arg,
        sender_node,
        receiver_node,
        configure_sender,
        configure_receiver,
        activate_sender,
        activate_receiver,
    ])