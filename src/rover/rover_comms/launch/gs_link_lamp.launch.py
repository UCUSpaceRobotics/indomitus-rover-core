from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument(
        'gs_link_state_topic',
        default_value='/gs/link/state',
        description="link_status_node's heartbeat, published under the "
                    "ground station's /gs namespace"
    )

    # 4 missed samples at link_status_node's 2 Hz.
    timeout_arg = DeclareLaunchArgument(
        'timeout_sec',
        default_value='2.0',
        description='Seconds without a fresh /gs/link/state before the lamp goes off'
    )

    return LaunchDescription([
        topic_arg,
        timeout_arg,
        Node(
            package='rover_comms',
            executable='gs_link_lamp_node',
            name='gs_link_lamp_node',
            output='screen',
            parameters=[{
                'gs_link_state_topic': LaunchConfiguration('gs_link_state_topic'),
                'timeout_sec': LaunchConfiguration('timeout_sec'),
            }],
        ),
    ])
