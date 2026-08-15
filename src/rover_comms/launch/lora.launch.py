from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The 40-pin header UART. nvgetty and serial-getty@ttyTHS0 must both be
    # disabled, and the user running this must be in dialout - neither is true
    # on a fresh JetPack image.
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyTHS1',
        description='Serial port the E32 is wired to (M0/M1 strapped low, AUX unused)'
    )

    # These must match lora_gateway_node's max_linear/max_angular on the ground
    # station. The wire carries percent of full scale, not m/s, so a mismatch
    # scales every command silently rather than failing.
    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.3',
        description='m/s at 100%. Lower than the Wi-Fi path on purpose'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='0.6',
        description='rad/s at 100%. Lower than the Wi-Fi path on purpose'
    )

    return LaunchDescription([
        port_arg,
        max_linear_arg,
        max_angular_arg,
        Node(
            package='rover_comms',
            executable='lora_rover_node',
            name='lora_rover_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
            }],
        ),
    ])
