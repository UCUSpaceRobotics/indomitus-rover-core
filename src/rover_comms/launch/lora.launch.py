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

    # Wire scale, NOT the speed limit - that is limit_linear/limit_angular
    # below. These must match lora_gateway_node's max_linear/max_angular on the
    # ground station, which in turn match the joystick's scales. The wire
    # carries percent of full scale, not m/s, so a mismatch rescales every
    # command silently rather than failing.
    max_linear_arg = DeclareLaunchArgument(
        'max_linear',
        default_value='0.5',
        description='m/s the wire calls 100%. Must equal lora_gateway_node\'s'
    )

    max_angular_arg = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        description='rad/s the wire calls 100%. Must equal lora_gateway_node\'s'
    )

    # The speed limit, applied after decoding and owned entirely by this end.
    # Lower than the Wi-Fi path on purpose: this is a crawl-home link. Do not
    # try to get a lower top speed by shrinking the wire scale above - that
    # rescales everything below the cap too, and breaks the contract with the
    # ground station.
    limit_linear_arg = DeclareLaunchArgument(
        'limit_linear',
        default_value='0.3',
        description='m/s ceiling on the LoRa path. Lower than Wi-Fi on purpose'
    )

    limit_angular_arg = DeclareLaunchArgument(
        'limit_angular',
        default_value='0.6',
        description='rad/s ceiling on the LoRa path. Lower than Wi-Fi on purpose'
    )

    return LaunchDescription([
        port_arg,
        max_linear_arg,
        max_angular_arg,
        limit_linear_arg,
        limit_angular_arg,
        Node(
            package='rover_comms',
            executable='lora_rover_node',
            name='lora_rover_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
                'limit_linear': LaunchConfiguration('limit_linear'),
                'limit_angular': LaunchConfiguration('limit_angular'),
            }],
        ),
    ])
