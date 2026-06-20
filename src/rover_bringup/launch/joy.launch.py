from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_path = LaunchConfiguration('device_path')
    deadzone = LaunchConfiguration('deadzone')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')

    default_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'),
        'config',
        'joy.yaml',
    ])

    twist_mux_config = PathJoinSubstitution([
        FindPackageShare('rover_bringup'),
        'config',
        'twist_mux.yaml',
    ])

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_path': device_path,
            'deadzone': deadzone,
            'autorepeat_rate': autorepeat_rate,
        }],
        respawn=True,
        respawn_delay=10.0
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[default_config],
        remappings=[
            ('/cmd_vel', cmd_vel_topic),
        ]
    )

    joy_interpreter = Node(
        package='rover_control',
        executable='joystick_interpreter_node',
        output='screen',
        parameters=[default_config],
        remappings=[
            ('/cmd_vel', '/cmd_vel/teleop'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/input/js0',
            description='Joystick device pathy'
        ),
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.05',
            description='Joystick deadzone'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/joy_raw_cmd_vel',
            description='Output velocity command topic'
        ),
        DeclareLaunchArgument(
            'autorepeat_rate',
            default_value='20.0',
            description='Joystick autorepeat rate (Hz); 0.0 disables autorepeat, '
                        'joystick will only send on state change'
        ),
        joy_node,
        teleop_node,
        joy_interpreter,
    ])