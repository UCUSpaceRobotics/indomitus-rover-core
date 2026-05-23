from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    config_file = LaunchConfiguration('config_file')

    default_config = PathJoinSubstitution([
        FindPackageShare('indomitus_rover_bringup'),
        'config',
        'joy.yaml'
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/cmd_vel', cmd_vel_topic),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Output velocity command topic'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='teleop_twist_joy config file'
        ),
        joy_node,
        teleop_node,
    ])
