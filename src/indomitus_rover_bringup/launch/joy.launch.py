from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    enable_strafe = LaunchConfiguration('enable_strafe')

    config_filename = PythonExpression([
        "'joy_strafe_enabled.yaml' if '", enable_strafe, "' == 'true' else 'joy_strafe_disabled.yaml'"
    ])

    default_config = PathJoinSubstitution([
        FindPackageShare('indomitus_rover_bringup'),
        'config',
        config_filename
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
        parameters=[default_config],
        remappings=[
            ('/cmd_vel', cmd_vel_topic),
        ]
    )

    joy_intepreter = Node(
        package='indomitus_rover_control',
        executable='joystick_interpreter_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/joy_raw_cmd_vel',
            description='Output velocity command topic'
        ),
        DeclareLaunchArgument(
            'enable_strafe',
            default_value='true',
            choices=['true', 'false'],
            description='Set to true to enable y-axis strafing, false for differential drive'
        ),
        joy_node,
        teleop_node,
        joy_intepreter,
    ])
