from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    deadzone = LaunchConfiguration('deadzone')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')

    default_config = PathJoinSubstitution([
        FindPackageShare('rover_teleop'),
        'config',
        'joy.yaml',
    ])

    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.0, # keep it 0.0, it fixes a bug 
            # in game_controller node source code
            'autorepeat_rate': ParameterValue(autorepeat_rate, value_type=float),
        }],
        respawn=True,
        respawn_delay=10.0
    )

    joy_interpreter = Node(
        package='rover_teleop',
        executable='joystick_interpreter_node',
        output='screen',
        parameters=[default_config],
        remappings=[
            ('/cmd_vel', '/cmd_vel_joy'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.0',
            description='Driver-side joystick deadzone. Keep at 0.0: a non-zero '
                        'value makes game_controller_node stop publishing /joy '
                        'while the sticks rest, because resting-stick noise gets '
                        'collapsed to 0.0 and starves its autorepeat heartbeat '
                        '(ros-drivers/joystick_drivers#304). The real deadzone is '
                        'applied by joystick_interpreter (see config/joy.yaml).'
        ),
        DeclareLaunchArgument(
            'autorepeat_rate',
            default_value='20.0',
            description='Joystick autorepeat rate (Hz); 0.0 disables autorepeat, '
                        'joystick will only send on state change'
        ),
        joy_node,
        joy_interpreter,
    ])
