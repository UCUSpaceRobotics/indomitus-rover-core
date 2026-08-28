from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    end_effector = LaunchConfiguration('end_effector')

    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            # Must stay 0.0 — game_controller_node stops publishing /joy
            # entirely once its own deadzone swallows resting-stick noise
            # (ros-drivers/joystick_drivers#304). Deadzone is applied in
            # GamepadInputLoop (_DEADZONE) instead.
            'deadzone': ParameterValue(0.0, value_type=float),
        }],
        respawn=True,
        respawn_delay=10.0,
    )

    gamepad_servo_node = Node(
        package='arm_tasks',
        executable='gamepad_servo_node',
        output='screen',
        parameters=[{
            'end_effector': end_effector,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='jaw',
            description=(
                "Which tool is physically mounted right now: 'jaw', "
                "'drill_sampling', or 'astrobio'. Gates the A/B/Y mode-jump "
                'buttons in gamepad_servo_node — match this to what you '
                'actually launched the arm with (demo.launch.py '
                'end_effector:=...).'
            ),
        ),
        game_controller_node,
        gamepad_servo_node,
    ])
