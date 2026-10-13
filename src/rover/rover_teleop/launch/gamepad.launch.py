"""Gamepad teleop: game_controller_node + gamepad_interpreter, side by side
on the machine the gamepad is plugged into.

Namespace: pushes rover_namespace (default $ROVER_NAMESPACE or 'rover'),
so it works when run on its own. A launch that already pushed the namespace
includes it with rover_namespace:='' (see rover_bringup/rover.launch.py).

On the operator's laptop, drive through the rover's twist_mux below the
rover-side gamepad:
    ros2 launch rover_teleop gamepad.launch.py cmd_vel_topic:=cmd_vel_gs
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')
    namespace_val = LaunchConfiguration('rover_namespace')

    default_config = PathJoinSubstitution([
        FindPackageShare('rover_teleop'),
        'config',
        'gamepad.yaml',
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
        executable='gamepad_interpreter_node',
        output='screen',
        parameters=[default_config],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ]
    )

    def _in_namespace(context):
        # Empty rover_namespace: the including launch has pushed it already.
        if not namespace_val.perform(context):
            return [joy_node, joy_interpreter]
        return [GroupAction([PushRosNamespace(namespace_val), joy_node, joy_interpreter])]

    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description="Namespace to push; '' when the including launch already pushed it.",
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel_joy',
            description="twist_mux input: cmd_vel_joy (gamepad on the rover, top priority) "
                        "or cmd_vel_gs (gamepad on the operator's laptop)",
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'autorepeat_rate',
            default_value='20.0',
            description='Joystick autorepeat rate (Hz); 0.0 disables autorepeat, '
                        'joystick will only send on state change'
        ),
        OpaqueFunction(function=_in_namespace),
    ])
