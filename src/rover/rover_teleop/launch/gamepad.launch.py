"""Gamepad teleop: game_controller_node + gamepad_interpreter, side by side
on the machine the gamepad is plugged into.

`source` says which control source this gamepad is (see
docs/software/control_sources.md). Each source has its own raw Joy topic, its
own node names and its own twist_mux input, so two of them never share a topic:

  source:=remote (default)  a gamepad on an operator's laptop
      /rover/remote/joy, /rover/remote_joy_node,
      /rover/remote_gamepad_interpreter -> cmd_vel_remote
  source:=rover             the wired gamepad plugged into the rover itself
      /rover/joy, /rover/joy_node, /rover/gamepad_interpreter -> cmd_vel_joy
      (top twist_mux priority); rover.launch.py passes this explicitly

On an operator's laptop:
    ros2 launch rover_teleop gamepad.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SOURCES = {
    'rover': ('', 'joy', 'cmd_vel_joy', 'teleop/joystick_active'),
    'remote': ('remote_', 'remote/joy', 'cmd_vel_remote', '~/active'),
}


def _nodes(context):
    source = LaunchConfiguration('source').perform(context)
    if source not in SOURCES:
        raise RuntimeError(f"gamepad.launch.py: source must be one of {list(SOURCES)}, "
                           f"got {source!r}")
    prefix, joy_topic, cmd_vel_topic, active_topic = SOURCES[source]

    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name=f'{prefix}joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.0, # keep it 0.0, it fixes a bug
            # in game_controller node source code
            'autorepeat_rate': ParameterValue(LaunchConfiguration('autorepeat_rate'),
                                              value_type=float),
        }],
        # set_feedback (rumble) too, or a laptop's rumble would reach the
        # rover's pad and the other way round.
        remappings=[('joy', joy_topic),
                    ('joy/set_feedback', f'{joy_topic}/set_feedback')],
        respawn=True,
        respawn_delay=10.0
    )

    joy_interpreter = Node(
        package='rover_teleop',
        executable='gamepad_interpreter_node',
        name=f'{prefix}gamepad_interpreter',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('rover_teleop'), 'config', 'gamepad.yaml'])],
        remappings=[
            ('joy', joy_topic),
            ('cmd_vel', cmd_vel_topic),
            ('teleop/joystick_active', active_topic),
        ]
    )

    # Empty rover_namespace: the including launch has pushed it already.
    namespace = LaunchConfiguration('rover_namespace').perform(context)
    if not namespace:
        return [joy_node, joy_interpreter]
    return [GroupAction([PushRosNamespace(namespace), joy_node, joy_interpreter])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'source',
            default_value='remote',
            choices=list(SOURCES),
            description="Which control source this gamepad is: 'remote' (an operator's "
                        "laptop, cmd_vel_remote) or 'rover' (wired to the rover, "
                        "cmd_vel_joy, top priority).",
        ),
        DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description="Namespace to push; '' when the including launch already pushed it.",
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
        OpaqueFunction(function=_nodes),
    ])
