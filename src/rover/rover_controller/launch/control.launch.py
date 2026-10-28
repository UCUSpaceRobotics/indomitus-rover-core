# rover_controller/launch/control.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_controller_spawners(context):
    yaml_path = LaunchConfiguration('controllers_yaml').perform(context)
    controllers = LaunchConfiguration('controllers').perform(context).split()
    inactive = set(LaunchConfiguration('inactive_controllers').perform(context).split())

    # One spawner per group, not per controller: with --unload-on-kill each
    # spawner stays alive until Ctrl+C, then unloads its controllers - so the
    # software stack can be restarted against an already-running Gazebo.
    groups = [
        ([c for c in controllers if c not in inactive], []),
        ([c for c in controllers if c in inactive], ['--inactive']),
    ]

    spawners = []
    for names, extra_args in groups:
        if not names:
            continue
        spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=[*names, '--param-file', yaml_path, '--unload-on-kill', *extra_args],
            output='screen',
        ))
    return spawners


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument('controllers_yaml'),
        DeclareLaunchArgument('controllers',
                               description='space-separated controller names to spawn'),
        DeclareLaunchArgument('inactive_controllers', default_value='',
                               description='space-separated subset of controllers to spawn inactive'),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': ParameterValue(LaunchConfiguration('robot_description'), value_type=str)},                LaunchConfiguration('controllers_yaml'),
            ],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_sim')),
        ),

        OpaqueFunction(function=make_controller_spawners),
    ])
