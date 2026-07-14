# rover_bringup/launch/control.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_controller_spawners(context):
    yaml_path = LaunchConfiguration('controllers_yaml').perform(context)
    controllers = LaunchConfiguration('controllers').perform(context).split()
    inactive = set(LaunchConfiguration('inactive_controllers').perform(context).split())

    spawners = []
    for name in controllers:
        args = [name, '--param-file', yaml_path]
        if name in inactive:
            args.append('--inactive')
        spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=args,
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
                {'robot_description': LaunchConfiguration('robot_description')},
                LaunchConfiguration('controllers_yaml'),
            ],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_sim')),
        ),

        OpaqueFunction(function=make_controller_spawners),
    ])
