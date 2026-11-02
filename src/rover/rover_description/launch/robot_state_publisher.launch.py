from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('xacro_file'),
        DeclareLaunchArgument('xacro_args', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_frequency', default_value='20.0'),
        DeclareLaunchArgument('log_level', default_value='info'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ', LaunchConfiguration('xacro_file'), ' ', LaunchConfiguration('xacro_args'),
                    ]),
                    value_type=str
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
            remappings=[
                # tf/tf_static stay global regardless of the pushed rover
                # namespace - see docs/software/tf_ownership.md.
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
        ),
    ])