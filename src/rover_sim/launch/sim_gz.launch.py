import os
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from rover_bringup.launch_utils import include_launch


def generate_bridge_config(context, rover_sim_share: str) -> list[Node]:
    world = LaunchConfiguration("world_name").perform(context)
    model = LaunchConfiguration("model_name").perform(context)

    template_path = os.path.join(rover_sim_share, 'config', 'bridge_gz.yaml')
    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world, model=model)

    path_bridge_config = f"/tmp/bridge_{world}_{model}_urdf.yaml"

    with open(path_bridge_config, "w") as f:
        f.write(rendered)

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={path_bridge_config}'],
        output='screen',
    )]


def make_gazebo_launch(rover_sim_share: str) -> IncludeLaunchDescription:
    world_file = PathJoinSubstitution([
        rover_sim_share, 'worlds', LaunchConfiguration('world_name')
    ])
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    return IncludeLaunchDescription(source, launch_arguments={
        'gz_args': ['-r ', world_file, '.sdf'],
        'on_exit_shutdown': 'True',
    }.items())


def make_spawn_node() -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('model_name'),
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '3.5',
        ],
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    rover_description_share = get_package_share_directory('rover_description')
    rover_sim_share         = get_package_share_directory('rover_sim')

    controllers_yaml_path = os.path.join(rover_sim_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='world_demo'),
        DeclareLaunchArgument('model_name', default_value='indomitus_rover'),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':',
                os.path.dirname(rover_description_share),
            ]
        ),

        make_gazebo_launch(rover_sim_share),

        include_launch('rover_description', 'robot_state_publisher.launch.py', {
            'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
            'xacro_args': f'use_sim:=true controllers_yaml_path:={controllers_yaml_path}',
            'use_sim_time': 'true',
        }),

        OpaqueFunction(function=generate_bridge_config,
                       kwargs={'rover_sim_share': rover_sim_share}),
        make_spawn_node(),

        include_launch('rover_bringup', 'control.launch.py', {
            'use_sim': 'true',
            'controllers_yaml': controllers_yaml_path,
            # 'controllers': 'joint_state_broadcaster swerve_radius_controller odometry_controller diff_bar_effort_controller',
            'controllers': 'joint_state_broadcaster swerve_controller odometry_controller diff_bar_effort_controller',
        }),

        include_launch('rover_localization', 'ekf.launch.py', {
            'use_sim_time': 'true',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),
    ])