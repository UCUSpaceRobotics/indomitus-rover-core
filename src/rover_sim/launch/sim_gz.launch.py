import os
from dataclasses import dataclass, field
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rover_bringup.launch_utils import include_launch


@dataclass
class RoverConfig:
    world_name: str = 'world_demo'
    model_name: str = 'indomitus_rover'
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_z: float = 3.5
    controllers: list[str] = field(default_factory=lambda: [
        'joint_state_broadcaster',
        'swerve_controller',
        'odometry_controller',
        'diff_bar_effort_controller',
    ])


def generate_bridge_config(context, rover_sim_share: str) -> list[Node]:
    world = LaunchConfiguration("world_name").perform(context)
    model = LaunchConfiguration("model_name").perform(context)

    template_path = os.path.join(
        rover_sim_share,
        'config',
        'bridge_gz.yaml'
    )
    with open(template_path) as f:
        template = Template(f.read())

    rendered = template.substitute(world=world, model=model)
    path_bridge_config = f"/tmp/bridge_{world}_{model}_urdf.yaml"

    with open(path_bridge_config, "w") as f:
        f.write(rendered)

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={path_bridge_config}'],
        output='screen',
    )]


def make_gazebo_launch(rover_sim_share: str, cfg: RoverConfig) -> IncludeLaunchDescription:
    world_file = os.path.join(rover_sim_share, 'worlds', f'{cfg.world_name}.sdf')
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    return IncludeLaunchDescription(source, launch_arguments={
        'gz_args': f'-r {world_file}',
        'on_exit_shutdown': 'True',
    }.items())


def make_spawn_node(cfg: RoverConfig) -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', cfg.model_name,
            '-topic', 'robot_description',
            '-x', str(cfg.spawn_x),
            '-y', str(cfg.spawn_y),
            '-z', str(cfg.spawn_z),
        ],
        output='screen',
    )

def make_controller_spawners(cfg: RoverConfig, controllers_yaml: str) -> list[Node]:
    return [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--param-file', controllers_yaml],
            output='screen',
        )
        for name in cfg.controllers
    ]

def generate_launch_description() -> LaunchDescription:
    cfg = RoverConfig()
    rover_description_share = get_package_share_directory('rover_description')
    rover_sim_share         = get_package_share_directory('rover_sim')

    controllers_yaml = os.path.join(rover_sim_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value=cfg.world_name),
        DeclareLaunchArgument('model_name', default_value=cfg.model_name),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':',
                os.path.dirname(rover_description_share),
            ]
        ),

        make_gazebo_launch(rover_sim_share, cfg),

        include_launch('rover_description', 'robot_state_publisher.launch.py', {
            'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
            'xacro_args': 'use_sim:=true',
            'use_sim_time': 'true',
        }),

        OpaqueFunction(function=generate_bridge_config,
                       kwargs={'rover_sim_share': rover_sim_share}),
        make_spawn_node(cfg),

        *make_controller_spawners(cfg, controllers_yaml),

        include_launch('rover_localization', 'ekf.launch.py', {
            'use_sim_time': 'true',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),
    ])