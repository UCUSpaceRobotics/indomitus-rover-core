import os

from dataclasses import dataclass, field
from string import Template
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


@dataclass
class RoverConfig:
    world_name: str = 'indomitus_world_demo'
    model_name: str = 'indomitus_rover'
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_z: float = 3.5
    controllers: list[str] = field(default_factory=lambda: [
        'joint_state_broadcaster',
        'steering_controller',
        'drive_controller',
        'r_rocker_position_controller',
    ])


def generate_bridge_config(context) -> list[Node]:
    world = LaunchConfiguration("world_name").perform(context)
    model = LaunchConfiguration("model_name").perform(context)

    template_path = os.path.join(
        get_package_share_directory('indomitus_rover_sim'),
        'parameters',
        'bridge_parameters_urdf.yaml'
    )

    with open(template_path) as f:
        template = Template(f.read())

    rendered = template.substitute(
        world=world,
        model=model
    )

    pathBridgeConfig = f"/tmp/bridge_{world}_{model}_urdf.yaml"

    with open(pathBridgeConfig, "w") as f:
        f.write(rendered)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={pathBridgeConfig}',
        ],
        output='screen',
    )

    return [bridge_node]


def controller_spawner(name: str) -> Node:
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[name],
        output='screen',
    )


def make_robot_description(rover_sim_share: str) -> str:
    path = os.path.join(rover_sim_share, 'urdf', 'indomitus_rover_gazebo.urdf.xacro')
    return xacro.process_file(path).toxml()


def make_gazebo_launch(rover_sim_share: str) -> IncludeLaunchDescription:
    world_file = os.path.join(rover_sim_share, 'worlds', 'rover_world.sdf')
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    return IncludeLaunchDescription(source, launch_arguments={
        'gz_args': f'-r -v -v4 {world_file}',
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


def generate_launch_description() -> LaunchDescription:
    cfg = RoverConfig()
    rover_description_share = get_package_share_directory('indomitus_rover_description')
    rover_sim_share = get_package_share_directory('indomitus_rover_sim')

    robot_description = make_robot_description(rover_sim_share)

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value=cfg.world_name),
        DeclareLaunchArgument('model_name', default_value=cfg.model_name),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), ':', os.path.dirname(rover_description_share)]
        ),
        make_gazebo_launch(rover_sim_share),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='indomitus_rover_control',
            executable='rover_kinematics_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('indomitus_rover_control'), 'config', 'rover_geometry.yaml')
            ],
        ),
        OpaqueFunction(function=generate_bridge_config),
        make_spawn_node(cfg),
        *[controller_spawner(c) for c in cfg.controllers],
        Node(package='indomitus_rover_sim', executable='sim_chassis_driver_node', output='screen'),
        Node(package='indomitus_rover_sim', executable='sim_diff_bar_node',   output='screen'),
    ])
