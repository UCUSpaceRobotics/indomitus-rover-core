import os
import subprocess
from dataclasses import dataclass
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    OpaqueFunction,
    TimerAction,
    EmitEvent,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rover_bringup.launch_utils import include_launch


@dataclass
class RoverConfig:
    world_name: str = 'nav2_test_world'
    model_name: str = 'indomitus_rover'
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_z: float = 0.5


# ROS 2 Humble -> Gz Sim 8 (Harmonic)
# ROS 2 Jazzy  -> Gz Sim 9 (Ionic)
# NOTE: Jazzy support requires gz_ros2_control and ros_gz_bridge also built
# against Gz Sim 9. Adding gz-sim9 here is not sufficient on its own.
_GZ_SIM_VERSIONS = ['gz-sim8', 'gz-sim9']


def _gz_ros2_control_lib_dir() -> str:
    share = get_package_share_directory('gz_ros2_control')
    return os.path.normpath(os.path.join(share, '..', '..', 'lib'))


def _detect_gz_sim_pkg() -> str | None:
    """Return the pkg-config name of the first installed Gz Sim version."""
    for pkg in _GZ_SIM_VERSIONS:
        try:
            subprocess.check_output(
                ['pkg-config', '--exists', pkg],
                stderr=subprocess.DEVNULL,
            )
            return pkg
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
    return None


def _gz_config_path() -> str:
    gz_pkg = _detect_gz_sim_pkg()
    if gz_pkg:
        try:
            prefix = subprocess.check_output(
                ['pkg-config', '--variable=prefix', gz_pkg],
                text=True, stderr=subprocess.DEVNULL,
            ).strip()
            return os.path.join(prefix, 'share', 'gz')
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass
    return '/usr/share/gz'


def _gz_sim_version_flag() -> list[str]:
    """Pass --force-version to prevent gz from picking the wrong installed version."""
    gz_pkg = _detect_gz_sim_pkg()
    if gz_pkg:
        return ['--force-version', gz_pkg.replace('gz-sim', '')]
    return []


def _gz_transport_env() -> dict:
    """Force TCP transport. Required in distrobox where shared memory is unavailable."""
    return {
        'GZ_IP': '127.0.0.1',
        'GZ_RELAY': '127.0.0.1',
    }


def generate_bridge_config(context) -> list[Node]:
    world = LaunchConfiguration('world_name').perform(context)
    model = LaunchConfiguration('model_name').perform(context)

    template_path = os.path.join(
        get_package_share_directory('rover_sim'),
        'config',
        'bridge_gz_nav2.yaml',
    )
    with open(template_path) as f:
        template = Template(f.read())

    rendered = template.substitute(world=world, model=model)
    path_bridge_config = f'/tmp/bridge_{world}_{model}.yaml'

    with open(path_bridge_config, 'w') as f:
        f.write(rendered)

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={path_bridge_config}'],
        output='screen',
    )]


def make_robot_description(rover_sim_share: str) -> Command:
    return Command([
        'xacro ',
        os.path.join(rover_sim_share, 'urdf', 'rover_sim.urdf.xacro'),
        ' use_nav:=true lidar_simulate_scan:=true',
        ' use_nav:=true stereo_camera_simulate_pointcloud:=true',
    ])


def make_gazebo_server(rover_sim_share: str) -> ExecuteProcess:
    """Launch server only. GUI starts separately to avoid the starting_world race."""
    world_name = LaunchConfiguration('world_name')
    return ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-r', '-v', '4',
            [rover_sim_share, '/worlds/', world_name, '.sdf'],
            *_gz_sim_version_flag(),
        ],
        output='screen',
        additional_env={
            'GZ_CONFIG_PATH': _gz_config_path(),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': _gz_ros2_control_lib_dir(),
            **_gz_transport_env(),
        },
    )


def make_gazebo_gui() -> ExecuteProcess:
    return ExecuteProcess(
        cmd=['gz', 'sim', '-g', *_gz_sim_version_flag()],
        output='screen',
        additional_env={
            'GZ_CONFIG_PATH': _gz_config_path(),
            **_gz_transport_env(),
        },
    )


def make_spawn_node(cfg: RoverConfig) -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('model_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    cfg = RoverConfig()
    rover_description_share = get_package_share_directory('rover_description')
    zed_description_share = get_package_share_directory('zed_description')
    rover_sim_share = get_package_share_directory('rover_sim')

    rover_localization_share = get_package_share_directory('rover_localization')

    robot_description = make_robot_description(rover_sim_share)
    gz_server = make_gazebo_server(rover_sim_share)

    robot_localization = include_launch('rover_localization', 'ekf.launch.py', {
        'use_sim': 'true'
    })

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value=cfg.world_name),
        DeclareLaunchArgument('model_name', default_value=cfg.model_name),
        DeclareLaunchArgument('spawn_x',    default_value=str(cfg.spawn_x)),
        DeclareLaunchArgument('spawn_y',    default_value=str(cfg.spawn_y)),
        DeclareLaunchArgument('spawn_z',    default_value=str(cfg.spawn_z)),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':',
                os.path.dirname(rover_description_share),
            ],
        ),
        SetEnvironmentVariable(name='GZ_IP',    value='127.0.0.1'),
        SetEnvironmentVariable(name='GZ_RELAY', value='127.0.0.1'),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=_gz_ros2_control_lib_dir(),
        ),

        gz_server,

        TimerAction(period=3.0, actions=[make_gazebo_gui()]),

        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_server,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        OpaqueFunction(function=generate_bridge_config),

        TimerAction(
            period=5.0,
            actions=[make_spawn_node(cfg)],
        ),

        # Controllers need gz_ros2_control to load inside the spawned model first.
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['swerve_controller'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['odometry_controller'],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_bar_effort_controller'],
                    output='screen',
                ),

                robot_localization,
                include_launch('rover_navigation', 'navigation.launch.py', {'use_sim_time': 'true'}),
                include_launch('rover_localization', 'slam.launch.py', {'use_sim_time': 'true'}),
            ],
        ),
    ])