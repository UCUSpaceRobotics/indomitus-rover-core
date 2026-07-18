import os
import subprocess
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rover_bringup.launch_utils import include_launch

_GZ_SIM_VERSIONS = ['gz-sim8', 'gz-sim9']


def _detect_gz_sim_pkg() -> str | None:
    for pkg in _GZ_SIM_VERSIONS:
        try:
            subprocess.check_output(['pkg-config', '--exists', pkg], stderr=subprocess.DEVNULL)
            return pkg
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
    return None


def _gz_config_path() -> str:
    gz_pkg = _detect_gz_sim_pkg()
    if gz_pkg:
        try:
            prefix = subprocess.check_output(['pkg-config', '--variable=prefix', gz_pkg], text=True,
                                             stderr=subprocess.DEVNULL).strip()
            return os.path.join(prefix, 'share', 'gz')
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass
    return '/usr/share/gz'


def _gz_sim_version_flag() -> list[str]:
    gz_pkg = _detect_gz_sim_pkg()
    if gz_pkg:
        return ['--force-version', gz_pkg.replace('gz-sim', '')]
    return []


def _gz_ros2_control_lib_dir() -> str:
    share = get_package_share_directory('gz_ros2_control')
    return os.path.normpath(os.path.join(share, '..', '..', 'lib'))


def _gz_transport_env() -> dict:
    return {'GZ_IP': '127.0.0.1', 'GZ_RELAY': '127.0.0.1'}


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


def make_gazebo_server(rover_sim_share: str) -> ExecuteProcess:
    world_name = LaunchConfiguration('world_name')
    return ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '4', [rover_sim_share, '/worlds/', world_name, '.sdf'],
             *_gz_sim_version_flag()],
        output='screen',
        additional_env={
            'GZ_CONFIG_PATH': _gz_config_path(),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': _gz_ros2_control_lib_dir(),
            **_gz_transport_env(),
        },
    )


def make_spawn_node() -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', LaunchConfiguration('model_name'), '-topic', 'robot_description', '-x', '0.0',
                   '-y', '0.0', '-z', '3.5'],
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    rover_description_share = get_package_share_directory('rover_description')
    zed_description_share = get_package_share_directory('zed_description')
    rover_sim_share = get_package_share_directory('rover_sim')

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
                ':',
                os.path.dirname(zed_description_share),
            ]
        ),

        make_gazebo_server(rover_sim_share),
        TimerAction(period=3.0, actions=[ExecuteProcess(cmd=['gz', 'sim', '-g', *_gz_sim_version_flag()], output='screen')]),

        include_launch('rover_description', 'robot_state_publisher.launch.py', {
            'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
            'xacro_args': f'use_sim:=true controllers_yaml_path:={controllers_yaml_path}',
            'use_sim_time': 'true',
        }),

        OpaqueFunction(function=generate_bridge_config, kwargs={'rover_sim_share': rover_sim_share}),
        TimerAction(period=5.0, actions=[make_spawn_node()]),

        include_launch('rover_bringup', 'control.launch.py', {
            'use_sim': 'true',
            'controllers_yaml': controllers_yaml_path,
            'controllers': 'joint_state_broadcaster swerve_controller odometry_controller diff_bar_effort_controller',
        }),

        include_launch('rover_localization', 'ekf.launch.py', {
            'use_sim': 'true',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),
    ])