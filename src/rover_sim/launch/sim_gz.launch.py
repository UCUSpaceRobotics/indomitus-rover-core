import os
import tempfile
import atexit
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from rover_bringup.launch_utils import include_launch


def generate_bridge_and_rsp(context, rover_sim_share: str, rover_description_share: str, controllers_yaml_path: str) -> list:
    """Dynamically handles configuration switching for the bridge and robot description."""
    world = LaunchConfiguration('world_name').perform(context)
    model = LaunchConfiguration('model_name').perform(context)
    extra_xacro_args = LaunchConfiguration('extra_xacro_args').perform(context)

    template_path = os.path.join(rover_sim_share, 'config', 'bridge_gz.yaml')
    
    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world, model=model)

    tmp_file = tempfile.NamedTemporaryFile(
        mode='w', delete=False, prefix=f'bridge_{world}_{model}_', suffix='.yaml'
    )
    tmp_file.write(rendered)
    tmp_file.close()
    atexit.register(os.remove, tmp_file.name)

    bridge_node = Node(
        package='ros_gz_bridge',
        name='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={tmp_file.name}'],
        output='screen',
    )

    xacro_args = f'use_sim:=true controllers_yaml_path:={controllers_yaml_path} {extra_xacro_args}'

    rsp_launch = include_launch('rover_description', 'robot_state_publisher.launch.py', {
        'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
        'xacro_args': xacro_args,
        'use_sim_time': 'true',
    })

    return [bridge_node, rsp_launch]


def make_gazebo_launch(rover_sim_share: str) -> IncludeLaunchDescription:
    """Abstracts Gazebo startup through the official ros_gz_sim package."""
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
        name='rover_spawner',
        executable='create',
        arguments=['-name', LaunchConfiguration('model_name'), '-topic', 'robot_description', 
                   '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-z', LaunchConfiguration('spawn_z')],
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    rover_description_share = get_package_share_directory('rover_description')
    zed_description_share = get_package_share_directory('zed_description')
    rover_sim_share = get_package_share_directory('rover_sim')

    controllers_yaml_path = os.path.join(rover_sim_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name', 
            default_value='world_demo', 
            description='Name of the Gazebo world file to load (without the .sdf extension).'
        ),
        DeclareLaunchArgument(
            'model_name', 
            default_value='indomitus_rover', 
            description='The name assigned to the robot model when spawned inside the Gazebo environment.'
        ),
        DeclareLaunchArgument(
            'spawn_delay', 
            default_value='5.0', 
            description='Time (in seconds) to wait before spawning the robot, ensuring the Gazebo world is fully loaded.'
        ),
        DeclareLaunchArgument(
            'extra_xacro_args', 
            default_value='', 
            description="Additional flags to pass to the URDF xacro compiler. Format strictly as space-separated key:=value pairs (e.g., 'use_nav:=true lidar_simulate_scan:=true')."
        ),
        DeclareLaunchArgument(
            'spawn_x', 
            default_value='0.0', 
            description='Initial X coordinate (in meters) for spawning the robot in the global world frame.'
        ),
        DeclareLaunchArgument(
            'spawn_y', 
            default_value='0.0', 
            description='Initial Y coordinate (in meters) for spawning the robot in the global world frame.'
        ),
        DeclareLaunchArgument(
            'spawn_z', 
            default_value='5.0', 
            description='Initial Z coordinate (in meters) for spawning the robot in the global world frame. Set higher than ground level.'
        ),

        # 1. Support modern Gazebo (Harmonic/Ionic)
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f"{os.path.dirname(rover_description_share)}:{os.path.dirname(zed_description_share)}"
        ),

        # 2. Support Ignition Gazebo (Fortress)
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=f"{os.path.dirname(rover_description_share)}:{os.path.dirname(zed_description_share)}"
        ),

        make_gazebo_launch(rover_sim_share),

        OpaqueFunction(
            function=generate_bridge_and_rsp, 
            kwargs={
                'rover_sim_share': rover_sim_share,
                'rover_description_share': rover_description_share,
                'controllers_yaml_path': controllers_yaml_path
            }
        ),

        TimerAction(
            period=LaunchConfiguration('spawn_delay'), 
            actions=[make_spawn_node()]
        ),

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