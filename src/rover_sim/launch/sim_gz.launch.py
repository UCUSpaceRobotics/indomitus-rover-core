import os
import shutil
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


def setup_environment() -> list:
    """Auto-detects host OS graphics and network issues, applying fixes instantly to the Python environment."""

    # 1. IPC / Networking Fix (Forces local ZeroMQ. Prevents GUI black screen & spawner deaf loops)
    os.environ['GZ_IP'] = '127.0.0.1'
    os.environ['IGN_IP'] = '127.0.0.1'

    # 2. Wayland GUI Crash Workaround (Forces X11 backend for Qt)
    if os.environ.get('XDG_SESSION_TYPE', '') == 'wayland' or os.environ.get('WAYLAND_DISPLAY'):
        os.environ['QT_QPA_PLATFORM'] = 'xcb'

    # 3. Nvidia Optimus & EGL Deadlock Workaround (Prevents EGL dri2 crash)
    nvidia_egl_path = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    if os.path.exists(nvidia_egl_path):
        os.environ['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'
        os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = nvidia_egl_path


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
            '-x', '0.0', '-y', '3.0', '-z', '1.0',
        ],
        output='screen',
    )


def setup_dynamic_map(context, rover_sim_share: str) -> list:
    resolution = LaunchConfiguration("map_resolution").perform(context)

    source_meshes_dir = os.path.join(rover_sim_share, 'models', 'mars_yard_2025', 'meshes')
    tmp_model_dir = '/tmp/sim_models/mars_yard_2025'
    tmp_meshes_dir = os.path.join(tmp_model_dir, 'meshes')

    os.makedirs(tmp_meshes_dir, exist_ok=True)

    selected_obj = os.path.join(source_meshes_dir, f'mars_yard_2025_{resolution}_resolution.obj')
    target_obj = os.path.join(tmp_meshes_dir, 'mars_yard_2025.obj')

    if os.path.exists(selected_obj):
        shutil.copy(selected_obj, target_obj)
    else:
        raise RuntimeError(f"Resolution file not found: {selected_obj}")

    shutil.copy(
        os.path.join(rover_sim_share, 'models', 'mars_yard_2025', 'model.config'),
        os.path.join(tmp_model_dir, 'model.config')
    )

    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="mars_yard_2025">
    <static>true</static>
    <link name="map_link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://mars_yard_2025/meshes/mars_yard_2025.obj</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://mars_yard_2025/meshes/mars_yard_2025.obj</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.6 0.3 0.1 1</ambient>
          <diffuse>0.7 0.35 0.15 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    with open(os.path.join(tmp_model_dir, 'model.sdf'), 'w') as f:
        f.write(sdf_content)

    return []


def generate_launch_description() -> LaunchDescription:
    setup_environment()

    rover_description_share = get_package_share_directory('rover_description')
    rover_sim_share         = get_package_share_directory('rover_sim')

    controllers_yaml_path = os.path.join(rover_sim_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='world_demo'),
        DeclareLaunchArgument('model_name', default_value='indomitus_rover'),

        DeclareLaunchArgument(
            'map_resolution',
            default_value='high',
            description='Options: low, medium, high'
        ),

        OpaqueFunction(function=setup_dynamic_map, kwargs={'rover_sim_share': rover_sim_share}),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ':',
                os.path.dirname(rover_description_share),
                ':',
                '/tmp/sim_models',
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
            'controllers': 'joint_state_broadcaster swerve_controller odometry_controller diff_bar_effort_controller',
        }),

        include_launch('rover_localization', 'ekf.launch.py', {
            'use_sim_time': 'true',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),
    ])
