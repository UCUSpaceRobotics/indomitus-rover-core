import os
import shutil
import tempfile
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
    NotEqualsSubstitution,
)
from launch_ros.actions import Node
from rover_bringup.launch_utils import include_launch


# List of available worlds.
SUPPORTED_WORLDS = ["mars_yard", "nav2_test_world"]

# List of worlds that utilize the dynamic resolution map loading feature.
# For any world not in this list, the map_resolution parameter is safely ignored.
SUPPORTED_RESOLUTION_WORLDS = ["mars_yard"]

# Default coordinates for rover to spawn
DEFAULT_SPAWN_X = 0.0
DEFAULT_SPAWN_Y = 3.0
DEFAULT_SPAWN_Z = 0.5


def setup_environment() -> list:
    """Auto-detects host OS graphics and network issues, applying fixes instantly to the Python environment."""

    # IPC / Networking Fix (Forces local ZeroMQ. Prevents GUI black screen & spawner deaf loops)
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["IGN_IP"] = "127.0.0.1"

    # Wayland GUI Crash Workaround (Forces X11 backend for Qt)
    if os.environ.get("XDG_SESSION_TYPE", "") == "wayland" or os.environ.get("WAYLAND_DISPLAY"):
        os.environ["QT_QPA_PLATFORM"] = "xcb"

    # Nvidia Optimus & EGL Deadlock Workaround (Prevents EGL dri2 crash)
    nvidia_egl_path = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(nvidia_egl_path):
        os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_egl_path


def generate_bridge_and_rsp(context, rover_sim_share: str, rover_description_share: str, controllers_yaml_path: str, launch_tmp_dir: str, world_name_sub, model_name_sub, extra_xacro_args_sub) -> list:
    """Dynamically handles configuration switching for the bridge and robot description."""
    world = world_name_sub.perform(context)
    model = model_name_sub.perform(context)
    extra_xacro_args = extra_xacro_args_sub.perform(context)

    template_path = os.path.join(rover_sim_share, "config", "bridge_gz.yaml")

    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world, model=model)

    bridge_yaml_path = os.path.join(launch_tmp_dir, f"bridge_{world}_{model}.yaml")
    with open(bridge_yaml_path, "w") as f:
        f.write(rendered)

    bridge_node = Node(
        package="ros_gz_bridge",
        name="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_yaml_path}"],
        output="screen",
    )

    xacro_args = f"use_sim:=true controllers_yaml_path:={controllers_yaml_path} {extra_xacro_args}"

    rsp_launch = include_launch("rover_description", "robot_state_publisher.launch.py", {
        "xacro_file": os.path.join(rover_description_share, "urdf", "rover.xacro"),
        "xacro_args": xacro_args,
        "use_sim_time": "true",
    })

    return [bridge_node, rsp_launch]


def make_gazebo_launch(rover_sim_share: str, world_name_sub) -> IncludeLaunchDescription:
    """Abstracts Gazebo startup through the official ros_gz_sim package."""
    world_file = PathJoinSubstitution([
        rover_sim_share, "worlds", world_name_sub
    ])

    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    )
    return IncludeLaunchDescription(source, launch_arguments={
        "gz_args": ["-r ", world_file, ".sdf"],
        "on_exit_shutdown": "True",
    }.items())


def make_spawn_node(model_name_sub, spawn_x_sub, spawn_y_sub, spawn_z_sub) -> Node:
    return Node(
        package="ros_gz_sim",
        name="rover_spawner",
        executable="create",
        arguments=[
            "-name", model_name_sub,
            "-topic", "robot_description",
            "-x", spawn_x_sub,
            "-y", spawn_y_sub,
            "-z", spawn_z_sub
        ],
        output="screen",
    )


def setup_dynamic_map(context, rover_sim_share: str, launch_tmp_dir: str, world_name_sub, map_resolution_sub) -> list:
    world = world_name_sub.perform(context)

    if world not in SUPPORTED_RESOLUTION_WORLDS:
        return []

    resolution = map_resolution_sub.perform(context)
    source_meshes_dir = os.path.join(rover_sim_share, "models", "mars_yard_2025", "meshes")

    tmp_model_dir = os.path.join(launch_tmp_dir, "mars_yard_2025")
    tmp_meshes_dir = os.path.join(tmp_model_dir, "meshes")

    os.makedirs(tmp_meshes_dir, exist_ok=True)

    selected_obj = os.path.join(source_meshes_dir, f"mars_yard_2025_{resolution}_resolution.obj")
    target_obj = os.path.join(tmp_meshes_dir, "mars_yard_2025.obj")

    if os.path.exists(selected_obj):
        shutil.copy(selected_obj, target_obj)
    else:
        raise RuntimeError(f"Resolution file not found: {selected_obj}")

    shutil.copy(
        os.path.join(rover_sim_share, "models", "mars_yard_2025", "model.config"),
        os.path.join(tmp_model_dir, "model.config")
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
        <cast_shadows>false</cast_shadows>
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

    with open(os.path.join(tmp_model_dir, "model.sdf"), "w") as f:
        f.write(sdf_content)

    return []


def generate_launch_description() -> LaunchDescription:
    setup_environment()

    rover_description_share = get_package_share_directory("rover_description")
    zed_description_share = get_package_share_directory("zed_description")
    rover_sim_share = get_package_share_directory("rover_sim")

    controllers_yaml_path = os.path.join(rover_sim_share, "config", "controllers.yaml")
    LAUNCH_TMP_DIR = tempfile.mkdtemp(prefix="rover_sim_")

    def cleanup_tmp_dir(context):
        if os.path.exists(LAUNCH_TMP_DIR):
            shutil.rmtree(LAUNCH_TMP_DIR)

    cleanup_handler = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_tmp_dir)]))

    is_world_name_present = NotEqualsSubstitution(LaunchConfiguration("world_name"), "")
    is_map_resolution_present = NotEqualsSubstitution(LaunchConfiguration("map_resolution"), "")
    is_model_name_present = NotEqualsSubstitution(LaunchConfiguration("model_name"), "")
    is_spawn_x_present = NotEqualsSubstitution(LaunchConfiguration("spawn_x"), "")
    is_spawn_y_present = NotEqualsSubstitution(LaunchConfiguration("spawn_y"), "")
    is_spawn_z_present = NotEqualsSubstitution(LaunchConfiguration("spawn_z"), "")
    extra_xacro_args_present = NotEqualsSubstitution(LaunchConfiguration("extra_xacro_args"), "")

    world_name_sub = IfElseSubstitution(is_world_name_present, if_value=LaunchConfiguration("world_name"), else_value="mars_yard")
    map_resolution_sub = IfElseSubstitution(is_map_resolution_present, if_value=LaunchConfiguration("map_resolution"), else_value="high")
    model_name_sub = IfElseSubstitution(is_model_name_present, if_value=LaunchConfiguration("model_name"), else_value="indomitus_rover")
    spawn_x_sub = IfElseSubstitution(is_spawn_x_present, if_value=LaunchConfiguration("spawn_x"), else_value=f"{DEFAULT_SPAWN_X}")
    spawn_y_sub = IfElseSubstitution(is_spawn_y_present, if_value=LaunchConfiguration("spawn_y"), else_value=f"{DEFAULT_SPAWN_Y}")
    spawn_z_sub = IfElseSubstitution(is_spawn_z_present, if_value=LaunchConfiguration("spawn_z"), else_value=f"{DEFAULT_SPAWN_Z}")
    extra_xacro_args_sub = IfElseSubstitution(extra_xacro_args_present, if_value=LaunchConfiguration("extra_xacro_args"), else_value="")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_name",
            default_value="mars_yard",
            description=f"Gazebo world file to load (without .sdf extension). Available options: {SUPPORTED_WORLDS}."
        ),
        DeclareLaunchArgument(
            "map_resolution",
            default_value="high",
            description=f"Options: low, medium, high. Dynamically applies only to: {SUPPORTED_RESOLUTION_WORLDS}. Ignored for other worlds."
        ),
        DeclareLaunchArgument("model_name", default_value="indomitus_rover", description="The name assigned to the robot model."),
        DeclareLaunchArgument("spawn_delay", default_value="5.0", description="Time (in seconds) to wait before spawning the robot."),
        DeclareLaunchArgument("extra_xacro_args", default_value="", description="Additional flags to pass to the URDF xacro compiler."),
        DeclareLaunchArgument("spawn_x", default_value=f"{DEFAULT_SPAWN_X}", description="Initial X coordinate (in meters)."),
        DeclareLaunchArgument("spawn_y", default_value=f"{DEFAULT_SPAWN_Y}", description="Initial Y coordinate (in meters)."),
        DeclareLaunchArgument("spawn_z", default_value=f"{DEFAULT_SPAWN_Z}", description="Initial Z coordinate (in meters)."),

        # 1. Support modern Gazebo (Harmonic/Ionic)
        AppendEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=f"{os.path.dirname(rover_description_share)}:{os.path.dirname(zed_description_share)}:{LAUNCH_TMP_DIR}",
        ),

        # 2. Support Ignition Gazebo (Fortress)
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=f"{os.path.dirname(rover_description_share)}:{os.path.dirname(zed_description_share)}:{LAUNCH_TMP_DIR}",
        ),

        OpaqueFunction(
            function=setup_dynamic_map,
            kwargs={
                "rover_sim_share": rover_sim_share,
                "launch_tmp_dir": LAUNCH_TMP_DIR,
                "world_name_sub": world_name_sub,
                "map_resolution_sub": map_resolution_sub
            }
        ),

        make_gazebo_launch(rover_sim_share, world_name_sub),

        OpaqueFunction(
            function=generate_bridge_and_rsp,
            kwargs={
                "rover_sim_share": rover_sim_share,
                "rover_description_share": rover_description_share,
                "controllers_yaml_path": controllers_yaml_path,
                "launch_tmp_dir": LAUNCH_TMP_DIR,
                "world_name_sub": world_name_sub,
                "model_name_sub": model_name_sub,
                "extra_xacro_args_sub": extra_xacro_args_sub
            }
        ),

        TimerAction(
            period=LaunchConfiguration("spawn_delay"),
            actions=[make_spawn_node(model_name_sub, spawn_x_sub, spawn_y_sub, spawn_z_sub)]
        ),

        include_launch("rover_bringup", "control.launch.py", {
            "use_sim_time": "true",
            "controllers_yaml": controllers_yaml_path,
            "controllers": "joint_state_broadcaster swerve_controller odometry_controller diff_bar_effort_controller",
        }),

        include_launch("rover_localization", "ekf.launch.py", {
            "use_sim_time": "true",
        }),

        include_launch("rover_bringup", "twist_mux.launch.py", {
            "use_sim_time": "true",
        }),

        # Trigger folder cleanup safely upon launch termination
        cleanup_handler,
    ])
