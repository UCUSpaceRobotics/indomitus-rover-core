"""
Rover with the arm mounted, in one Gazebo model, with two independent
ros2_control stacks:

  /rover  robot_state_publisher (rover.xacro), /rover/controller_manager,
          twist_mux, drive_power — same as rover_sim/sim_gz.launch.py
  /arm    arm_bringup/stack.launch.py with arm_on_rover.urdf.xacro:
          robot_state_publisher, /arm/controller_manager, move_group, servo

Both robot_state_publishers publish into the one global /tf tree
(base_link -> ... -> main_body_link from /rover, main_body_link -> arm_* from
/arm). The spawned model (indomitus_gz.urdf.xacro) has no
robot_state_publisher of its own and carries one gz_ros2_control plugin per
namespace; each plugin waits for its namespace's robot_description, so the
model is spawned only after both robot_state_publishers are started.
"""

import importlib.util
import os
import shutil
import tempfile
from string import Template

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from rover_bringup.launch_utils import include_launch

# Fixed: indomitus_gz.urdf.xacro hardcodes these as its plugins' namespaces.
ROVER_NAMESPACE = "rover"


def load_rover_sim_launch(rover_sim_share: str):
    """Reuses sim_gz.launch.py's helpers (world/map setup, Gazebo, controllers)."""
    path = os.path.join(rover_sim_share, "launch", "sim_gz.launch.py")
    spec = importlib.util.spec_from_file_location("rover_sim_gz_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_xacro_args(text: str) -> dict:
    return dict(arg.split(":=", 1) for arg in text.split() if ":=" in arg)


def generate_bridges(context, rover_sim_share: str, arm_sim_share: str, launch_tmp_dir: str) -> list:
    world = LaunchConfiguration("world_name").perform(context)
    model = LaunchConfiguration("model_name").perform(context)

    # Rover bridge, including /clock; topics land under /rover as in sim_gz.
    with open(os.path.join(rover_sim_share, "config", "bridge_gz.yaml")) as f:
        rendered = Template(f.read()).substitute(world=world, model=model)
    bridge_yaml_path = os.path.join(launch_tmp_dir, f"bridge_{world}_{model}.yaml")
    with open(bridge_yaml_path, "w") as f:
        f.write(rendered)

    rover_bridge = GroupAction([
        PushRosNamespace(ROVER_NAMESPACE),
        Node(
            package="ros_gz_bridge",
            name="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"use_sim_time": True}],
            arguments=["--ros-args", "-p", f"config_file:={bridge_yaml_path}"],
            output="screen",
        ),
    ])

    arm_camera_bridge = Node(
        package="ros_gz_bridge",
        name="arm_camera_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(arm_sim_share, "config", "gz_bridge.yaml"),
            "use_sim_time": True,
        }],
        output="screen",
        condition=IfCondition(LaunchConfiguration("arm_camera")),
    )

    return [rover_bridge, arm_camera_bridge]


def make_rover_rsp(context, rover_description_share: str, controllers_yaml_path: str) -> list:
    extra_xacro_args = LaunchConfiguration("extra_xacro_args").perform(context)
    xacro_args = (
        f"use_sim:=true controllers_yaml_path:={controllers_yaml_path} "
        f"rover_namespace:={ROVER_NAMESPACE} {extra_xacro_args}"
    )
    return [GroupAction([
        PushRosNamespace(ROVER_NAMESPACE),
        include_launch("rover_description", "robot_state_publisher.launch.py", {
            "xacro_file": os.path.join(rover_description_share, "urdf", "rover.xacro"),
            "xacro_args": xacro_args,
            "use_sim_time": "true",
        }),
    ])]


def make_rover_control(context, controllers_yaml_path: str, base_controllers: list) -> list:
    """sim_gz.launch.py's controller set, with a longer switch timeout: the
    simulation is held while the second gz_ros2_control plugin starts up."""
    swerve_controller = LaunchConfiguration("swerve_controller").perform(context)
    return [GroupAction([
        PushRosNamespace(ROVER_NAMESPACE),
        include_launch("rover_bringup", "control.launch.py", {
            "use_sim": "true",
            "controllers_yaml": controllers_yaml_path,
            "controllers": f"{swerve_controller} " + " ".join(base_controllers),
            "inactive_controllers": "",
            "spawner_args": "--switch-timeout 60",
        }),
    ])]


def make_gz_model_spawn(context, indomitus_description_share: str, launch_tmp_dir: str) -> list:
    """Renders indomitus_gz.urdf.xacro to a file and spawns it after spawn_delay."""
    mappings = parse_xacro_args(LaunchConfiguration("extra_xacro_args").perform(context))
    mappings.update({
        "end_effector": LaunchConfiguration("end_effector").perform(context),
        "camera": LaunchConfiguration("arm_camera").perform(context),
    })
    doc = xacro.process_file(
        os.path.join(indomitus_description_share, "urdf", "indomitus_gz.urdf.xacro"),
        mappings=mappings,
    )
    model_path = os.path.join(launch_tmp_dir, "indomitus_gz.urdf")
    with open(model_path, "w") as f:
        f.write(doc.toxml())

    spawn = Node(
        package="ros_gz_sim",
        name="indomitus_spawner",
        executable="create",
        arguments=[
            "-name", LaunchConfiguration("model_name"),
            "-file", model_path,
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
        ],
        output="screen",
    )
    return [TimerAction(period=LaunchConfiguration("spawn_delay"), actions=[spawn])]


# --- Switch panel, ported unchanged from rover_sim/sim_gz_full.launch.py ---

def generate_panel_bridge_config(context, panel_sim_share: str, launch_tmp_dir: str) -> list[Node]:
    world = LaunchConfiguration("world_name").perform(context)

    template_path = os.path.join(panel_sim_share, "config", "gz_bridge.yaml")
    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world)

    path_bridge_config = os.path.join(launch_tmp_dir, f"panel_bridge_{world}.yaml")
    with open(path_bridge_config, "w") as f:
        f.write(rendered)

    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="panel",
        arguments=["--ros-args", "-p", f"config_file:={path_bridge_config}"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )]


def make_panel_nodes(panel_description_share: str) -> list:
    xacro_file = os.path.join(panel_description_share, "urdf", "panel_standalone.urdf.xacro")

    # panel_x/y/z/yaw are the same launch args passed to panel_spawn below --
    # the published world -> panel_base_link TF must match where the model
    # is actually spawned in Gazebo, or perception/planning against the
    # panel will be working off a stale pose.
    panel_description_content = ParameterValue(
        Command([
            "xacro ", xacro_file, " sim:=true",
            " panel_x:=", LaunchConfiguration("panel_x"),
            " panel_y:=", LaunchConfiguration("panel_y"),
            " panel_z:=", LaunchConfiguration("panel_z"),
            " panel_yaw:=", LaunchConfiguration("panel_yaw"),
        ]),
        value_type=str,
    )

    # frame_prefix makes the published frame IDs actually be panel/* (the
    # namespace alone only namespaces topics, not tf2 frame_id strings), and
    # remapping tf/tf_static back onto the global topics puts those prefixed
    # frames on the one shared TF tree instead of an isolated panel/tf that
    # no other node's TransformListener would ever see by default.
    panel_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="panel",
        output="screen",
        parameters=[{
            "robot_description": panel_description_content,
            "use_sim_time": True,
            "frame_prefix": "panel/",
        }],
        remappings=[
            ("/panel/tf", "/tf"),
            ("/panel/tf_static", "/tf_static"),
        ],
    )

    panel_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "indomitus_panel",
            "-topic", "panel/robot_description",
            "-x", LaunchConfiguration("panel_x"),
            "-y", LaunchConfiguration("panel_y"),
            "-z", LaunchConfiguration("panel_z"),
            "-Y", LaunchConfiguration("panel_yaw"),
        ],
        output="screen",
    )

    return [panel_state_publisher, panel_spawn]


def generate_launch_description() -> LaunchDescription:
    rover_description_share = get_package_share_directory("rover_description")
    zed_description_share = get_package_share_directory("zed_description")
    rover_sim_share = get_package_share_directory("rover_sim")
    arm_description_share = get_package_share_directory("arm_description")
    arm_sim_share = get_package_share_directory("arm_sim")
    indomitus_description_share = get_package_share_directory("indomitus_description")
    panel_description_share = get_package_share_directory("panel_description")
    panel_sim_share = get_package_share_directory("panel_sim")

    rover_sim = load_rover_sim_launch(rover_sim_share)
    rover_sim.setup_environment()
    rover_sim.check_git_lfs(rover_sim_share)

    controllers_yaml_path = os.path.join(rover_sim_share, "config", "controllers.yaml")
    launch_tmp_dir = tempfile.mkdtemp(prefix="indomitus_sim_")

    def cleanup_tmp_dir(context):
        if os.path.exists(launch_tmp_dir):
            shutil.rmtree(launch_tmp_dir)

    resource_path = os.pathsep.join([
        os.path.dirname(rover_description_share),
        os.path.dirname(zed_description_share),
        os.path.dirname(arm_description_share),
        os.path.dirname(panel_description_share),
        launch_tmp_dir,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "swerve_controller",
            default_value=rover_sim.DEFAULT_SWERVE_CONTROLLER,
            choices=list(rover_sim.SWERVE_CONTROLLERS),
            description="Which swerve controller to spawn active."),
        DeclareLaunchArgument(
            "world_name", default_value="mars_yard", choices=rover_sim.SUPPORTED_WORLDS,
            description="Gazebo world from rover_sim/worlds (without .sdf)."),
        DeclareLaunchArgument(
            "map_year", default_value="2026", choices=rover_sim.SUPPORTED_MAP_YEARS,
            description="mars_yard map year."),
        DeclareLaunchArgument(
            "map_resolution", default_value="high", choices=rover_sim.SUPPORTED_MAP_RESOLUTIONS,
            description="mars_yard (2025) mesh resolution; ignored for 2026."),
        DeclareLaunchArgument("model_name", default_value="indomitus"),
        DeclareLaunchArgument("spawn_delay", default_value="5.0",
                              description="Seconds to wait (for both robot_state_publishers) before spawning."),
        DeclareLaunchArgument("spawn_x", default_value=f"{rover_sim.DEFAULT_SPAWN_X}"),
        DeclareLaunchArgument("spawn_y", default_value=f"{rover_sim.DEFAULT_SPAWN_Y}"),
        DeclareLaunchArgument("spawn_z", default_value=f"{rover_sim.DEFAULT_SPAWN_Z}"),
        DeclareLaunchArgument("extra_xacro_args", default_value="",
                              description="Extra rover xacro flags (e.g. lidar_simulate_scan:=true)."),
        DeclareLaunchArgument(
            "end_effector", default_value="jaw",
            description="'jaw', 'other_tool', or 'drill_sampling' — see arm_macro.xacro"),
        DeclareLaunchArgument("arm_camera", default_value="true",
                              description="Enable the arm wrist camera render/bridge"),
        DeclareLaunchArgument("spawn_panel", default_value="false",
                              description="Also spawn the switch panel task board in the same world"),
        DeclareLaunchArgument("panel_x", default_value="2.0"),
        DeclareLaunchArgument("panel_y", default_value="0.0"),
        DeclareLaunchArgument("panel_z", default_value="0.5"),
        DeclareLaunchArgument("panel_yaw", default_value="3.14159"),

        AppendEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path),
        AppendEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=resource_path),

        # 1. Gazebo + bridges
        OpaqueFunction(function=rover_sim.setup_dynamic_map, kwargs={
            "rover_sim_share": rover_sim_share,
            "launch_tmp_dir": launch_tmp_dir,
            "world_name_sub": LaunchConfiguration("world_name"),
            "map_resolution_sub": LaunchConfiguration("map_resolution"),
            "map_year_sub": LaunchConfiguration("map_year"),
        }),
        rover_sim.make_gazebo_launch(rover_sim_share, LaunchConfiguration("world_name")),
        OpaqueFunction(function=generate_bridges, kwargs={
            "rover_sim_share": rover_sim_share,
            "arm_sim_share": arm_sim_share,
            "launch_tmp_dir": launch_tmp_dir,
        }),

        # 2. /rover robot_state_publisher
        OpaqueFunction(function=make_rover_rsp, kwargs={
            "rover_description_share": rover_description_share,
            "controllers_yaml_path": controllers_yaml_path,
        }),

        # 3. /arm stack - the same file the real arm runs. Own group so the
        # namespace and its launch_arguments don't leak into what follows.
        GroupAction([
            PushRosNamespace("arm"),
            include_launch("arm_bringup", "stack.launch.py", {
                "description_file": os.path.join(
                    indomitus_description_share, "urdf", "arm_on_rover.urdf.xacro"),
                "use_sim": "true",
                "camera": LaunchConfiguration("arm_camera"),
                "end_effector": LaunchConfiguration("end_effector"),
            }),
        ]),

        # 4. The one Gazebo model, after both robot_state_publishers are up.
        OpaqueFunction(function=make_gz_model_spawn, kwargs={
            "indomitus_description_share": indomitus_description_share,
            "launch_tmp_dir": launch_tmp_dir,
        }),

        # 5. Rover controllers (spawners resolve to /rover/controller_manager),
        # twist_mux and drive_power, as in sim_gz.launch.py.
        OpaqueFunction(function=make_rover_control, kwargs={
            "controllers_yaml_path": controllers_yaml_path,
            "base_controllers": rover_sim.BASE_CONTROLLERS,
        }),
        GroupAction([
            PushRosNamespace(ROVER_NAMESPACE),
            include_launch("rover_bringup", "twist_mux.launch.py", {
                "use_sim_time": "true",
            }),
        ]),
        GroupAction([
            PushRosNamespace(ROVER_NAMESPACE),
            include_launch("rover_power", "drive_power.launch.py", {
                "use_sim_time": "true",
                "controller_name": LaunchConfiguration("swerve_controller"),
            }),
        ]),

        # 6. Optional switch panel, in its own `panel` namespace.
        GroupAction(
            actions=[
                *make_panel_nodes(panel_description_share),
                OpaqueFunction(function=generate_panel_bridge_config, kwargs={
                    "panel_sim_share": panel_sim_share,
                    "launch_tmp_dir": launch_tmp_dir,
                }),
            ],
            condition=IfCondition(LaunchConfiguration("spawn_panel")),
        ),

        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_tmp_dir)])),
    ])
