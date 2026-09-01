import contextlib
import os
import random
import sys

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _arg_from_argv(name: str, default: str) -> str:
    """Plain-str mappings force MoveItConfigsBuilder's single-eval xacro
    path — LaunchConfiguration ones desync across sub-launches. Same
    helper/reasoning as arm_bringup/arm.launch.py's own.
    """
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def generate_launch_description() -> LaunchDescription:
    arm_description_dir = get_package_share_directory("arm_description")
    arm_sim_dir = get_package_share_directory("arm_sim")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    panel_description_dir = get_package_share_directory("panel_description")

    xacro_file = os.path.join(arm_description_dir, "urdf", "arm_standalone.urdf.xacro")
    panel_xacro_file = os.path.join(panel_description_dir, "urdf", "panel_standalone.urdf.xacro")
    world_file = os.path.join(arm_sim_dir, "worlds", "empty.sdf")
    bridge_config = os.path.join(arm_sim_dir, "config", "gz_bridge.yaml")
    bridge_config_no_camera = os.path.join(arm_sim_dir, "config", "gz_bridge_no_camera.yaml")

    resource_path_root = os.path.dirname(arm_description_dir)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(filter(
        None, [resource_path_root, os.path.dirname(panel_description_dir), existing_gz_path]))
    ign_resource_path = os.pathsep.join(filter(
        None, [resource_path_root, os.path.dirname(panel_description_dir), existing_ign_path]))

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " sim:=true",
                " camera:=",
                LaunchConfiguration("camera"),
                " end_effector:=",
                LaunchConfiguration("end_effector"),
            ]
        ),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )


    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "indomitus_arm", "-z", "0.3"],
        output="screen",
    )

    # Panel placement, found via a real reachability SEARCH rather than
    # hand-eyeballed vector math (which repeatedly produced unreachable
    # targets in practice — see project_panel_detection_align memory).
    # /tmp/find_reachable_panel2.py called MoveIt's own /compute_ik on a
    # grid of candidate (distance, azimuth, height) panel placements, each
    # oriented to face squarely back at the arm, using the REAL FOV-fit
    # standoff (compute_standoff_distance(), ~0.6m for this panel/camera)
    # AND panel_align_node's actual PANEL_CENTER_LOCAL_OFFSET (the first
    # search version omitted this, which validated a target aimed at the
    # panel's bottom edge instead of its center — the arm reached that
    # target fine, but the camera ended up looking past the panel instead
    # of at it). This is the best of 64 verified-reachable candidates:
    # target tip pose ends up 0.498m from arm_mount_link with 37.8%
    # joint-limit margin to spare (checked via panel_align_node's own
    # margin formula). Panel position here is in Gazebo SPAWN/world
    # coordinates, which is arm_mount_link's TF pose (identity) + 0.3m —
    # the "-z 0.3" spawn_entity offset below is real in Gazebo but never
    # reflected in TF (nothing publishes a world->arm_mount_link
    # transform accounting for it).
    PANEL_X, PANEL_Y, PANEL_Z, PANEL_YAW = "0.450", "0.779423", "0.450", "-0.5236"
    _panel_yaw_f = float(PANEL_YAW)
    PANEL_QZ, PANEL_QW = str(np.sin(_panel_yaw_f / 2)), str(np.cos(_panel_yaw_f / 2))

    # Randomized fresh on every launch (not just once and cached) — per
    # competition rules, any 3 of ArUco IDs {11,13,14,15} may be used, in
    # any of the panel's 3 physical mount positions, so this exercises
    # panel_geometry.py's generalized (ID-agnostic) orientation fit
    # against a genuinely different layout each run instead of always
    # the same one. random.sample already returns non-repeating picks;
    # the assignment to the 3 named roles below is itself the "any
    # position" half of the randomization.
    #
    # An env var (not a DeclareLaunchArgument: this runs as plain Python
    # during launch-description construction, before any launch argument
    # would be resolved) makes a specific layout reproducible on demand —
    # PANEL_MARKER_LAYOUT_SEED=1234 ros2 launch ... replays the exact same
    # choice. The chosen layout is always logged (see
    # panel_pose_fuser_node.py's own startup log) so a failure can be
    # traced back to what was actually in play, seed or not.
    ALLOWED_MARKER_IDS = [11, 13, 14, 15]
    _seed_env = os.environ.get('PANEL_MARKER_LAYOUT_SEED')
    _rng = random.Random(int(_seed_env)) if _seed_env else random.Random()
    marker_id_top_left, marker_id_top_right, marker_id_bottom_left = _rng.sample(
        ALLOWED_MARKER_IDS, 3)

    panel_description_content = ParameterValue(
        Command([
            "xacro ", panel_xacro_file,
            " sim:=true",
            " panel_x:=", PANEL_X, " panel_y:=", PANEL_Y,
            " panel_z:=", PANEL_Z, " panel_yaw:=", PANEL_YAW,
            " marker_id_top_left:=", str(marker_id_top_left),
            " marker_id_top_right:=", str(marker_id_top_right),
            " marker_id_bottom_left:=", str(marker_id_bottom_left),
        ]),
        value_type=str,
    )

    # panel_pose_fuser_node is launched separately (its own terminal, per
    # this repo's usual sim workflow) and has no direct way to see the
    # random choice made above. Rather than rely on the operator copying
    # parameters by hand from console output, write it to a well-known
    # file that panel_pose_fuser_node itself checks at startup (see that
    # file's own matching logic — PANEL_MARKER_LAYOUT_SIM_FILE_TEMPLATE
    # must stay in sync between the two) — so `ros2 run panel_perception
    # panel_pose_fuser_node` with NO extra args just works in sim.
    #
    # Scoped by ROS_DOMAIN_ID (not one bare global path) so two sim
    # instances on the same host with different domain IDs don't clobber
    # each other's layout file. Best-effort cleanup on a clean exit (see
    # delete_marker_layout_file below); the read side additionally
    # ignores this file if it's stale (see panel_pose_fuser_node.py) as a
    # backstop against a crashed run's leftover file being picked up by
    # a later, unrelated one.
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    PANEL_MARKER_LAYOUT_SIM_FILE = f"/tmp/panel_marker_layout_domain{domain_id}.yaml"
    with open(PANEL_MARKER_LAYOUT_SIM_FILE, "w") as f:
        yaml.safe_dump({
            "top_left": marker_id_top_left,
            "top_right": marker_id_top_right,
            "bottom_left": marker_id_bottom_left,
        }, f)

    def _delete_marker_layout_file(event, context):
        with contextlib.suppress(OSError):
            os.remove(PANEL_MARKER_LAYOUT_SIM_FILE)

    delete_marker_layout_file = RegisterEventHandler(
        OnShutdown(on_shutdown=_delete_marker_layout_file)
    )

    panel_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="panel",
        output="screen",
        parameters=[{"robot_description": panel_description_content, "use_sim_time": True,
                     "frame_prefix": "panel/"}],
        remappings=[("/panel/tf", "/tf"), ("/panel/tf_static", "/tf_static")],
        condition=IfCondition(LaunchConfiguration("spawn_panel")),
    )

    # Without this, the panel's own robot_state_publisher (frame_prefix
    # "panel/", rooted at its own <link name="world"/>) publishes a TF
    # tree with no connection to the arm's — move_group's
    # planning_scene_monitor then can't resolve any panel/* frame
    # against the "world" planning frame ("Tf has two or more
    # unconnected trees" warning spam). Same PANEL_X/Y/Z/YAW as the
    # actual spawn pose above, so world -> panel/world matches where the
    # model really is.
    panel_world_connector = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", PANEL_X, "--y", PANEL_Y, "--z", PANEL_Z,
            "--qz", PANEL_QZ, "--qw", PANEL_QW,
            "--frame-id", "world", "--child-frame-id", "panel/world",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("spawn_panel")),
    )

    panel_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "indomitus_panel", "-topic", "panel/robot_description",
            "-x", PANEL_X, "-y", PANEL_Y, "-z", PANEL_Z, "-Y", PANEL_YAW,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_panel")),
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    ros_gz_bridge_no_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config_no_camera, "use_sim_time": True}],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("camera")),
    )

    # Sim is single-host, but panel_align_node/keyboard_servo_node still
    # run as separate processes here too — bring up the same lock server
    # the real GS/Jetson split needs (arm_bringup/arm.launch.py), so sim
    # actually exercises the real locking path instead of silently having
    # none.
    arm_motion_lock_server = Node(
        package="arm_teleop",
        executable="arm_motion_lock_server",
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "indomitus_arm_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    # Streaming teleop controller, spawned inactive — JTC owns the joints
    # until arm_teleop switches controllers for Servo. Mirrors arm_bringup/arm.launch.py;
    # a separate spawner call because --inactive applies to the whole call.
    forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["indomitus_arm_forward_position_controller", "--inactive"],
        output="screen",
    )

    # Each startup stage runs only if the previous one exited with code 0;
    # otherwise the whole launch shuts down instead of starting nodes
    # against a broken stack.
    def _after_spawn(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Entity spawn failed (exit code {event.returncode})."),
                Shutdown(reason="entity spawn failed"),
            ]
        return [controller_spawner, forward_spawner]

    delayed_controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=_after_spawn,
        )
    )

    # Spawned only after indomitus_arm_controller is confirmed loaded and
    # active (chained off controller_spawner's own exit, not run alongside
    # it). This ordering was originally added to test a startup-race
    # hypothesis for the gripper reliability bug; that hypothesis was
    # disproven (the real cause was gz_ros2_control 0.7.20 dropping writes
    # at a joint's own command_interface bound, fixed via
    # finger_limit_margin in arm_macro.xacro). Kept anyway since it's a
    # harmless, slightly cleaner startup order.
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_right_controller",
            "gripper_left_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    def _after_arm_controller(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Controller activation failed (exit code {event.returncode})."),
                Shutdown(reason="controller activation failed"),
            ]
        # Jaw finger joints only exist when end_effector:=jaw.
        if LaunchConfiguration("end_effector").perform(context) != "jaw":
            return [LogInfo(
                msg="end_effector != 'jaw' — skipping gripper_right/left_controller spawn."
            )]
        return [gripper_spawner]

    delayed_gripper_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=_after_arm_controller,
        )
    )

    # planning_pipelines restricted to ompl on purpose: with none of the
    # config yaml files it discovers for chomp/pilz_industrial_motion_planner
    # actually present in this package (only ompl_planning.yaml exists),
    # letting MoveItConfigsBuilder auto-load its bundled default configs
    # for all three still left move_group picking an ambiguous
    # "planning_plugin" between them ("Multiple planning plugins
    # available... Using 'chomp_interface/CHOMPPlanner' for now" — even
    # when the request explicitly asked for the 'ompl' pipeline_id).
    # panel_align_node's Cartesian pose-constraint goals need real OMPL
    # (CHOMP only accepts joint-space goals and rejects the rest outright
    # with MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS); nothing in this repo
    # uses chomp or pilz, so there's no reason to load them at all.
    # Without this mapping move_group/servo always assume 'jaw', so with a
    # drill they wait forever for finger joints that never publish.
    moveit_config = MoveItConfigsBuilder(
        "indomitus_arm", package_name="arm_moveit_config"
    ).robot_description(mappings={
        "end_effector": _arg_from_argv("end_effector", "jaw"),
    }).planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)

    moveit_config_dir = get_package_share_directory("arm_moveit_config")
    with open(os.path.join(moveit_config_dir, "config", "servo.yaml")) as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    # Inverse Jacobian only — see arm_bringup/arm.launch.py (KDL searchPositionIK
    # from home makes +X teleop freeze while -X still works).
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            servo_params,
        ],
    )

    def _after_controllers(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Controller activation failed (exit code {event.returncode})."),
                Shutdown(reason="controller activation failed"),
            ]
        return list(move_group_launch.entities) + [servo_node]

    delayed_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=_after_controllers,
        )
    )

    ld = LaunchDescription(
        [
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument(
                "end_effector", default_value="jaw",
                description="'jaw', 'other_tool', or 'drill_sampling' — see arm_macro.xacro"),
            DeclareLaunchArgument(
                "spawn_panel",
                # Defaults off for the drill build; still overridable.
                default_value=PythonExpression([
                    "'false' if '", LaunchConfiguration("end_effector"), "' == 'drill_sampling' else 'true'"
                ]),
                description="Also spawn the switch panel task board, for panel_align_node/CV testing "
                            "(defaults to false when end_effector:=drill_sampling)"),
            SetParameter(name="use_sim_time", value=True),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            delete_marker_layout_file,
            arm_motion_lock_server,
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            panel_state_publisher,
            panel_world_connector,
            panel_spawn,
            ros_gz_bridge,
            ros_gz_bridge_no_camera,
            delayed_controller_spawners,
            delayed_move_group,
            delayed_gripper_spawner,
        ]
    )

    return ld