"""
Arm-only Gazebo sim: world, bridges, the arm_standalone model and the
optional switch panel. The arm's ROS stack (robot_state_publisher,
controllers, move_group, servo) is arm_bringup/stack.launch.py under /arm,
the same file the real arm runs.
"""

import contextlib
import os
import random

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arm_description_dir = get_package_share_directory("arm_description")
    arm_sim_dir = get_package_share_directory("arm_sim")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    panel_description_dir = get_package_share_directory("panel_description")

    xacro_file = os.path.join(arm_description_dir, "urdf", "arm_standalone.urdf.xacro")
    panel_xacro_file = os.path.join(panel_description_dir, "urdf", "panel_standalone.urdf.xacro")
    world_file = os.path.join(arm_sim_dir, "worlds", "empty.sdf")
    bridge_config = os.path.join(arm_sim_dir, "config", "gz_bridge.yaml")

    resource_path_root = os.path.dirname(arm_description_dir)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(filter(
        None, [resource_path_root, os.path.dirname(panel_description_dir), existing_gz_path]))
    ign_resource_path = os.pathsep.join(filter(
        None, [resource_path_root, os.path.dirname(panel_description_dir), existing_ign_path]))

    # Spawned from a string, not from a robot_state_publisher topic: the
    # only arm robot_state_publisher is /arm's, in arm_bringup/stack.launch.py.
    spawn_description_content = Command([
        "xacro ", xacro_file,
        " sim:=true",
        " camera:=", LaunchConfiguration("camera"),
        " end_effector:=", LaunchConfiguration("end_effector"),
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", spawn_description_content, "-name", "indomitus_arm", "-z", "0.3"],
        output="screen",
    )

    # The same arm stack as on the real arm, under /arm. Own group so the
    # namespace and its launch_arguments don't leak into the rest of this file.
    arm_stack = GroupAction([
        PushRosNamespace("arm"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("arm_bringup"),
                             "launch", "stack.launch.py")
            ),
            launch_arguments={
                "description_file": xacro_file,
                "use_sim": "true",
                "camera": LaunchConfiguration("camera"),
                "end_effector": LaunchConfiguration("end_effector"),
            }.items(),
        ),
    ])

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

    clock_bridge = Node(
        package="ros_gz_bridge",
        name="clock_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        name="arm_camera_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    def _after_spawn(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Entity spawn failed (exit code {event.returncode})."),
                Shutdown(reason="entity spawn failed"),
            ]
        return None

    spawn_failure_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=_after_spawn,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument(
            "end_effector", default_value="jaw",
            description="Tool mounted on the arm - see arm_macro.xacro"),
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
        gz_sim,
        clock_bridge,
        camera_bridge,
        arm_stack,
        spawn_entity,
        spawn_failure_handler,
        panel_state_publisher,
        panel_world_connector,
        panel_spawn,
    ])
