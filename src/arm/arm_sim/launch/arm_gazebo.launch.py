import os

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
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


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

    panel_description_content = ParameterValue(
        Command([
            "xacro ", panel_xacro_file,
            " sim:=true",
            " panel_x:=", PANEL_X, " panel_y:=", PANEL_Y,
            " panel_z:=", PANEL_Z, " panel_yaw:=", PANEL_YAW,
        ]),
        value_type=str,
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

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "indomitus_arm_controller",
            "gripper_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
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
        return [controller_spawner]

    delayed_controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=_after_spawn,
        )
    )

    moveit_config = MoveItConfigsBuilder(
        "indomitus_arm", package_name="arm_moveit_config"
    ).to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)

    moveit_config_dir = get_package_share_directory("arm_moveit_config")
    with open(os.path.join(moveit_config_dir, "config", "servo.yaml")) as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
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
                "spawn_panel", default_value="true",
                description="Also spawn the switch panel task board, for panel_align_node/CV testing"),
            SetParameter(name="use_sim_time", value=True),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
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
        ]
    )

    return ld