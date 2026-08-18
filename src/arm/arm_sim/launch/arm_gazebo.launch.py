import os

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

    xacro_file = os.path.join(arm_description_dir, "urdf", "arm_standalone.urdf.xacro")
    world_file = os.path.join(arm_sim_dir, "worlds", "empty.sdf")
    bridge_config = os.path.join(arm_sim_dir, "config", "gz_bridge.yaml")
    bridge_config_no_camera = os.path.join(arm_sim_dir, "config", "gz_bridge_no_camera.yaml")

    resource_path_root = os.path.dirname(arm_description_dir)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_gz_path]))
    ign_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_ign_path]))

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
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    # Streaming teleop controller, spawned inactive — JTC owns the joints
    # until arm_tasks switches controllers for Servo. Mirrors demo.launch.py;
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
    # it) — gz_ros2_control was found to sometimes silently drop a gripper
    # joint's very first write when its controller activated in the same
    # burst as the arm's; giving it a separate, later spawn call avoids
    # that startup window entirely instead of guessing at a fixed delay.
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
        return [gripper_spawner]

    delayed_gripper_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=_after_arm_controller,
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

    # Inverse Jacobian only — see demo.launch.py (KDL searchPositionIK
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
            SetParameter(name="use_sim_time", value=True),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            ros_gz_bridge_no_camera,
            delayed_controller_spawners,
            delayed_move_group,
            delayed_gripper_spawner,
        ]
    )

    return ld