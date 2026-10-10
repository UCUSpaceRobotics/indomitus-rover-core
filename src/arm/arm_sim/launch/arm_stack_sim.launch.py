"""
The simulated arm's ROS stack, all under /arm: robot_state_publisher,
controller spawners, move_group, servo and the motion lock server.

Starts no Gazebo and spawns nothing — the entry point does that:
  - arm_gz.launch.py (arm-only): description_file:=arm_standalone.urdf.xacro
  - the combined sim, arm mounted on the rover: description_file:= the
    whole-robot package's arm_on_rover.urdf.xacro (root main_body_link)

The gz_ros2_control plugin in the spawned model runs /arm/controller_manager
and reads its robot_description from /arm/robot_state_publisher, which this
file starts from description_file. MoveIt uses that same rendered URDF, so
its root link (world arm-only, main_body_link mounted) exists in TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_move_group_launch

from arm_bringup.launch_utils import END_EFFECTORS, build_moveit_config, servo_node

ARM_NAMESPACE = "arm"


def _in_arm_namespace(*actions):
    # Actions started later from an event handler run after this file's
    # namespace push has been popped, so they need their own.
    return GroupAction([
        PushRosNamespace(ARM_NAMESPACE),
        SetParameter(name="use_sim_time", value=True),
        *actions,
    ])


def _launch_setup(context):
    description_file = LaunchConfiguration("description_file").perform(context)
    end_effector = LaunchConfiguration("end_effector").perform(context)
    camera = LaunchConfiguration("camera").perform(context)

    # Without the end_effector mapping move_group/servo always assume 'jaw',
    # so with a drill they wait forever for finger joints that never publish.
    moveit_config = build_moveit_config(description_file, {
        "sim": "true",
        "camera": camera,
        "end_effector": end_effector,
    })

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        # tf/tf_static stay global — see docs/software/tf_ownership.md.
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
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

    # No --controller-manager-timeout: /arm/controller_manager only appears
    # once the entry point has spawned the model, which may take a while.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "indomitus_arm_controller",
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
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    move_group_launch = generate_move_group_launch(moveit_config)
    servo = servo_node(moveit_config)

    # Each startup stage runs only if the previous one exited with code 0;
    # otherwise the whole launch shuts down instead of starting nodes
    # against a broken stack.
    def _after_arm_controller(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Controller activation failed (exit code {event.returncode})."),
                Shutdown(reason="controller activation failed"),
            ]
        actions = [*move_group_launch.entities, servo]
        # Jaw finger joints only exist when end_effector:=jaw.
        if end_effector == "jaw":
            actions.append(gripper_spawner)
        else:
            actions.append(LogInfo(
                msg="end_effector != 'jaw' — skipping gripper_right/left_controller spawn."
            ))
        return [_in_arm_namespace(*actions)]

    return [
        robot_state_publisher,
        arm_motion_lock_server,
        controller_spawner,
        forward_spawner,
        RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner,
            on_exit=_after_arm_controller,
        )),
    ]


def generate_launch_description() -> LaunchDescription:
    arm_description_dir = get_package_share_directory("arm_description")

    return LaunchDescription([
        DeclareLaunchArgument(
            "description_file",
            default_value=os.path.join(arm_description_dir, "urdf", "arm_standalone.urdf.xacro"),
            description="Arm xacro for /arm robot_state_publisher and MoveIt "
                        "(arm_standalone.urdf.xacro, or arm_on_rover.urdf.xacro when mounted)"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument(
            "end_effector", default_value="jaw", choices=END_EFFECTORS,
            description="Tool mounted on the arm - see arm_macro.xacro"),
        # The only namespace push for the arm stack. Nodes below use relative
        # names; gamepad_servo.launch.py hardcodes namespace='arm' itself, so
        # it must never be included inside this group (/arm/arm/...).
        GroupAction([
            PushRosNamespace(ARM_NAMESPACE),
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=_launch_setup),
        ]),
    ])
