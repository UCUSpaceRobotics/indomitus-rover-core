"""
Launch file for MoveIt demo + moveit_servo.

Servo streams joint *positions* (Float64MultiArray) to
indomitus_arm_forward_position_controller. That controller is spawned
inactive; arm_tasks switches JTC <-> forward around home / start_servo.

servo_node still waits until indomitus_arm_controller (JTC) has been
spawned successfully so /joint_states and the hardware stack are up.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

ARM_CONTROLLER_NAME = "indomitus_arm_controller"
FORWARD_CONTROLLER_NAME = "indomitus_arm_forward_position_controller"


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description() -> LaunchDescription:
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Whether to use fake hardware or real CAN bus",
    )

    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings={"use_fake_hardware": use_fake_hardware})
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    # Light smoothing only: heavy coeffs lag the command and, with high MIT
    # kp, the arm overshoots then waits — stop-go buzz at the Servo rate.
    smoothing_params = {"butterworth_filter_coeff": 2.0}

    # Do NOT pass robot_description_kinematics into servo_node.
    # Humble Servo prefers the KDL plugin (searchPositionIK, 5 ms timeout)
    # over the inverse Jacobian. From home, +X (W) often fails that IK so
    # cartesianServoCalcs() returns false and the last joint command is
    # republished (frozen commands, status still 0). -X (S) succeeds, so
    # teleop looks one-sided. Without a solver, Servo uses J^# which is
    # symmetric in ±X. move_group still gets KDL for Plan&Execute.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # Without this, joint_limits.yaml (per-joint max_velocity/
            # max_acceleration/position bounds) never reaches Servo's robot
            # model — it falls back to the bare URDF <limit> tags only, so
            # the wrist_1_wrist_2 singularity fence and the tuned speed
            # ceilings were silently not enforced during teleop.
            moveit_config.joint_limits,
            servo_params,
            smoothing_params,
        ],
    )

    def _on_process_exit(event: ProcessExited, context: LaunchContext):
        cmd_parts = [str(part) for part in (event.cmd or [])]

        is_spawner = any(part.endswith("spawner") for part in cmd_parts)
        has_controller_arg = ARM_CONTROLLER_NAME in cmd_parts

        if not (is_spawner and has_controller_arg):
            return None

        if event.returncode != 0:
            print(
                f"[servo_launch] Spawner for '{ARM_CONTROLLER_NAME}' "
                f"exited with code {event.returncode} — servo_node will NOT be started."
            )
            return None

        print(
            f"[servo_launch] Controller '{ARM_CONTROLLER_NAME}' is active — "
            f"starting servo_node (streams to '{FORWARD_CONTROLLER_NAME}'; "
            f"activate that controller before teleop via gamepad A / start_servo)."
        )
        return [servo_node]

    # Load the streaming teleop controller inactive — JTC owns the joints until
    # arm_tasks switches controllers for Servo.
    forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[FORWARD_CONTROLLER_NAME, "--inactive"],
        output="screen",
    )

    demo_launch = generate_demo_launch(moveit_config)

    ld = LaunchDescription()
    ld.add_action(declare_use_fake_hardware_cmd)
    for action in demo_launch.entities:
        ld.add_action(action)
    ld.add_action(forward_spawner)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=None,
                on_exit=_on_process_exit,
            )
        )
    )

    # generate_demo_launch() only spawns ARM_CONTROLLER_NAME (the one
    # trajectory-execution controller MoveIt itself knows about) — the
    # gripper's own position controller(s) aren't part of that and need
    # their own spawner. Which controllers exist depends on the same
    # use_fake_hardware split arm_macro.xacro makes for the gripper
    # <ros2_control> block: fake hardware (or sim) exposes both fingers,
    # but real hardware only stubs the right one (JawGripperStub — the real
    # arm has no gripper motor yet, see arm_macro.xacro) with no left-finger
    # interface at all, so spawning gripper_left_controller there would
    # just fail waiting for an interface that will never exist.
    ld.add_action(
        Node(
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
            condition=IfCondition(use_fake_hardware),
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_right_controller",
                "--controller-manager-timeout", "60",
                "--switch-timeout", "60",
                "--service-call-timeout", "70",
            ],
            output="screen",
            condition=UnlessCondition(use_fake_hardware),
        )
    )

    return ld
