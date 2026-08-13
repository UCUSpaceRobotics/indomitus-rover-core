"""
Launch file for MoveIt demo + moveit_servo.

IMPORTANT (Servo readiness):
moveit_servo (servo_node) subscribes to its command sources and expects
up-to-date /joint_states plus an active trajectory controller
(ARM_CONTROLLER_NAME) as soon as it starts. If servo_node is launched
before `controller_manager` has actually activated that controller,
the first Servo commands can be dropped or fail with
"controller not active" errors.

For this reason, servo_node is NOT added to the LaunchDescription
directly. Instead, we wait until the spawner for the target controller
exits with return code 0 (i.e. the controller was successfully loaded
and activated), and only then dynamically add servo_node to the launch
tree via OnProcessExit.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

ARM_CONTROLLER_NAME = "indomitus_arm_controller"


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description() -> LaunchDescription:
    moveit_config = MoveItConfigsBuilder(
        "indomitus_arm", package_name="arm_moveit_config"
    ).to_moveit_configs()

    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
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
            f"starting servo_node."
        )
        return [servo_node]

    demo_launch = generate_demo_launch(moveit_config)

    ld = LaunchDescription()
    for action in demo_launch.entities:
        ld.add_action(action)

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
    # gripper's own position controller isn't part of that and needs its
    # own spawner.
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller",
                "--controller-manager-timeout", "60",
                "--switch-timeout", "60",
                "--service-call-timeout", "70",
            ],
            output="screen",
        )
    )

    return ld