from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .to_moveit_configs()
    )

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

    ARM_CONTROLLER_NAME = "indomitus_arm_controller"

    def _on_process_exit(event: ProcessExited, context: LaunchContext):
        cmd_parts = [str(part) for part in (event.cmd or [])]
        is_spawner = any(part.endswith("spawner") for part in cmd_parts)
        has_controller_arg = ARM_CONTROLLER_NAME in cmd_parts
        if is_spawner and has_controller_arg:
            return [servo_node]
        return None

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

    return ld