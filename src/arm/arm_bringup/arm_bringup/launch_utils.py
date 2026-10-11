"""Launch helpers for the arm: include_launch and the MoveIt config + Servo
node used by arm_bringup/stack.launch.py (real arm and both sims).

The rover's Jetson runs Humble, dev/sim runs Jazzy; MoveIt differs in the
OMPL pipeline format and in moveit_servo (executable name, kinematics).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Condition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

IS_HUMBLE = os.environ.get('ROS_DISTRO') == 'humble'

END_EFFECTORS = ['jaw', 'other_tool', 'drill_sampling', 'astrobio']  # see arm_macro.xacro

ARM_CONTROLLER = 'indomitus_arm_controller'
FORWARD_CONTROLLER = 'indomitus_arm_forward_position_controller'


def include_launch(
    package: str, launch_file: str, launch_arguments: dict | None = None,
    condition: Condition | None = None,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def _load_config_yaml(name: str) -> dict:
    path = os.path.join(get_package_share_directory('arm_moveit_config'), 'config', name)
    with open(path) as f:
        return yaml.safe_load(f)


def build_moveit_config(description_file: str, mappings: dict):
    """Render the arm description once and load the MoveIt config for it.

    mappings must be plain strings (performed launch configurations): xacro
    runs right here, not at launch time.
    """
    # Only OMPL: with chomp/pilz also loaded, move_group picks CHOMP even for
    # pipeline_id='ompl' requests, and CHOMP rejects panel_align_node's
    # Cartesian pose goals.
    moveit_config = (
        MoveItConfigsBuilder('indomitus_arm', package_name='arm_moveit_config')
        .robot_description(file_path=description_file, mappings=mappings)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    if IS_HUMBLE:
        # ompl_planning.yaml is in Jazzy format, see ompl_planning_humble.yaml.
        ompl = moveit_config.planning_pipelines['ompl']
        for key in ('planning_plugins', 'request_adapters', 'response_adapters'):
            ompl.pop(key, None)
        ompl.update(_load_config_yaml('ompl_planning_humble.yaml'))
    return moveit_config


def servo_node(moveit_config, extra_parameters=()) -> Node:
    """moveit_servo for the arm, with the distro-specific bits."""
    servo_params = _load_config_yaml('servo.yaml')['moveit_servo']['ros__parameters']
    if IS_HUMBLE:
        # servo.yaml streams JointTrajectory into the JTC, which only works well
        # with Jazzy's rolling command window; Humble keeps the forward
        # position controller (keyboard_servo_node switches to it there).
        servo_params.update({
            'command_out_type': 'std_msgs/Float64MultiArray',
            'command_out_topic': f'{FORWARD_CONTROLLER}/commands',
            'publish_joint_velocities': False,
        })
    # Jazzy Servo rejects every command without an IK solver. Humble Servo
    # must not get one: it then uses KDL IK, which fails for +X from home
    # (one-sided teleop); without it, it uses the symmetric J^#.
    kinematics = [] if IS_HUMBLE else [moveit_config.robot_description_kinematics]
    return Node(
        package='moveit_servo',
        executable='servo_node_main' if IS_HUMBLE else 'servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            *kinematics,
            # Without joint_limits.yaml Servo only sees the URDF <limit> tags.
            moveit_config.joint_limits,
            {'moveit_servo': servo_params},
            *extra_parameters,
        ],
    )
