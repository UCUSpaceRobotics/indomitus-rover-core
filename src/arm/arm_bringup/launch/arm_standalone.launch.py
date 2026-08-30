import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _arg_from_argv(name: str, default: str) -> str:
    """Plain-str mappings force MoveItConfigsBuilder's single-eval xacro
    path — LaunchConfiguration ones desync across sub-launches. Same
    helper/reasoning as demo.launch.py's own.
    """
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def generate_launch_description():
    arm_description_dir = get_package_share_directory('arm_description')
    arm_moveit_config_dir = get_package_share_directory('arm_moveit_config')
    # FindPackageShare is lazy (unlike get_package_share_directory) — won't
    # crash this whole file if arm_viz is absent (e.g. Jetson prod) and
    # use_rviz is false.
    arm_viz_rviz_config = PathJoinSubstitution(
        [FindPackageShare('arm_viz'), 'rviz', 'arm.rviz']
    )
    xacro_file = os.path.join(arm_description_dir, 'urdf', 'arm_standalone.urdf.xacro')
    ros2_controllers_yaml = os.path.join(arm_moveit_config_dir, 'config', 'ros2_controllers.yaml')

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use mock_components/GenericSystem instead of the real CAN hardware interface'
    )

    end_effector_arg = DeclareLaunchArgument(
        'end_effector',
        default_value='jaw',
        description="'jaw', 'other_tool', or 'drill_sampling' — see arm_macro.xacro"
    )

    # Pure kinematic preview: sliders drive /joint_states directly, no ros2_control,
    # no CAN, no real/mock hardware loop at all. Mutually exclusive with actually
    # controlling anything — never run this together with the real arm.
    gui_only_arg = DeclareLaunchArgument(
        'gui_only',
        default_value='false',
        description='Pure URDF/TF preview with manual sliders — no ros2_control, no hardware'
    )

    # Default true (laptop viz check); set false for headless (e.g. Jetson).
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch rviz2 (needs arm_viz\'s rviz2 dependency present)'
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
            ' end_effector:=', LaunchConfiguration('end_effector')
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['indomitus_arm_controller'],
        namespace='arm',
        condition=UnlessCondition(LaunchConfiguration('gui_only'))
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_right_controller', 'gripper_left_controller'],
        namespace='arm',
        output='screen',
    )

    # planning_pipelines restricted to ompl on purpose — see the matching
    # comment in arm_sim/launch/arm_gazebo.launch.py: without this,
    # move_group ambiguously picks CHOMP, which rejects panel_align_node's
    # Cartesian pose-constraint goals outright (INVALID_GOAL_CONSTRAINTS).
    moveit_config = (
        MoveItConfigsBuilder('indomitus_arm', package_name='arm_moveit_config')
        .robot_description(mappings={
            'use_fake_hardware': _arg_from_argv('use_fake_hardware', 'true'),
            'end_effector': _arg_from_argv('end_effector', 'jaw'),
        })
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    move_group_launch = generate_move_group_launch(moveit_config)

    with open(os.path.join(arm_moveit_config_dir, 'config', 'servo.yaml')) as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = {'moveit_servo': servo_yaml['moveit_servo']['ros__parameters']}

    # Inverse Jacobian only — see demo.launch.py (KDL searchPositionIK
    # from home makes +X teleop freeze while -X still works).
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        namespace='arm',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            servo_params,
        ],
    )

    def _after_arm_controller(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f'Controller activation failed (exit code {event.returncode}).'),
                Shutdown(reason='controller activation failed'),
            ]
        # move_group_launch comes from a MoveIt helper with no namespace hook
        # of its own — wrap it fresh here rather than relying on an outer
        # PushRosNamespace, since this whole list is only added to the tree
        # once this event fires (well after any earlier group would have
        # closed its scope).
        return [
            GroupAction([PushRosNamespace('arm'), *move_group_launch.entities]),
            servo_node,
            gripper_spawner,
        ]

    return LaunchDescription([
        use_fake_hardware_arg,
        end_effector_arg,
        gui_only_arg,
        use_rviz_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='arm',
            parameters=[robot_description]
        ),

        # --- gui_only:=true path — visualization sliders, nothing else ---
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace='arm',
            condition=IfCondition(LaunchConfiguration('gui_only'))
        ),

        # --- gui_only:=false path (default) — real ros2_control loop ---
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='arm',
            parameters=[robot_description, ros2_controllers_yaml],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gui_only'))
        ),
        TimerAction(
            period=2.0,  # give ros2_control_node time to come up before spawning
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    namespace='arm',
                    condition=UnlessCondition(LaunchConfiguration('gui_only'))
                ),
                arm_controller_spawner,
                # Streaming teleop controller, spawned inactive — JTC owns
                # the joints until arm_tasks switches controllers for
                # Servo. Mirrors arm_gazebo.launch.py/demo.launch.py.
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['indomitus_arm_forward_position_controller', '--inactive'],
                    namespace='arm',
                    condition=UnlessCondition(LaunchConfiguration('gui_only'))
                ),
            ]
        ),

        # move_group + gripper controllers, chained off the arm
        # controller's own spawn confirming success — mirrors
        # arm_gazebo.launch.py's delayed_move_group/delayed_gripper_spawner.
        RegisterEventHandler(
            OnProcessExit(target_action=arm_controller_spawner, on_exit=_after_arm_controller)
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='arm',
            arguments=['-d', arm_viz_rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])