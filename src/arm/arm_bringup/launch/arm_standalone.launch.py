import os
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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    arm_description_dir = get_package_share_directory('arm_description')
    arm_moveit_config_dir = get_package_share_directory('arm_moveit_config')
    arm_viz_dir = get_package_share_directory('arm_viz')
    xacro_file = os.path.join(arm_description_dir, 'urdf', 'arm_standalone.urdf.xacro')
    ros2_controllers_yaml = os.path.join(arm_moveit_config_dir, 'config', 'ros2_controllers.yaml')

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use mock_components/GenericSystem instead of the real CAN hardware interface'
    )

    # Pure kinematic preview: sliders drive /joint_states directly, no ros2_control,
    # no CAN, no real/mock hardware loop at all. Mutually exclusive with actually
    # controlling anything — never run this together with the real arm.
    gui_only_arg = DeclareLaunchArgument(
        'gui_only',
        default_value='false',
        description='Pure URDF/TF preview with manual sliders — no ros2_control, no hardware'
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware')
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['indomitus_arm_controller'],
        condition=UnlessCondition(LaunchConfiguration('gui_only'))
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_right_controller', 'gripper_left_controller'],
        output='screen',
    )

    # planning_pipelines restricted to ompl on purpose — see the matching
    # comment in arm_sim/launch/arm_gazebo.launch.py: without this,
    # move_group ambiguously picks CHOMP, which rejects panel_align_node's
    # Cartesian pose-constraint goals outright (INVALID_GOAL_CONSTRAINTS).
    moveit_config = MoveItConfigsBuilder(
        'indomitus_arm', package_name='arm_moveit_config'
    ).planning_pipelines(pipelines=['ompl']).to_moveit_configs()
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
        return list(move_group_launch.entities) + [servo_node, gripper_spawner]

    return LaunchDescription([
        use_fake_hardware_arg,
        gui_only_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),

        # --- gui_only:=true path — visualization sliders, nothing else ---
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui_only'))
        ),

        # --- gui_only:=false path (default) — real ros2_control loop ---
        Node(
            package='controller_manager',
            executable='ros2_control_node',
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
            arguments=['-d', os.path.join(arm_viz_dir, 'rviz', 'arm.rviz')]
        )
    ])