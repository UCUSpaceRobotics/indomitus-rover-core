import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['indomitus_arm_controller'],
                    condition=UnlessCondition(LaunchConfiguration('gui_only'))
                ),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(arm_viz_dir, 'rviz', 'arm.rviz')]
        )
    ])