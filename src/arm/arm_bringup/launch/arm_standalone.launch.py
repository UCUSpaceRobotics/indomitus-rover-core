import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription([
        use_fake_hardware_arg,
        end_effector_arg,
        gui_only_arg,
        use_rviz_arg,

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
            arguments=['-d', arm_viz_rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])