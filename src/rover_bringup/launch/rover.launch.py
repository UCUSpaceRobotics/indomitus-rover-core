# rover_bringup/launch/rover.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(
    package: str,
    launch_file: str,
    launch_arguments: dict | None = None,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    rover_bringup_share = get_package_share_directory('rover_bringup')

    interface_arg = DeclareLaunchArgument(
        'interface', default_value='can0',
        description='SocketCAN network interface name',
    )


    robot_description = Command([
        'xacro ',
        os.path.join(rover_bringup_share, 'urdf', 'rover_real.urdf.xacro'),
        ' can_interface:=', LaunchConfiguration('interface'),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(rover_bringup_share, 'config', 'controllers.yaml'),
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_controller', '--inactive'],
        output='screen',
    )

    odometry_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['odometry_controller'],
        output='screen',
    )

    return LaunchDescription([
        interface_arg,
        include_launch('rover_bringup', 'can.launch.py', {
            'interface': LaunchConfiguration('interface'),
        }),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        swerve_controller_spawner,
        odometry_controller_spawner,
        include_launch('rover_bringup', 'twist_mux.launch.py'),
        include_launch('rover_peripherals', 'lighting.launch.py'),
        include_launch('rover_localization', 'ekf.launch.py')
    ])
