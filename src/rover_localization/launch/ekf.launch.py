import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_localization_share = get_package_share_directory('rover_localization')
    ekf_config_yaml = os.path.join(rover_localization_share, 'config', 'ekf_filter.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
    ])
