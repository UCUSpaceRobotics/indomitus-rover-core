import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_localization_share = get_package_share_directory('rover_localization')

    ekf_real_config = os.path.join(rover_localization_share, 'config', 'ekf.yaml')
    ekf_sim_config = os.path.join(rover_localization_share, 'config', 'ekf_sim.yaml')

    use_sim = LaunchConfiguration('use_sim')
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')

    return LaunchDescription([
        use_sim_arg,

        Node(
            condition=IfCondition(use_sim),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_sim_config,
                {'use_sim_time': use_sim},
            ],
        ),

        Node(
            condition=UnlessCondition(use_sim),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_real_config,
                {'use_sim_time': use_sim},
            ],
        ),
    ])
