import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution


def generate_launch_description():
    rover_localization_share = get_package_share_directory('rover_localization')

    ekf_real_config = os.path.join(rover_localization_share, 'config', 'ekf.yaml')
    ekf_sim_config = os.path.join(rover_localization_share, 'config', 'ekf_sim.yaml')
    ekf_tilt_config = os.path.join(rover_localization_share, 'config', 'ekf_tilt.yaml')

    use_sim_time_val = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Enable simulation mode (uses sim clock and sim EKF config).',
    )

    output_odom_topic_val = LaunchConfiguration('output_odom_topic')
    output_odom_topic_arg = DeclareLaunchArgument(
        'output_odom_topic',
        default_value='/odom',
        description='Topic to which EKF node will publish filtered odometry'
    )

    config_file = IfElseSubstitution(
        use_sim_time_val,
        if_value=ekf_sim_config,
        else_value=ekf_real_config,
    )

    return LaunchDescription([
        use_sim_time_arg,
        output_odom_topic_arg,

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time_val},
            ],
            remappings=[('odometry/filtered', output_odom_topic_val)],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_tilt_node',
            output='screen',
            parameters=[
                ekf_tilt_config,
                {'use_sim_time': use_sim_time_val},
            ],
            remappings=[('odometry/filtered', '/tilt_ekf/odom')],
        ),
    ])
