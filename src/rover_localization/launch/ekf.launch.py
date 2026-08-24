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
            package='rover_localization',
            executable='tilt_broadcaster_node',
            name='tilt_broadcaster_node',
            output='screen',
            parameters=[
                {'imu_topic': '/zed2i/imu/data'},
                {'parent_frame': 'base_footprint'},
                {'child_frame': 'base_link'},

                # Height of base_link above base_footprint/ground -- must match
                # base_link_ground_height in rover_description/urdf/properties.xacro.
                {'ground_height': 0.4597},

                # Age at which the IMU stops being trusted and the tilt
                # transform is no longer sent.
                {'imu_timeout': 0.5},

                {'use_sim_time': use_sim_time_val},
            ],
        ),
    ])
