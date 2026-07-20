from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from rover_bringup.launch_utils import include_launch


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='nav2_test_world'),
        DeclareLaunchArgument('model_name', default_value='indomitus_rover'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='1.0'),

        include_launch('rover_sim', 'sim_gz.launch.py', {
            'world_name': LaunchConfiguration('world_name'),
            'model_name': LaunchConfiguration('model_name'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'extra_xacro_args': 'use_nav:=true lidar_simulate_scan:=true stereo_camera_simulate_depth:=true',
        }),

        TimerAction(
            period=10.0,
            actions=[
                include_launch('rover_localization', 'slam.launch.py', {
                    'use_sim_time': 'true',
                }),
                include_launch('rover_navigation', 'nav2.launch.py', {
                    'use_sim': 'true',
                    'cmd_vel_topic': 'cmd_vel_nav',
                }),
            ],
        ),
    ])