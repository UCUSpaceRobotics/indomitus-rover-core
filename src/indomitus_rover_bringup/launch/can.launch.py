#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN network interface name (e.g. can0, can1)'
    )

    # ── Kinematics node ───────────────────────────────────────────────────────

    kinematics_node = Node(
        package='indomitus_rover_control',
        executable='rover_kinematics_node',
        output='screen',
        parameters=[
            os.path.join(FindPackageShare('indomitus_rover_control'), 'config', 'rover_geometry.yaml')
        ],
    )

    # ── Damiao driver ─────────────────────────────────────────────────────────

    # damiao_driver_node = Node(
    #     package='damiao_driver',
    #     executable='damiao_driver_node',
    #     name='damiao_driver',
    #     output='screen',
    #     remappings=[
    #         ('wheel_targets', '/wheel_targets'),
    #         ('from_can_bus',  '/from_can_bus'),
    #         ('to_can_bus',    '/to_can_bus'),
    #     ],
    # )

    # ── ros2_socketcan bridge (sender + receiver в одному launch) ─────────────

    # socketcan_bridge = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ros2_socketcan'),
    #             'launch',
    #             'socket_can_bridge.launch.xml',
    #         ])
    #     ]),
    #     launch_arguments={
    #         'interface': LaunchConfiguration('can_interface'),
    #     }.items(),
    # )

    return LaunchDescription([
        can_interface_arg,
        kinematics_node,
        # damiao_driver_node,
        # socketcan_bridge,
    ])