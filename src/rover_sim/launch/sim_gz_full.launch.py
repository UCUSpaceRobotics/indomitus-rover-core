import os
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rover_bringup.launch_utils import include_launch


def generate_rover_bridge_config(context, rover_sim_share: str) -> list[Node]:
    world = LaunchConfiguration('world_name').perform(context)
    model = LaunchConfiguration('model_name').perform(context)

    template_path = os.path.join(rover_sim_share, 'config', 'bridge_gz_with_arm_camera.yaml')
    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world, model=model)

    path_bridge_config = f"/tmp/bridge_{world}_{model}_urdf.yaml"
    with open(path_bridge_config, "w") as f:
        f.write(rendered)

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={path_bridge_config}'],
        output='screen',
    )]


def make_gazebo_launch(rover_sim_share: str) -> IncludeLaunchDescription:
    world_file = PathJoinSubstitution([
        rover_sim_share, 'worlds', LaunchConfiguration('world_name')
    ])
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    return IncludeLaunchDescription(source, launch_arguments={
        'gz_args': ['-r ', world_file, '.sdf'],
        'on_exit_shutdown': 'True',
    }.items())


def make_rover_spawn_node() -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('model_name'),
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '3.5',
        ],
        output='screen',
    )


def generate_panel_bridge_config(context, panel_sim_share: str) -> list[Node]:
    world = LaunchConfiguration('world_name').perform(context)

    template_path = os.path.join(panel_sim_share, 'config', 'gz_bridge.yaml')
    with open(template_path) as f:
        rendered = Template(f.read()).substitute(world=world)

    path_bridge_config = f"/tmp/panel_bridge_{world}.yaml"
    with open(path_bridge_config, "w") as f:
        f.write(rendered)

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='panel',
        arguments=['--ros-args', '-p', f'config_file:={path_bridge_config}'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )]


def make_panel_nodes(panel_description_share: str) -> list:
    xacro_file = os.path.join(panel_description_share, 'urdf', 'panel_standalone.urdf.xacro')

    # panel_x/y/z/yaw are the same launch args passed to panel_spawn below --
    # the published world -> panel_base_link TF must match where the model
    # is actually spawned in Gazebo, or perception/planning against the
    # panel will be working off a stale pose.
    panel_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file, ' sim:=true',
            ' panel_x:=', LaunchConfiguration('panel_x'),
            ' panel_y:=', LaunchConfiguration('panel_y'),
            ' panel_z:=', LaunchConfiguration('panel_z'),
            ' panel_yaw:=', LaunchConfiguration('panel_yaw'),
        ]),
        value_type=str,
    )

    # frame_prefix makes the published frame IDs actually be panel/* (the
    # namespace alone only namespaces topics, not tf2 frame_id strings), and
    # remapping tf/tf_static back onto the global topics puts those prefixed
    # frames on the one shared TF tree instead of an isolated panel/tf that
    # no other node's TransformListener would ever see by default.
    panel_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='panel',
        output='screen',
        parameters=[{
            'robot_description': panel_description_content,
            'use_sim_time': True,
            'frame_prefix': 'panel/',
        }],
        remappings=[
            ('/panel/tf', '/tf'),
            ('/panel/tf_static', '/tf_static'),
        ],
    )

    panel_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'indomitus_panel',
            '-topic', 'panel/robot_description',
            '-x', LaunchConfiguration('panel_x'),
            '-y', LaunchConfiguration('panel_y'),
            '-z', LaunchConfiguration('panel_z'),
            '-Y', LaunchConfiguration('panel_yaw'),
        ],
        output='screen',
    )

    return [panel_state_publisher, panel_spawn]


def generate_launch_description() -> LaunchDescription:
    rover_description_share = get_package_share_directory('rover_description')
    rover_sim_share = get_package_share_directory('rover_sim')
    panel_description_share = get_package_share_directory('panel_description')
    panel_sim_share = get_package_share_directory('panel_sim')
    arm_description_share = get_package_share_directory('arm_description')

    controllers_yaml_path = os.path.join(rover_sim_share, 'config', 'controllers_with_arm.yaml')

    resource_dirs = [
        os.path.dirname(rover_description_share),
        os.path.dirname(panel_description_share),
        os.path.dirname(arm_description_share),
    ]
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = os.pathsep.join(filter(None, resource_dirs + [existing_gz_path]))

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='world_demo'),
        DeclareLaunchArgument('model_name', default_value='indomitus_rover'),
        DeclareLaunchArgument('arm_camera', default_value='true',
                               description='Enable the arm wrist camera render/bridge'),
        DeclareLaunchArgument('spawn_panel', default_value='true',
                               description='Also spawn the switch panel task board in the same world'),
        DeclareLaunchArgument('panel_x', default_value='2.0'),
        DeclareLaunchArgument('panel_y', default_value='0.0'),
        DeclareLaunchArgument('panel_z', default_value='0.5'),
        DeclareLaunchArgument('panel_yaw', default_value='3.14159'),

        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),

        make_gazebo_launch(rover_sim_share),

        # Rover + arm as ONE combined robot_description: mount_arm bolts the
        # indomitus_arm macro onto main_body_link, and standalone_gz_plugin:=false
        # stops the arm from registering its own (redundant) gz_ros2_control-system
        # plugin -- rover.xacro's own plugin (below, via controllers_yaml_path)
        # already loads every <ros2_control> block in the combined model.
        include_launch('rover_description', 'robot_state_publisher.launch.py', {
            'xacro_file': os.path.join(rover_description_share, 'urdf', 'rover.xacro'),
            'xacro_args': [
                'use_sim:=true ',
                f'controllers_yaml_path:={controllers_yaml_path} ',
                'mount_arm:=true sim:=true standalone_gz_plugin:=false ',
                'camera:=', LaunchConfiguration('arm_camera'),
            ],
            'use_sim_time': 'true',
        }),

        OpaqueFunction(function=generate_rover_bridge_config, kwargs={'rover_sim_share': rover_sim_share}),
        make_rover_spawn_node(),

        include_launch('rover_bringup', 'control.launch.py', {
            'use_sim': 'true',
            'controllers_yaml': controllers_yaml_path,
            'controllers': (
                'joint_state_broadcaster swerve_controller odometry_controller '
                'diff_bar_effort_controller indomitus_arm_controller '
                'gripper_right_controller gripper_left_controller'
            ),
        }),

        include_launch('rover_localization', 'ekf.launch.py', {
            'use_sim_time': 'true',
        }),
        include_launch('rover_bringup', 'twist_mux.launch.py'),

        # Serves the joystick's motor / compact / clear-errors buttons, and the
        # same services the ground station calls.
        include_launch('rover_teleop', 'drive_power.launch.py', {
            'use_sim_time': 'true',
            'controller_name': 'swerve_controller_test',
        }),

        GroupAction(
            actions=[
                *make_panel_nodes(panel_description_share),
                OpaqueFunction(function=generate_panel_bridge_config,
                                kwargs={'panel_sim_share': panel_sim_share}),
            ],
            condition=IfCondition(LaunchConfiguration('spawn_panel')),
        ),
    ])
