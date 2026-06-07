import os
from typing import List

import launch.actions
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


args_descriptions = {
    "name": "Name of the robot used as a tf prefix",
    "use_rviz": "Launch RViz2 for visualization",
    "use_joint_state_publisher_gui": "Launch joint_state_publisher_gui for joint control"
}


def urdf(name: str = '') -> str:
    urdf_xacro = os.path.join(get_package_share_directory('rover_description'),
                              'urdf', 'rover_s1.urdf.xacro')
    
    xacro_args = [f'name:={name}']
    
    opts, input_file_name = xacro.process_args([urdf_xacro] + xacro_args)
    try:
        doc = xacro.process_file(input_file_name, **vars(opts))
        return doc.toprettyxml(indent='  ')
    except Exception as e:
        print(f"Error processing URDF for S1: {e}")
        return ''


def launch_nodes(context: LaunchContext,
                 **substitutions: launch.substitutions.LaunchConfiguration
                 ) -> List[Node]:
    kwargs = {k: perform_substitutions(context, [v]) for k, v in substitutions.items()}
    
    use_rviz = kwargs.get('use_rviz', 'true').lower() == 'true'
    use_joint_gui = kwargs.get('use_joint_state_publisher_gui', 'true').lower() == 'true'
    
    urdf_string = urdf(**{k: v for k, v in kwargs.items() 
                          if k not in ('use_rviz', 'use_joint_state_publisher_gui')})
    
    nodes = []
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_string, 'publish_frequency': 100.0}],
        output='screen',
        arguments=["--ros-args", "--log-level", "warn"]
    )
    nodes.append(robot_state_publisher_node)
    
    # Joint State Publisher (GUI або без GUI)
    if use_joint_gui:
        joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    else:
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    nodes.append(joint_state_publisher_node)
    
    # RViz2
    if use_rviz:
        rviz_config_file = os.path.join(
            get_package_share_directory('rover_bringup'),
            'rviz', 'robot.rviz'
        )
        # Завжди використовуємо конфігураційний файл, якщо він існує
        if os.path.exists(rviz_config_file):
            rviz_args = ['-d', rviz_config_file]
        else:
            # Якщо файл не знайдено в install директорії, спробуємо в source директорії
            source_rviz_config = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                'rover_bringup', 'rviz', 'robot.rviz'
            )
            if os.path.exists(source_rviz_config):
                rviz_args = ['-d', source_rviz_config]
            else:
                rviz_args = []
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args
        )
        nodes.append(rviz_node)
    
    return nodes


def generate_launch_description():
    urdf_args = [
        launch.actions.DeclareLaunchArgument(
            k, default_value=str(urdf.__defaults__[i]), description=args_descriptions.get(k, ''))
        for i, (k, _) in enumerate(urdf.__annotations__.items()) if k != 'return'
    ]
    
    additional_args = [
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui for joint control'
        )
    ]
    
    all_kwargs = {k: launch.substitutions.LaunchConfiguration(k)
                  for (k, _) in urdf.__annotations__.items() if k != 'return'}
    all_kwargs['use_rviz'] = launch.substitutions.LaunchConfiguration('use_rviz')
    all_kwargs['use_joint_state_publisher_gui'] = launch.substitutions.LaunchConfiguration('use_joint_state_publisher_gui')
    
    return LaunchDescription(
        urdf_args + additional_args + [
            launch.actions.OpaqueFunction(
                function=launch_nodes,
                kwargs=all_kwargs),
        ])