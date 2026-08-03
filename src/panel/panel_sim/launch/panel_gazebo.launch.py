import os
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = "panel_sim_world"  # must match the <world name="..."> in worlds/empty.sdf


def generate_launch_description() -> LaunchDescription:
    panel_description_dir = get_package_share_directory("panel_description")
    panel_sim_dir = get_package_share_directory("panel_sim")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(panel_description_dir, "urdf", "panel_standalone.urdf.xacro")
    world_file = os.path.join(panel_sim_dir, "worlds", "empty.sdf")

    bridge_template = os.path.join(panel_sim_dir, "config", "gz_bridge.yaml")
    with open(bridge_template) as f:
        rendered = Template(f.read()).substitute(world=WORLD_NAME)
    bridge_config = f"/tmp/panel_bridge_{WORLD_NAME}.yaml"
    with open(bridge_config, "w") as f:
        f.write(rendered)

    resource_path_root = os.path.dirname(panel_description_dir)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_gz_path]))
    ign_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_ign_path]))

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file, " sim:=true"]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 4 {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description", "-name", "indomitus_panel", "-z", "0.2",
            "--ros-args", "--log-level", "debug",
        ],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
        ]
    )
