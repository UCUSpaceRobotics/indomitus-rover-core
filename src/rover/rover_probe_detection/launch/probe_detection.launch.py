"""Probe detection: per-camera 2D detector + multi-camera triangulator.

Needs rover_cameras (images, camera_info) and the robot_state_publishers
(camera frames in TF). See rover_probe_detection/probe_triangulator_node.py.

fake_probes:="x,y,z;x,y,z" (in base_footprint) replaces the detector with
fake_detections_node, which projects those points into the cameras, for testing
without a model.

Namespace: pushes rover_namespace (default $ROVER_NAMESPACE or 'rover'); an
including launch that already pushed it passes rover_namespace:=''.

    ros2 launch rover_probe_detection probe_detection.launch.py
    ros2 launch rover_probe_detection probe_detection.launch.py fake_probes:="1.5,0.3,0;2,-0.5,0"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def parse_probes(text):
    """[x1, y1, z1, ...] from "x,y,z;x,y,z"."""
    flat = []
    for probe in filter(None, (s.strip() for s in text.split(';'))):
        xyz = [float(v) for v in probe.split(',')]
        if len(xyz) != 3:
            raise RuntimeError(f'fake_probes: {probe!r} is not x,y,z')
        flat += xyz
    return flat


def _nodes(context):
    config = PathJoinSubstitution(
        [FindPackageShare('rover_probe_detection'), 'config', 'probe_detection.yaml'])
    common = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    fake = parse_probes(LaunchConfiguration('fake_probes').perform(context))
    if fake:
        detector = Node(package='rover_probe_detection', executable='fake_detections_node',
                        name='fake_probe_detections', output='screen',
                        parameters=[config, common, {'probes': fake}])
    else:
        detector = Node(package='rover_probe_detection', executable='probe_detector_node',
                        name='probe_detector', output='screen', parameters=[config, common])
    triangulator = Node(package='rover_probe_detection', executable='probe_triangulator_node',
                        name='probe_triangulator', output='screen', parameters=[config, common])
    namespace = LaunchConfiguration('rover_namespace').perform(context)
    return [GroupAction(([PushRosNamespace(namespace)] if namespace else [])
                        + [detector, triangulator])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description="Namespace to push; '' when the including launch already did"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'fake_probes', default_value='',
            description='"x,y,z;..." in base_footprint: fake detections instead of the detector'),
        OpaqueFunction(function=_nodes),
    ])
