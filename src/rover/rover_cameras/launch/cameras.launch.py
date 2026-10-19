"""The rover's cameras: one camera_node per entry of config/cameras.yaml.

Each serves MJPEG over HTTP to the ground station (same URLs as the GS's
start-cameras.sh) and publishes cameras/<name>/image_raw + camera_info for
nodes on the rover. See rover_cameras/camera_node.py. dataset_node adds
cameras/capture and cameras/record[/toggle] for all started cameras at once.

Stop the GS's camera servers first (`start-cameras.sh --stop`): a V4L2 camera
streams to one process only.

Namespace: pushes rover_namespace (default $ROVER_NAMESPACE or 'rover'), so it
works when run on its own; a launch that already pushed it includes this with
rover_namespace:=''.

    ros2 launch rover_cameras cameras.launch.py
    ros2 launch rover_cameras cameras.launch.py cameras:=arducam-front,arm-camera
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# cameras.yaml keys passed to the node as parameters of the same name.
PASSTHROUGH = ('device', 'format', 'fps', 'port', 'quality', 'save_quality',
               'jpeg_encoder', 'ros_fps', 'ros_encoding', 'save_dir', 'rotate_180')


def _size(value, key, name):
    if not value:
        return 0, 0
    try:
        w, h = str(value).lower().split('x')
        return int(w), int(h)
    except ValueError:
        raise RuntimeError(f'cameras.yaml: {name}.{key} must be WxH, got {value!r}') from None


def camera_params(name, cam, defaults, calibration_dir):
    """Parameters of the camera_node for cameras.yaml entry `name`."""
    c = dict(defaults or {}, **(cam or {}))
    if 'device' not in c:
        raise RuntimeError(f'cameras.yaml: {name} has no device')
    ros_name = name.replace('-', '_')
    params = {k: c[k] for k in PASSTHROUGH if k in c}
    params['name'] = name
    params['fps'] = float(c.get('fps', 15))
    params['ros_fps'] = float(c.get('ros_fps', 5))
    params['width'], params['height'] = _size(c.get('res', '960x600'), 'res', name)
    params['ros_width'], params['ros_height'] = _size(c.get('ros_res', ''), 'ros_res', name)
    params['frame_id'] = c.get('frame_id', f'{ros_name}_optical_frame')

    calib = c.get('calibration') or f'{name}.yaml'
    path = calib if os.path.isabs(calib) else os.path.join(calibration_dir, calib)
    if os.path.isfile(path):
        params['camera_info_file'] = path
    elif 'calibration' in c:
        raise RuntimeError(f'cameras.yaml: {name}.calibration {path} does not exist')
    return ros_name, params


def _nodes(context):
    config = LaunchConfiguration('config').perform(context)
    wanted = [n for n in LaunchConfiguration('cameras').perform(context).split(',') if n]
    with open(config) as f:
        conf = yaml.safe_load(f) or {}
    cameras = conf.get('cameras') or {}
    unknown = [n for n in wanted if n not in cameras]
    if unknown:
        raise RuntimeError(f'cameras.launch.py: not in {config}: {unknown} '
                           f'(have {list(cameras)})')
    calibration_dir = os.path.join(
        get_package_share_directory('rover_cameras'), 'config', 'calibration')

    nodes, ros_names = [], []
    for name, cam in cameras.items():
        if wanted and name not in wanted:
            continue
        ros_name, params = camera_params(name, cam, conf.get('defaults'), calibration_dir)
        ros_names.append(ros_name)
        nodes.append(Node(
            package='rover_cameras',
            executable='camera_node',
            name=ros_name,
            namespace='cameras',
            output='screen',
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        ))

    # cameras/capture, cameras/record[/toggle] for all of them at once.
    nodes.append(Node(
        package='rover_cameras',
        executable='dataset_node',
        name='dataset',
        namespace='cameras',
        output='screen',
        parameters=[{'cameras': ros_names or ['']}],
        respawn=True,
        respawn_delay=2.0,
    ))

    namespace = LaunchConfiguration('rover_namespace').perform(context)
    return [GroupAction(([PushRosNamespace(namespace)] if namespace else []) + nodes)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description="Namespace to push; '' when the including launch already did"),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(get_package_share_directory('rover_cameras'),
                                       'config', 'cameras.yaml'),
            description='Camera list (see the file for its keys)'),
        DeclareLaunchArgument(
            'cameras', default_value='',
            description="Comma-separated camera names to start; '' = all in config"),
        OpaqueFunction(function=_nodes),
    ])
