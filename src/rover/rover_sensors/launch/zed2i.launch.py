import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

COMMON_CONFIG_NAME = "zed2i_common.yaml"
RGB_CONFIG_NAME = "zed2i_rgb.yaml"
NAV_CONFIG_NAME = "zed2i_nav.yaml"
CAMERA_MODEL = "zed2i"


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge `override` into `base`, with `override` winning leaf conflicts."""
    merged = dict(base)
    for key, value in override.items():
        if isinstance(merged.get(key), dict) and isinstance(value, dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _resolve_config_file(mode: str, config_path: str) -> str:
    """
    `zed_camera.launch.py`'s `ros_params_override_path` only accepts a single file, so the
    general/video/sensors settings shared by every mode (zed2i_common.yaml) and the
    mode-specific overrides (zed2i_rgb.yaml / zed2i_nav.yaml) are merged here into one temp
    file instead of duplicating the shared block in both mode config files.
    """
    if config_path:
        return config_path

    config_dir = os.path.join(get_package_share_directory("rover_sensors"), "config")
    mode_config_name = NAV_CONFIG_NAME if mode == "nav" else RGB_CONFIG_NAME

    with open(os.path.join(config_dir, COMMON_CONFIG_NAME)) as f:
        merged_config = yaml.safe_load(f)
    with open(os.path.join(config_dir, mode_config_name)) as f:
        merged_config = _deep_merge(merged_config, yaml.safe_load(f))

    merged_file = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix=f"zed2i_{mode}_", delete=False,
    )
    yaml.safe_dump(merged_config, merged_file, sort_keys=False, default_flow_style=False)
    merged_file.close()
    return merged_file.name


def _remove_file(path: str):
    def _on_shutdown(event, context):
        if os.path.exists(path):
            os.remove(path)
    return _on_shutdown


def _launch_setup(context: LaunchContext, *args, **kwargs):
    child_launch_file_path = os.path.join(
        get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"
    )

    config_path_arg = LaunchConfiguration("config_path").perform(context)
    config_file = _resolve_config_file(
        mode=LaunchConfiguration("mode").perform(context),
        config_path=config_path_arg,
    )

    actions = [
        GroupAction(
            actions=[
                # Odometry
                SetRemap(src="zed2i/zed_node/odom", dst="zed2i/odom"),

                # RGB Images
                SetRemap(src="zed2i/zed_node/rgb/color/rect/image", dst="zed2i/rgb/image_rect_color"),
                SetRemap(src="zed2i/zed_node/rgb/color/rect/camera_info", dst="zed2i/rgb/camera_info"),

                # Pointcloud & Depth Images
                SetRemap(src="zed2i/zed_node/point_cloud/cloud_registered", dst="zed2i/points"),
                SetRemap(src="zed2i/zed_node/depth/depth_registered", dst="zed2i/depth/depth_registered"),
                SetRemap(src="zed2i/zed_node/depth/camera_info", dst="zed2i/depth/camera_info"),

                # Pose & IMU
                SetRemap(src="zed2i/zed_node/pose", dst="zed2i/pose"),
                SetRemap(src="zed2i/zed_node/imu/data", dst="zed2i/imu/data"),

                # tf/tf_static stay global regardless of the pushed rover
                # namespace - see docs/software/tf_ownership.md.
                SetRemap(src="tf", dst="/tf"),
                SetRemap(src="tf_static", dst="/tf_static"),

                IncludeLaunchDescription(
                    launch_description_source=PythonLaunchDescriptionSource(child_launch_file_path),
                    launch_arguments={
                        "camera_model": CAMERA_MODEL,
                        "ros_params_override_path": config_file,
                        "camera_name": CAMERA_MODEL,
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "publish_map_tf": LaunchConfiguration("publish_map_tf"),
                        "publish_urdf": LaunchConfiguration("publish_urdf_tf"),
                        "publish_imu_tf": LaunchConfiguration("publish_imu_tf"),
                    }.items()
                ),
            ]
        )
    ]

    if not config_path_arg:
        # config_file is a merged temp file only when config_path wasn't set explicitly;
        # remove it on shutdown instead of leaking it into the OS temp dir on every launch.
        actions.append(
            RegisterEventHandler(OnShutdown(on_shutdown=_remove_file(config_file)))
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="rgb",
        description=(
            "Camera operation mode: 'rgb' publishes only the rectified color feed for the "
            "operator, 'nav' additionally enables the point cloud and VIO used by navigation stack. "
            "Selects the default config file, unless `config_path` is set explicitly."
        ),
        choices=["rgb", "nav"],
    )

    config_path_arg = DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description=(
            "Full path to the config for the ZED 2i stereo camera. Overrides the default "
            "config chosen by `mode` when set - zed2i_common.yaml is not merged in that case, "
            "the file is used as-is."
        ),
    )

    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="Enable publication of the `odom -> camera_link` TF.",
        choices=["true", "false"],
    )

    publish_map_tf_arg = DeclareLaunchArgument(
        "publish_map_tf",
        default_value="false",
        description="Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.",
        choices=["true", "false"],
    )

    publish_urdf_arg = DeclareLaunchArgument(
        "publish_urdf_tf",
        default_value="false",
        description="Enable URDF processing and starts Robot State Published to propagate static TF.",
        choices=["true", "false"],
    )

    publish_imu_tf_arg = DeclareLaunchArgument(
        "publish_imu_tf",
        default_value="true",
        description="Enable publication of the static `zed2i_left_camera_frame -> "
                     "zed2i_imu_link` TF. Unlike the other TF args this does not "
                     "conflict with robot_state_publisher: rover_description's URDF "
                     "never defines the IMU link, so the ZED node's factory-calibrated "
                     "extrinsic is the only source for it.",
        choices=["true", "false"],
    )

    return LaunchDescription([
        mode_arg,
        config_path_arg,
        publish_tf_arg,
        publish_map_tf_arg,
        publish_urdf_arg,
        publish_imu_tf_arg,
        OpaqueFunction(function=_launch_setup),
    ])
