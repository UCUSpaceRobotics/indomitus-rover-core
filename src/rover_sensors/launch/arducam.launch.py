import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext, *args, **kwargs):
    namespace_val = LaunchConfiguration("namespace").perform(context)
    camera_name_val = LaunchConfiguration("camera_name").perform(context)
    config_path_val = LaunchConfiguration("config_path").perform(context)

    default_config_file = os.path.join(
        get_package_share_directory("rover_sensors"), "config", "arducam",
        f"{camera_name_val}.yaml",
    )
    config_file = config_path_val if config_path_val else default_config_file

    camera_path_val = LaunchConfiguration("camera_path").perform(context)
    camera_info_url_val = LaunchConfiguration("camera_info_url").perform(context)
    camera_frame_id_val = LaunchConfiguration("camera_frame_id").perform(context)

    actions = []
    if not os.path.exists(camera_path_val):
        actions.append(LogInfo(
            msg=f"[arducam:{camera_name_val}] {camera_path_val} not found. Check that this "
                "camera is plugged into its designated hub port (1=mast, 2=rear, "
                "3=container), then relaunch."
        ))

    throttle_rate_val = LaunchConfiguration("throttle_rate").perform(context)
    throttle_rate_override = (
        {"msgs_per_sec": float(throttle_rate_val)} if throttle_rate_val else {}
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="arducam_node",
        namespace=namespace_val,
        output="screen",
        parameters=[
            config_file,
            {
                "video_device": camera_path_val,
                "camera_info_url": camera_info_url_val,
                "camera_frame_id": camera_frame_id_val,
            },
        ],
        respawn=True,       # Crucial: Restarts node if USB cable wiggles loose
        respawn_delay=2.0,
    )

    throttle_raw_node = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_raw",
        namespace=namespace_val,
        output="screen",
        arguments=["messages"],
        parameters=[
            config_file,
            {
                "throttle_type": "messages",
                "input_topic": "image_raw",
                "output_topic": "image_raw_slow",
                **throttle_rate_override,
            },
        ]
    )

    throttle_compressed_node = Node(
        package="topic_tools",
        executable="throttle",
        name="throttle_compressed",
        namespace=namespace_val,
        output="screen",
        arguments=["messages"],
        parameters=[
            config_file,
            {
                "throttle_type": "messages",
                "input_topic": "image_raw/compressed",
                "output_topic": "image_raw_slow/compressed",
                **throttle_rate_override,
            },
        ]
    )

    actions += [v4l2_camera_node, throttle_raw_node, throttle_compressed_node]
    return actions


def generate_launch_description():
    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="mast",
        description="Which arducam is being launched. Selects the default config "
                     "file config/arducam/<camera_name>.yaml; ignored if config_path "
                     "is set.",
        choices=["mast", "rear", "container"],
    )

    declare_camera_path = DeclareLaunchArgument(
        "camera_path",
        default_value="/dev/video0",
        description="Path to the camera device (e.g., /dev/video0 or /dev/camera_left)",
    )

    declare_camera_info_url = DeclareLaunchArgument(
        "camera_info_url",
        default_value="",
        description="URL to the camera calibration file (e.g., file:///path/to/cal.yaml)",
    )

    declare_camera_frame_id = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="camera",
        description="TF frame ID attached to the published image headers "
                     "(e.g., rear_arducam_optical_frame or mast_arducam_optical_frame)",
    )

    declare_config_path = DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="Full path to the YAML configuration file with camera parameters. "
                     "Overrides camera_name's config/arducam/<camera_name>.yaml when set.",
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="camera",
        description="Namespace for the camera node",
    )

    declare_throttle_rate = DeclareLaunchArgument(
        "throttle_rate",
        default_value="",
        description="Maximum messages per second for the throttled image_raw / "
                     "image_raw/compressed output topics, published on "
                     "image_raw_slow and image_raw_slow/compressed. Overrides the "
                     "msgs_per_sec value set in the selected config file when set.",
    )

    return LaunchDescription([
        declare_camera_name,
        declare_camera_path,
        declare_camera_info_url,
        declare_camera_frame_id,
        declare_config_path,
        declare_namespace,
        declare_throttle_rate,
        OpaqueFunction(function=_launch_setup),
    ])
