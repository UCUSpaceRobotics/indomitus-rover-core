from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, NotEqualsSubstitution
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import SetRemap

DEFAULT_ZED2I_CONFIG_NAME = "zed2i.yaml"
CAMERA_MODEL = "zed2i"


def generate_launch_description():
    child_launch_file_path = PathJoinSubstitution([
        FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"
    ])
    default_config_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors"), "config", DEFAULT_ZED2I_CONFIG_NAME
    ])

    config_path_val = LaunchConfiguration("config_path")
    config_path_arg = DeclareLaunchArgument(
        name="config_path",
        default_value=default_config_path,
        description="Path to the config for the ZED 2i stereo camera",
    )

    publish_tf_val = LaunchConfiguration("publish_tf")
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="Enable publication of the `odom -> camera_link` TF.",
        choices=["true", "false"],
    )

    publish_map_tf_val = LaunchConfiguration("publish_map_tf")
    publish_map_tf_arg = DeclareLaunchArgument(
        "publish_map_tf",
        default_value="false",
        description="Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.",
        choices=["true", "false"],
    )

    publish_urdf_tf_val = LaunchConfiguration("publish_urdf_tf")
    publish_urdf_arg = DeclareLaunchArgument(
        "publish_urdf_tf",
        default_value="false",
        description="Enable URDF processing and starts Robot State Published to propagate static TF.",
        choices=["true", "false"],
    )

    custom_config_file_present = NotEqualsSubstitution(config_path_val, "")
    config_file = IfElseSubstitution(
        custom_config_file_present,
        if_value=config_path_val,
        else_value=default_config_path,
    )

    launch_file_with_remappings = GroupAction(
        actions=[
            # Odometry
            SetRemap(src="/zed2i/zed_node/odom", dst="/zed2i/odom"),

            # RGB Images
            SetRemap(src="/zed2i/zed_node/rgb/color/rect/image", dst="/zed2i/rgb/image_rect_color"),
            SetRemap(src="/zed2i/zed_node/rgb/color/rect/camera_info", dst="/zed2i/rgb/camera_info"),

            # Pointcloud & Depth Images
            SetRemap(src="/zed2i/zed_node/point_cloud/cloud_registered", dst="/zed2i/points"),
            SetRemap(src="/zed2i/zed_node/depth/depth_registered", dst="/zed2i/depth/depth_registered"),
            SetRemap(src="/zed2i/zed_node/depth/camera_info", dst="/zed2i/depth/camera_info"),

            # Pose & IMU
            SetRemap(src="/zed2i/zed_node/pose", dst="/zed2i/pose"),
            SetRemap(src="/zed2i/zed_node/imu/data", dst="/zed2i/imu/data"),

            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(child_launch_file_path),
                launch_arguments={
                    "camera_model": CAMERA_MODEL,
                    "ros_params_override_path": config_file,
                    "camera_name": CAMERA_MODEL,
                    "publish_tf": publish_tf_val,
                    "publish_map_tf": publish_map_tf_val,
                    "publish_urdf": publish_urdf_tf_val,
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        config_path_arg,
        publish_tf_arg,
        publish_map_tf_arg,
        publish_urdf_arg,
        launch_file_with_remappings,
    ])