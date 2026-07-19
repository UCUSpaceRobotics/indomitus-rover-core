from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import SetRemap # <-- Import SetRemap

DEFAULT_ZED2I_CONFIG_NAME = "zed2i.yaml"
CAMERA_MODEL = "zed2i"

def generate_launch_description():
    child_launch_file_path = PathJoinSubstitution([
        FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"
    ])
    config_path = PathJoinSubstitution([
        FindPackageShare("rover_sensors"), "config", DEFAULT_ZED2I_CONFIG_NAME
    ])

    config_path_argument = DeclareLaunchArgument(
        name="config_path",
        default_value=config_path,
        description="Path to the config for the ZED 2i stereo camera"
    )

    launch_file_with_remappings = GroupAction(
        actions=[
            SetRemap(src='/zed2i/zed_node/odom', dst='/zed2i/odom'),
            SetRemap(src='/zed2i/zed_node/point_cloud/cloud_registered', dst='/zed2i/points'),

            # RGB Images
            SetRemap(src='/zed2i/zed_node/rgb/color/rect/image', dst='/zed2i/rgb/image_rect_color'),
            SetRemap(src='/zed2i/zed_node/rgb/color/rect/camera_info', dst='/zed2i/rgb/camera_info'),

            # Depth Images
            SetRemap(src='/zed2i/zed_node/depth/depth_registered', dst='/zed2i/depth/depth_registered'),
            SetRemap(src='/zed2i/zed_node/depth/camera_info', dst='/zed2i/depth/camera_info'),

            # Pose & IMU
            SetRemap(src='/zed2i/zed_node/pose', dst='/zed2i/pose'),
            SetRemap(src='/zed2i/zed_node/imu/data', dst='/zed2i/imu/data'),

            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(child_launch_file_path),
                launch_arguments={
                    "camera_model": CAMERA_MODEL,
                    "ros_params_override_path": LaunchConfiguration("config_path"),
                    "camera_name": CAMERA_MODEL,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        config_path_argument,
        launch_file_with_remappings,
    ])
