import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description() -> LaunchDescription:
    """Gazebo sim plus RViz in one launch.

    Thin wrapper: includes arm_gazebo.launch.py unchanged (Gazebo,
    ros2_control, move_group, servo) and adds the same RViz setup that
    arm_moveit_config's moveit_rviz.launch.py provides. Use this instead
    of running demo.launch.py next to Gazebo — demo.launch.py starts its
    own mock-hardware controller_manager, which conflicts with the
    Gazebo-backed one (duplicate /joint_states publishers).
    """
    arm_sim_dir = get_package_share_directory("arm_sim")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_sim_dir, "launch", "arm_gazebo.launch.py")
        ),
        launch_arguments={"camera": LaunchConfiguration("camera")}.items(),
    )

    moveit_config = MoveItConfigsBuilder(
        "indomitus_arm", package_name="arm_moveit_config"
    ).to_moveit_configs()
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera", default_value="true"),
            SetParameter(name="use_sim_time", value=True),
            gazebo_launch,
            *rviz_launch.entities,
        ]
    )
