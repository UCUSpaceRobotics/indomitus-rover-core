import os
from ament_index_python.packages import get_package_share_directory
from launch import Condition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_launch(
    package: str, launch_file: str, launch_arguments: dict | None = None,
    condition: Condition | None = None,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )
