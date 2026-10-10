import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _arg_from_argv(name: str, default: str) -> str:
    # See rsp.launch.py's _arg_from_argv.
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def generate_launch_description():
    # planning_pipelines restricted to ompl on purpose — see the matching
    # comment in arm_sim/launch/arm_gazebo.launch.py: without this,
    # move_group ambiguously picks CHOMP, which rejects panel_align_node's
    # Cartesian pose-constraint goals outright (INVALID_GOAL_CONSTRAINTS).
    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings={
            "use_fake_hardware": _arg_from_argv("use_fake_hardware", "true"),
            "end_effector": _arg_from_argv("end_effector", "jaw"),
        })
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    if os.environ.get("ROS_DISTRO") == "humble":
        # ompl_planning.yaml is in Jazzy format, see ompl_planning_humble.yaml.
        ompl = moveit_config.planning_pipelines["ompl"]
        for key in ("planning_plugins", "request_adapters", "response_adapters"):
            ompl.pop(key, None)
        config_dir = os.path.join(get_package_share_directory("arm_moveit_config"), "config")
        with open(os.path.join(config_dir, "ompl_planning_humble.yaml")) as f:
            ompl.update(yaml.safe_load(f))
    return generate_move_group_launch(moveit_config)
