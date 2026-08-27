import sys

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def _arg_from_argv(name: str, default: str) -> str:
    # This file builds its own moveit_config, bypassing demo.launch.py's — must
    # re-resolve args itself. See demo.launch.py's _arg_from_argv.
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings={
            "use_fake_hardware": _arg_from_argv("use_fake_hardware", "true"),
            "end_effector": _arg_from_argv("end_effector", "jaw"),
        })
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config)
