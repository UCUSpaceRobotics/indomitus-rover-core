import sys

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def _arg_from_argv(name: str, default: str) -> str:
    # generate_demo_launch() pulls this file in via IncludeLaunchDescription
    # rather than calling generate_rsp_launch(moveit_config) with demo.launch.py's
    # own end_effector/use_fake_hardware-aware moveit_config — so this file must
    # resolve them itself or it silently falls back to xacro's defaults (jaw,
    # fake hardware) no matter what was passed on the command line. See
    # demo.launch.py's _arg_from_argv for the full story (xacro mappings must
    # be plain str, not LaunchConfiguration, for MoveItConfigsBuilder to
    # actually apply them here).
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
