"""
Launch file for MoveIt demo + moveit_servo.

Servo streams joint *positions* (Float64MultiArray) to
indomitus_arm_forward_position_controller. That controller is spawned
inactive; arm_tasks switches JTC <-> forward around home / start_servo.

servo_node still waits until indomitus_arm_controller (JTC) has been
spawned successfully so /joint_states and the hardware stack are up.
"""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

ARM_CONTROLLER_NAME = "indomitus_arm_controller"
FORWARD_CONTROLLER_NAME = "indomitus_arm_forward_position_controller"


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def _arg_from_argv(name: str, default: str) -> str:
    """Plain-str mappings force MoveItConfigsBuilder's single-eval xacro path — LaunchConfiguration ones desync across sub-launches."""
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return default


def _arg_from_argv_or_none(name: str):
    """Like _arg_from_argv(), but None (no fallback) instead of a default."""
    prefix = f"{name}:="
    for arg in sys.argv:
        if arg.startswith(prefix):
            return arg[len(prefix):]
    return None


def generate_launch_description() -> LaunchDescription:
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Whether to use fake hardware or real CAN bus",
    )

    end_effector = LaunchConfiguration("end_effector")
    declare_end_effector_cmd = DeclareLaunchArgument(
        "end_effector",
        default_value="jaw",
        description="'jaw', 'other_tool', or 'drill_sampling' — see arm_macro.xacro",
    )

    declare_can_interface_cmd = DeclareLaunchArgument(
        "can_interface",
        default_value="",
        description="SocketCAN interface for joints + end-effector; unset uses each component's own default.",
    )

    declare_bring_up_can_bridge_cmd = DeclareLaunchArgument(
        "bring_up_can_bridge",
        default_value="true",
        description="Bring up our own ros2_socketcan bridge; set false if rover.launch.py already did.",
    )
    bring_up_can_bridge = LaunchConfiguration("bring_up_can_bridge")

    declare_report_collisions_cmd = DeclareLaunchArgument(
        "report_collisions",
        default_value="true",
        description=(
            "Run collision_link_reporter alongside move_group — logs exactly "
            "which link pair is in contact (via /check_state_validity) "
            "whenever the current pose goes invalid. Read-only, cheap "
            "(2 Hz polling); set false to silence it."
        ),
    )

    # Runtime (LaunchConfiguration) form, for the IfCondition/UnlessCondition
    # gripper-spawner split below — separate from the plain-string
    # _arg_from_argv() value passed into robot_description(mappings=...),
    # which must stay a plain string (see _arg_from_argv's own docstring).
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # planning_pipelines restricted to ompl on purpose — same fix as
    # arm_gazebo.launch.py (see its own comment for the full story): with
    # chomp/pilz_industrial_motion_planner also loaded, move_group picks
    # an ambiguous "planning_plugin" between them and silently falls back
    # to CHOMP even when a request explicitly asks for pipeline_id='ompl'
    # (as panel_align_node.py's align requests do), and CHOMP rejects any
    # Cartesian pose-constraint goal outright. This is the real-hardware
    # launch path, so panel align needs the same fix here as in sim.
    robot_description_mappings = {
        "use_fake_hardware": _arg_from_argv("use_fake_hardware", "true"),
        "end_effector": _arg_from_argv("end_effector", "jaw"),
    }
    can_interface_from_argv = _arg_from_argv_or_none("can_interface")
    if can_interface_from_argv is not None:
        robot_description_mappings["can_interface"] = can_interface_from_argv

    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings=robot_description_mappings)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml["moveit_servo"]["ros__parameters"]}

    # Light smoothing only: heavy coeffs lag the command and, with high MIT
    # kp, the arm overshoots then waits — stop-go buzz at the Servo rate.
    smoothing_params = {"butterworth_filter_coeff": 2.0}

    # Do NOT pass robot_description_kinematics into servo_node.
    # Humble Servo prefers the KDL plugin (searchPositionIK, 5 ms timeout)
    # over the inverse Jacobian. From home, +X (W) often fails that IK so
    # cartesianServoCalcs() returns false and the last joint command is
    # republished (frozen commands, status still 0). -X (S) succeeds, so
    # teleop looks one-sided. Without a solver, Servo uses J^# which is
    # symmetric in ±X. move_group still gets KDL for Plan&Execute.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # Without this, joint_limits.yaml (per-joint max_velocity/
            # max_acceleration/position bounds) never reaches Servo's robot
            # model — it falls back to the bare URDF <limit> tags only, so
            # the wrist_1_wrist_2 singularity fence and the tuned speed
            # ceilings were silently not enforced during teleop.
            moveit_config.joint_limits,
            servo_params,
            smoothing_params,
        ],
    )

    def _on_process_exit(event: ProcessExited, context: LaunchContext):
        cmd_parts = [str(part) for part in (event.cmd or [])]

        is_spawner = any(part.endswith("spawner") for part in cmd_parts)
        has_controller_arg = ARM_CONTROLLER_NAME in cmd_parts

        if not (is_spawner and has_controller_arg):
            return None

        if event.returncode != 0:
            print(
                f"[servo_launch] Spawner for '{ARM_CONTROLLER_NAME}' "
                f"exited with code {event.returncode} — servo_node will NOT be started."
            )
            return None

        print(
            f"[servo_launch] Controller '{ARM_CONTROLLER_NAME}' is active — "
            f"starting servo_node (streams to '{FORWARD_CONTROLLER_NAME}'; "
            f"activate that controller before teleop via gamepad A / start_servo)."
        )
        return [servo_node]

    # move_group/moveit_servo are prebuilt binaries (not vendored here), so
    # their own collision-check code can't be patched to print link names —
    # this polls the same /check_state_validity service move_group already
    # exposes and logs the colliding link pair by name. See
    # arm_tasks/collision_link_reporter.py.
    collision_link_reporter = Node(
        package="arm_tasks",
        executable="collision_link_reporter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("report_collisions")),
    )

    # Load the streaming teleop controller inactive — JTC owns the joints until
    # arm_tasks switches controllers for Servo.
    forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[FORWARD_CONTROLLER_NAME, "--inactive"],
        output="screen",
    )

    demo_launch = generate_demo_launch(moveit_config)

    ld = LaunchDescription()
    # DeclareLaunchArgument (unlike SetLaunchConfiguration) only applies its
    # default when the arg isn't already set — so use_rviz:=true on the CLI
    # still wins over this, and generate_demo_launch()'s own default (true)
    # never gets a chance to apply since this one runs first.
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="false"))
    ld.add_action(declare_use_fake_hardware_cmd)
    ld.add_action(declare_end_effector_cmd)
    ld.add_action(declare_can_interface_cmd)
    ld.add_action(declare_bring_up_can_bridge_cmd)
    ld.add_action(declare_report_collisions_cmd)
    for action in demo_launch.entities:
        ld.add_action(action)
    ld.add_action(forward_spawner)
    ld.add_action(collision_link_reporter)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=None,
                on_exit=_on_process_exit,
            )
        )
    )

    # Gripper controllers aren't spawned by generate_demo_launch(); only jaw
    # has finger joints, and which finger interfaces exist depends on
    # use_fake_hardware too: fake hardware (or sim) exposes both fingers,
    # but real hardware only stubs the right one (JawGripperStub — the real
    # arm has no gripper motor yet, see arm_macro.xacro) with no left-finger
    # interface at all, so spawning gripper_left_controller there would just
    # fail waiting for an interface that will never exist.
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_right_controller",
                "gripper_left_controller",
                "--controller-manager-timeout", "60",
                "--switch-timeout", "60",
                "--service-call-timeout", "70",
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", end_effector, "' == 'jaw' and '", use_fake_hardware, "' == 'true'"
            ])),
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_right_controller",
                "--controller-manager-timeout", "60",
                "--switch-timeout", "60",
                "--service-call-timeout", "70",
            ],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", end_effector, "' == 'jaw' and '", use_fake_hardware, "' != 'true'"
            ])),
        )
    )

    # ---- End-effector CAN link (real hardware only) ----
    real_hardware_condition = IfCondition(PythonExpression([
        "'", use_fake_hardware, "' != 'true'"
    ]))

    end_effector_can_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("arm_peripherals"),
                         "launch", "end_effector_can.launch.py")
        ),
        launch_arguments={"end_effector": end_effector}.items(),
        condition=real_hardware_condition,
    )

    # reuses rover_bringup's ros2_socketcan bridge; filter covers only
    # jaw/astrobio/drill_sampling cmd/ack pairs (0x1A-0x1F), not joint IDs.
    can_bridge_condition = IfCondition(PythonExpression([
        "'", bring_up_can_bridge, "' == 'true' and '", use_fake_hardware, "' != 'true'"
    ]))

    can_bridge_launch_arguments = {"receiver_filters": "1A:7FE,1C:7FE,1E:7FE"}
    if can_interface_from_argv is not None:
        can_bridge_launch_arguments["interface"] = can_interface_from_argv

    can_bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rover_bringup"),
                         "launch", "can.launch.py")
        ),
        launch_arguments=can_bridge_launch_arguments.items(),
        condition=can_bridge_condition,
    )

    ld.add_action(can_bridge_include)
    ld.add_action(end_effector_can_include)

    return ld
