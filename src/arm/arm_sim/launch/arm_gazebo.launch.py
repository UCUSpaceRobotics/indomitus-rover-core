import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description() -> LaunchDescription:
    arm_description_dir = get_package_share_directory("arm_description")
    arm_sim_dir = get_package_share_directory("arm_sim")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(arm_description_dir, "urdf", "arm_standalone.urdf.xacro")
    world_file = os.path.join(arm_sim_dir, "worlds", "empty.sdf")
    bridge_config = os.path.join(arm_sim_dir, "config", "gz_bridge.yaml")

    # Gazebo needs to know where to look for the "arm_description" folder
    # to resolve package://arm_description/meshes/*.stl URIs used in the
    # xacro visuals/collisions. The resource path is the *parent* of the
    # package's own share dir, since gz appends "arm_description/..." to
    # each entry when resolving package:// URIs.
    resource_path_root = os.path.dirname(arm_description_dir)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_gz_path]))
    ign_resource_path = os.pathsep.join(filter(None, [resource_path_root, existing_ign_path]))

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file, " sim:=true"]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "indomitus_arm", "-z", "0.05"],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

    # NOTE (spawner readiness):
    # gz_ros2_control brings up /controller_manager only once the model has
    # actually been spawned inside Gazebo (the <gazebo> plugin activates as
    # part of the spawned entity). `ros_gz_sim create` is a one-shot process
    # that exits as soon as the entity is spawned. If the controller
    # spawner runs in parallel with spawn_entity, it can hit
    # /controller_manager before it exists and fail with
    # "service not available". So, same pattern as demo.launch.py: we wait
    # for spawn_entity to exit, then start the spawner via OnProcessExit.
    #
    # NOTE (single spawner call):
    # Both controllers are spawned via ONE `spawner` invocation instead of
    # two parallel ones. Each spawner process independently calls the
    # controller_manager's switch_controller service; running two of them
    # at once races against each other (one hangs until the internal
    # switch timeout, the other aborts because the controller set is
    # mid-transition, i.e. "Switch controller timed out" /
    # "Aborting, no controller is switched! (::STRICT switch)"). Passing
    # both controller names to a single spawner issues one switch request
    # for both controllers, which avoids the race entirely.

    # NOTE (switch-timeout workaround):
    # controller_manager's switch_controller has a HARDCODED internal
    # ~5s timeout for confirming a controller's activation (introduced by
    # ros2_control PR #1638), separate from --controller-manager-timeout
    # (which only governs waiting for services to become available). If
    # gz_ros2_control's Update() hasn't ticked enough times within that 5s
    # window (slow startup / rendering), activation fails EVERY time,
    # regardless of which controller or how many spawner processes are
    # used. This is a known upstream issue:
    # https://github.com/ros-controls/gz_ros2_control/issues/421
    # Fix (PR https://github.com/ros-controls/ros2_control/pull/1790):
    # explicitly raise the switch timeout via an extra spawner flag.
    # The exact flag name depends on your installed ros2_control version
    # run `ros2 run controller_manager spawner --help` to confirm which
    # one your Humble install exposes and edit accordingly:
    #   --switch-timeout <seconds>            (older PR revisions)
    #   --controller-manager-switch-timeout   (newer revisions)

    # NOTE (service-call-timeout must match switch-timeout):
    # --switch-timeout (above) raises the SERVER-side internal deadline
    # for confirming controller activation. But the spawner's ROS 2
    # SERVICE CLIENT has its own, separate --service-call-timeout
    # (default 10s) for how long it waits for a response to the
    # switch_controller call. If the server legitimately takes longer
    # than that to finish activating (as it now can, up to 60s), the
    # client gives up first with "Failed getting a result from calling
    # /controller_manager/switch_controller in 10.0", retries, and by
    # then the controller is in an inconsistent state -> abort. Both
    # timeouts must be raised together.

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "indomitus_arm_controller",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
            "--service-call-timeout", "70",
        ],
        output="screen",
    )

    delayed_controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller_spawner],
        )
    )

    # NOTE (move_group deferral):
    # move_group is a heavy CPU consumer at startup (it loads the pilz,
    # chomp, and ompl planning pipelines). Launching it in parallel with
    # the controller spawn/activation window competes for CPU with the
    # controller_manager's real-time update thread, which can make even a
    # trivial controller like joint_state_broadcaster miss the internal
    # (hardcoded, ~5s) switch_controller activation deadline -> "Switch
    # controller timed out after 5.000000 seconds!". To give the
    # activation handshake a clear run, move_group is now started only
    # after controller_spawner has exited (i.e. once both controllers are
    # confirmed active), instead of being started immediately at launch.

    moveit_config = MoveItConfigsBuilder(
        "indomitus_arm", package_name="arm_moveit_config"
    ).to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)

    delayed_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=list(move_group_launch.entities),
        )
    )

    ld = LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            delayed_controller_spawners,
            delayed_move_group,
        ]
    )

    return ld