"""
can_bridge.launch.py
====================
Brings up ros2_socketcan sender and receiver lifecycle nodes.

Launch arguments:
    interface            (default: can0)   — SocketCAN interface name
    sender_timeout_sec   (default: 0.01)   — sender TX timeout
    receiver_interval_sec (default: 0.01)  — receiver polling interval

Prerequisites (host-side, before launching):
    The can interface need to be activated and the bitrate need to be set.
    (Now it is done in the docker entrypoint)
"""

import json
import subprocess
from typing import List

import launch.logging
from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _validate_can_interface(context, *args, **kwargs) -> List[Action]:
    interface_name = LaunchConfiguration("interface").perform(context)
    logger = launch.logging.get_logger("can_validation")

    try:
        result = subprocess.run(
            ["ip", "-j", "link", "show", interface_name],
            capture_output=True, text=True, check=True
        )
        
        interface_data = json.loads(result.stdout)
        flags = interface_data[0].get("flags", [])
        
        if "UP" not in flags:
            logger.error(f"[CAN] ERROR: interface '{interface_name}' exists but is not UP.")
            return [Shutdown(reason=f"Interface {interface_name} is down")]

        logger.info(f"[CAN] Interface '{interface_name}' is UP — OK")
        return []

    except subprocess.CalledProcessError:
        logger.error(f"[CAN] ERROR: interface '{interface_name}' does not exist.")
        return [Shutdown(reason=f"Interface {interface_name} missing")]
        
    except (json.JSONDecodeError, IndexError, KeyError):
        logger.error(f"[CAN] ERROR: Failed to parse state for '{interface_name}'.")
        return [Shutdown(reason=f"Interface {interface_name} parse error")]


def _auto_configure(node: LifecycleNode, label: str) -> RegisterEventHandler:
    """Trigger CONFIGURE transition once the node OS process has started."""
    return RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                LogInfo(msg=f"[{label}] process started — sending configure..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

def _auto_activate(node: LifecycleNode, label: str) -> RegisterEventHandler:
    """Trigger ACTIVATE transition once the node reaches the inactive state."""
    return RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                LogInfo(msg=f"[{label}] inactive — sending activate..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

def _fail_fast(node: LifecycleNode, label: str) -> RegisterEventHandler:
    """Shut down the entire launch if a lifecycle node enters error state."""
    return RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="errorprocessing",
            entities=[
                LogInfo(msg=f"[{label}] entered error state — shutting down launch."),
                Shutdown(reason=f"[{label}] lifecycle error"),
            ],
        )
    )

def _launch_can_nodes(context, *args, **kwargs) -> List[Action]:
    interface_name = LaunchConfiguration("interface").perform(context)
    logger = launch.logging.get_logger("can_validation")

    # --- validating ---
    try:
        result = subprocess.run(
            ["ip", "-j", "link", "show", interface_name],
            capture_output=True, text=True, check=True
        )
        interface_data = json.loads(result.stdout)
        flags = interface_data[0].get("flags", [])
        if "UP" not in flags:
            logger.error(f"[CAN] ERROR: interface '{interface_name}' exists but is not UP.")
            raise RuntimeError(f"Interface {interface_name} is down")
        logger.info(f"[CAN] Interface '{interface_name}' is UP — OK")

    except subprocess.CalledProcessError:
        raise RuntimeError(f"[CAN] Interface '{interface_name}' does not exist.")
    except (json.JSONDecodeError, IndexError, KeyError):
        raise RuntimeError(f"[CAN] Failed to parse state for '{interface_name}'.")

    sender_node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_sender_node_exe",
        name="socket_can_sender",
        namespace="",
        parameters=[{
            "interface": LaunchConfiguration("interface"),
            "timeout_sec": LaunchConfiguration("sender_timeout_sec"),
        }],
        output="screen",
    )

    receiver_node = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        namespace="",
        parameters=[{
            "interface": LaunchConfiguration("interface"),
            "interval_sec": LaunchConfiguration("receiver_interval_sec"),
            "filters": LaunchConfiguration("receiver_filters"),
        }],
        arguments=["--ros-args", "--log-level", "socket_can_receiver:=WARN"],
        output="screen",
    )

    return [
        sender_node,
        receiver_node,
        _auto_configure(sender_node, "socket_can_sender"),
        _auto_configure(receiver_node, "socket_can_receiver"),
        _auto_activate(sender_node, "socket_can_sender"),
        _auto_activate(receiver_node, "socket_can_receiver"),
        _fail_fast(sender_node, "socket_can_sender"),
        _fail_fast(receiver_node, "socket_can_receiver"),
    ]

def generate_launch_description() -> LaunchDescription:
    interface_arg = DeclareLaunchArgument(
        "interface",
        default_value="can0",
        description="SocketCAN network interface name (e.g. can0, can1)",
    )
    sender_timeout_arg = DeclareLaunchArgument(
        "sender_timeout_sec",
        default_value="0.01",
        description="How long the sender waits for a message before retrying (seconds)",
    )
    receiver_interval_arg = DeclareLaunchArgument(
        "receiver_interval_sec",
        default_value="0.01",
        description="CAN socket polling interval for the receiver (seconds)",
    )

    receiver_filters_arg = DeclareLaunchArgument(
        "receiver_filters",
        default_value="300:700",
        description="candump-syntax CAN id:mask filter (hex, no 0x prefix)",
    )

    # sender_node = LifecycleNode(
    #     package="ros2_socketcan",
    #     executable="socket_can_sender_node_exe",
    #     name="socket_can_sender",
    #     namespace="",
    #     parameters=[{
    #         "interface": LaunchConfiguration("interface"),
    #         "timeout_sec": LaunchConfiguration("sender_timeout_sec"),
    #     }],
    #     output="screen",
    # )

    # receiver_node = LifecycleNode(
    #     package="ros2_socketcan",
    #     executable="socket_can_receiver_node_exe",
    #     name="socket_can_receiver",
    #     namespace="",
    #     parameters=[{
    #         "interface": LaunchConfiguration("interface"),
    #         "interval_sec": LaunchConfiguration("receiver_interval_sec"),
    #         # candump-syntax: id:mask (hex, without 0x).
    #         # Pass only 0x300-0x3FF (ESP),
    #         # all other ids are filtered out.
    #         # example: 'filters': '300:700,400:700',
    #         "filters": LaunchConfiguration("receiver_filters"),
    #     }],
    #     arguments=["--ros-args", "--log-level", "socket_can_receiver:=WARN"],
    #     output="screen",
    # )

    # sender_configure_handler = _auto_configure(sender_node, "socket_can_sender")
    # receiver_configure_handler = _auto_configure(receiver_node, "socket_can_receiver")
    # sender_activate_handler = _auto_activate(sender_node, "socket_can_sender")
    # receiver_activate_handler = _auto_activate(receiver_node, "socket_can_receiver")
    # sender_fail_fast_handler = _fail_fast(sender_node, "socket_can_sender")
    # receiver_fail_fast_handler = _fail_fast(receiver_node, "socket_can_receiver")

    return LaunchDescription([
        # Arguments
        interface_arg,
        sender_timeout_arg,
        receiver_interval_arg,
        receiver_filters_arg,

        # Validate CAN interface presence
        OpaqueFunction(function=_validate_can_interface),

        # # Event handlers
        # sender_configure_handler,
        # receiver_configure_handler,
        # sender_activate_handler,
        # receiver_activate_handler,
        # sender_fail_fast_handler,
        # receiver_fail_fast_handler,

        # # Nodes
        # sender_node,
        # receiver_node,

        OpaqueFunction(function=_launch_can_nodes),
    ])
