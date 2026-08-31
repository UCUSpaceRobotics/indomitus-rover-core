#!/usr/bin/env python3
"""Gamepad control node for MoveIt Servo.

Reads sensor_msgs/Joy messages and sends Cartesian velocity commands to
MoveIt Servo. Core logic lives in ``servo_controller.ServoController`` —
this file is just the runnable entry point that wires it to
``gamepad_input.GamepadInputLoop``.

Gamepad controls (via ros2 joy game_controller_node, e.g. Stadia controller).
All gamepad translation is view-relative (arm_camera_link); rotation is
about arm_tcp_link, same as the keyboard's I/K/U/O/J/L:
    Left stick  left/right — EEF left / right (camera)
    Left stick  up/down    — EEF forward / back (camera)
    Right stick up/down    — EEF up / down (camera)
    Right stick left/right — yaw (TCP)
    R1 + right stick up/down    — pitch (TCP)
    R1 + right stick left/right — roll (TCP)
    9 (button)       — push boost (hold)
    A                — jaw/astrobio: move to home + start servo
                        drill_sampling: go to sampling_home. Also exits
                        drill mode back to sampling — but only from
                        drill_home.
    B                — jaw/astrobio: locked out
                        drill_sampling: go to drill_home. Enters drill mode
                        from sampling — but only from sampling_home. Once
                        already in drill mode, freely returns to drill_home
                        (e.g. from drill_container).
    Y                — jaw/astrobio: locked out
                        drill_sampling: context-dependent — from
                        sampling_home goes to sampling_container, from
                        drill_home goes to drill_container, otherwise
                        refused (needs sampling_home/A or drill_home/B first)
    11 / 13 (button) — gripper/claw/drill open / close (see GAMEPAD_HELP
                        in gamepad_input.py for the full per-tool mapping)
    12 / 14          — drill_sampling lock / unlock
    7 / 8            — align to detected panel / reorient gripper only
                        (jaw only — see GamepadInputLoop's own lockout)
    6 (button)       — point tool straight down (collision-checked)
    A/B/Y/7/8: 5s activity-indicator wait before moving — ERC 2026 Rules,
    Appendix 3, REQ-OPS-080/090/100 (see ServoController.run_planned_activity()).
    X                — exit

Usage:
    ros2 launch arm_bringup arm.launch.py use_fake_hardware:=false
    ros2 launch arm_teleop gamepad.launch.py

Requires a running ``joy``/``game_controller_node`` publisher — see
``arm_teleop/launch/gamepad.launch.py`` and ``gamepad_joy.launch.py``.
"""

import rclpy

from arm_teleop.servo_controller import ServoController, _run_teleop
from arm_teleop.gamepad_input import GamepadInputLoop


def main():
    """Entry point: initialize ROS2, run the gamepad input loop, and clean up.

    See ``_run_teleop`` (servo_controller.py) for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController(node_name='gamepad_teleop')
    _run_teleop(controller, GamepadInputLoop(controller))


if __name__ == '__main__':
    main()
