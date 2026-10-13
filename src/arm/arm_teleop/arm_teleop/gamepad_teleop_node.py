#!/usr/bin/env python3
"""Gamepad control node for MoveIt Servo (gamepad_servo_node): /joy ->
GamepadInputLoop -> ServoController. Controls: see keyboard_teleop_node.py
and gamepad_input.GAMEPAD_HELP."""

from arm_teleop.gamepad_input import GamepadInputLoop
from arm_teleop.servo_controller import ServoController, init_ros, run_teleop


def main():
    """Entry point: initialize ROS2, run the gamepad input loop, and clean up.

    Requires a running ``joy`` publisher — see ``arm_teleop/launch/gamepad_joy.launch.py``.
    This node itself comes from ``gamepad_servo.launch.py`` (included by ``arm.launch.py``).
    See ``servo_controller.run_teleop`` for the shared spin/cleanup lifecycle.
    """
    init_ros()
    controller = ServoController()
    run_teleop(controller, GamepadInputLoop(controller))



if __name__ == '__main__':
    main()
