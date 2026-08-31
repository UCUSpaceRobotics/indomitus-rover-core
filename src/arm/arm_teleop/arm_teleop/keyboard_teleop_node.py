#!/usr/bin/env python3
"""Keyboard control node for MoveIt Servo.

Reads keyboard input and sends Cartesian velocity commands to MoveIt Servo.
Core logic lives in ``servo_controller.ServoController`` — this file is
just the runnable entry point that wires it to ``keyboard_input.KeyboardInputLoop``.

Controls:
    EEF translation — absolute (in arm_mount_link):
        w / s  — +X / -X
        a / d  — +Y / -Y
        q / e  — +Z / -Z

    EEF translation — view-relative (in arm_camera_link, rigid with the
    gripper; axes are REP-103 so +X is where the camera and gripper point):
        Up / Down    — forward / back
        Left / Right — left / right
        t / g        — up / down

    Both translation sets are summed, so they can be held together.

    EEF rotation (about arm_tcp_link). Names are from the operator's point of
    view, i.e. the camera frame, which is what the TCP axes actually work out
    to: TCP +X is the camera's left-right axis (pitch), TCP +Y its vertical
    axis (yaw), TCP +Z its line of sight (roll):
        i / k  — pitch (+/- wx)
        u / o  — yaw   (+/- wy)
        j / l  — roll  (+/- wz)

    Gripper (commanded directly on gripper_right/left_controller, bypassing Servo):
        b / v  — open / close

    r      — move to home + start servo
    p      — align to detected panel (see panel_align_node); the first
             successful align each session is remembered, so later presses
             replay that exact position instead of re-planning
    f      — level tool (collision-checked; locks pitch/yaw after — 'r'
             unlocks)
    m      — rotate the gripper in place to face the remembered panel
             direction, current position kept (needs 'p' once first)
    ESC/x  — exit

    r/p/f/m first light the activity indicator and hold for 5s with the arm
    untouched before actually moving — ERC 2026 Rules, Appendix 3,
    REQ-OPS-080/090/100 (see ServoController.run_planned_activity()).

Usage (stack in one terminal, input in another):
        ros2 launch arm_bringup arm.launch.py use_fake_hardware:=false
        ros2 run arm_teleop keyboard_teleop_node --ros-args -r __ns:=/arm
        # optional: pin a device — ... -p keyboard_device_path:=/dev/input/event19

    Gazebo sim (sim clock + faster speeds, see arm_sim/config/keyboard_servo_sim.yaml):
        ros2 run arm_teleop keyboard_teleop_node --ros-args -r __ns:=/arm \\
            --params-file $(ros2 pkg prefix arm_sim)/share/arm_sim/config/keyboard_servo_sim.yaml

    Gamepad instead: see gamepad_teleop_node.py / ros2 launch arm_teleop gamepad.launch.py
"""

import rclpy

from arm_teleop.servo_controller import ServoController, _run_teleop
from arm_teleop.keyboard_input import KeyboardInputLoop


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    See ``_run_teleop`` (servo_controller.py) for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController(node_name='keyboard_teleop')
    _run_teleop(controller, KeyboardInputLoop(controller))


if __name__ == '__main__':
    main()
