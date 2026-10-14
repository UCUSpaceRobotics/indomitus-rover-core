#!/usr/bin/env python3
"""
Keyboard control node for MoveIt Servo.

Reads keyboard input and sends Cartesian velocity commands to MoveIt Servo.
Designed to be modular — core logic is in ServoController class,
so switching to gamepad (joy package) requires only replacing the input source.

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

    Gripper (bypassing Servo):
        b / v  — open / close: one 'open'/'close' to the real tool on press
                 (end_effector_controller/command -> CAN), and while held the
                 URDF fingers move on gripper_right/left_controller (sim, RViz)

    r      — move to home + start servo
    p      — align to detected panel (see panel_align_node); the first
             successful align each session is remembered, so later presses
             replay that exact position instead of re-planning
    f      — level tool (collision-checked; locks pitch/yaw after — 'r'
             unlocks)
    n      — unstow: slowly lift shoulder/elbow off the body/base (no
             planner; only lifts), then resume Servo. 'r' does this
             automatically when the arm starts in collision.
    m      — rotate the gripper in place to face the remembered panel
             direction, current position kept (needs 'p' once first)
    c      — activate: turn the arm motors on (the real arm starts with
             them off: positions read, no torque), stiffness ramps up
    z      — park: lay the arm on its rest support (poses.json "park"),
             motors still on
    y      — release: from the park pose only, ramp the stiffness down
             and turn the motors off (refused, stiffness restored, if the
             arm moves meanwhile)
    ESC/x  — exit

    r/p/f/m (and their gamepad equivalents, see below) first light the
    activity indicator and hold for 5s with the arm untouched before
    actually moving — ERC 2026 Rules, Appendix 3, REQ-OPS-080/090/100 (see
    ServoController.run_planned_activity()).

Gamepad controls (via ros2 joy joy_node, e.g. Stadia controller).
All gamepad translation is view-relative (arm_camera_link); rotation is
about arm_tcp_link, same as the keyboard's I/K/U/O/J/L:
    Left stick  left/right — EEF left / right (camera)
    Left stick  up/down    — EEF forward / back (camera)
    Right stick up/down    — EEF up / down (camera)
    Right stick left/right — yaw (TCP)
    R1 + right stick up/down    — pitch (TCP)
    R1 + right stick left/right — roll (TCP)
    A                — move to home + start servo
    Button 7         — align to detected panel (see keyboard 'p' above)
    Button 8         — reorient gripper only (see keyboard 'm' above)
    Button 4 (BACK)  — unstow (see keyboard 'n' above)
    R1 + A / BACK / X — activate / park / release (keyboard c / z / y)
    X                — exit

Usage (stack in one terminal, input in another):
        ros2 launch arm_bringup arm.launch.py use_fake_hardware:=false
        ros2 launch arm_teleop keyboard.launch.py
        # optional: pin a device — ros2 run arm_teleop keyboard_servo_node --ros-args
        #   -r __ns:=/arm -p keyboard_device_path:=/dev/input/event19

    Gazebo sim (sim clock):
        ros2 launch arm_teleop keyboard.launch.py use_sim_time:=true

    Gamepad instead (arm.launch.py already starts gamepad_servo_node):
        ros2 launch arm_teleop gamepad_joy.launch.py      # where the gamepad is plugged in
        ros2 launch arm_teleop gamepad_servo.launch.py    # only if the stack did not start it (sims)
"""


from arm_teleop.keyboard_input import KeyboardInputLoop
from arm_teleop.servo_controller import ServoController, init_ros, run_teleop


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    See ``servo_controller.run_teleop`` for the shared spin/cleanup lifecycle.
    """
    init_ros()
    controller = ServoController()
    run_teleop(controller, KeyboardInputLoop(controller))




if __name__ == '__main__':
    main()
