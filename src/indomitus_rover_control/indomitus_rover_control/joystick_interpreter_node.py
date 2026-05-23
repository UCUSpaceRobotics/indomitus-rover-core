#!/usr/bin/env python3
"""
Joystick Interpreter Node.

Sits between teleop_twist_joy and the rest of the stack.
Subscribes to raw cmd_vel from teleop and /joy for button events,
applies swerve-aware wz inversion and vy toggle, then publishes to /cmd_vel.

Subscriptions:
    /joy_raw_cmd_vel  (geometry_msgs/Twist)  — raw output from teleop_twist_joy
    /joy              (sensor_msgs/Joy)       — raw joystick for button handling

Publications:
    /cmd_vel          (geometry_msgs/Twist)   — processed output

Parameters:
    vy_toggle_button  (int, default: 4)  — button index to toggle vy mode (LB on most controllers)
    vy_enabled_default (bool, default: false) — initial state of vy mode
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        self.declare_parameter('vy_toggle_button', 4)
        self.declare_parameter('vy_enabled_default', False)

        self._vy_toggle_button: int = self.get_parameter('vy_toggle_button').value
        self._vy_enabled: bool = self.get_parameter('vy_enabled_default').value

        # Track previous button state to detect press edge (not hold)
        self._prev_vy_button: int = 0

        self._raw_sub = self.create_subscription(
            Twist,
            '/joy_raw_cmd_vel',
            self._on_raw_cmd_vel,
            10,
        )
        self._joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self._on_joy,
            10,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._vy_toggle_button}, '
            f'vy_enabled={self._vy_enabled}'
        )

    # ── Button handling ────────────────────────────────────────────────────

    def _on_joy(self, msg: Joy):
        """Detect button press edges for toggle actions."""
        if self._vy_toggle_button >= len(msg.buttons):
            return

        current = msg.buttons[self._vy_toggle_button]

        # Rising edge only — toggle on press, not on hold
        if current == 1 and self._prev_vy_button == 0:
            self._vy_enabled = not self._vy_enabled
            state_str = 'ENABLED' if self._vy_enabled else 'DISABLED'
            self.get_logger().info(f'vy mode: {state_str}')

        self._prev_vy_button = current

    # ── Twist processing ───────────────────────────────────────────────────

    def _on_raw_cmd_vel(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 1. vy toggle
        if not self._vy_enabled:
            vy = 0.0

        # 2. Swerve-aware wz inversion:
        #    When moving backward (dominant axis determines "backward"),
        #    invert wz so steering feels intuitive from the operator's perspective.
        wz = self._apply_swerve_wz_correction(vx, vy, wz)

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)

    def _apply_swerve_wz_correction(
        self, vx: float, vy: float, wz: float
    ) -> float:
        """
        Invert wz when the rover is moving 'backward' relative to its heading.

        Uses the dominant axis to decide direction so diagonal motion
        (e.g. vx=-0.1, vy=0.9) is handled intuitively — mostly sideways
        motion does not trigger inversion.
        """
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            # Pure rotation — no correction needed
            return wz

        if abs(vx) >= abs(vy):
            if vx < 0.0:
                return -wz
        else:
            if vy < 0.0:
                return -wz

        return wz


def main(args=None):
    rclpy.init(args=args)
    node = JoystickInterpreterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()