#!/usr/bin/env python3
"""
Joystick Interpreter Node.

Sits between teleop_twist_joy and the rest of the stack.
Subscribes to raw cmd_vel from teleop and /joy for button events,
applies swerve-aware wz inversion and vy toggle, then publishes to /cmd_vel.

Subscriptions:
    /joy_raw_cmd_vel  (geometry_msgs/Twist)  — raw output from teleop_twist_joy
    /joy              (sensor_msgs/Joy)       — raw joystick for button handling
    /chassis/set_motors_enabled (std_srvs/SetBool) — explicit chassis motor enable/disable

Publications:
    /cmd_vel          (geometry_msgs/Twist)   — processed output

Parameters:
    vy_toggle_button  (int, default: 4)  — button index to toggle vy mode (LB on most controllers)
    motor_toggle_button (int, default: 5) — button index to toggle chassis motors
    vy_enabled_default (bool, default: false) — initial state of vy mode
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        self.declare_parameter('vy_toggle_button', 4)
        self.declare_parameter('motor_toggle_button', 6)
        self.declare_parameter('vy_enabled_default', False)
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('timeout_pub_rate', 10.0)
        self.declare_parameter('initial_timed_out', True)
        self.declare_parameter('compact_mode_button', 1)

        self._vy_toggle_button: int = self.get_parameter('vy_toggle_button').value
        self._motor_toggle_button: int = self.get_parameter('motor_toggle_button').value
        self._vy_enabled: bool = self.get_parameter('vy_enabled_default').value
        self._motors_enabled: bool = False
        self._motor_toggle_pending: bool = False

        self._cmd_timeout: float = float(self.get_parameter('cmd_timeout').value)
        self._timeout_pub_rate: float = float(self.get_parameter('timeout_pub_rate').value)
        self._timed_out: bool = bool(self.get_parameter('initial_timed_out').value)

        self._last_joy_msg_time: float = 0.0

        self._compact_mode: bool = False
        self._prev_compact_button: int = 0
        self._compact_toggle_button: int = self.get_parameter('compact_mode_button').value

        # Track previous button state to detect press edge (not hold)
        self._prev_vy_button: int = 0
        self._prev_motor_button: int = 0

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
        self._motor_enable_client = self.create_client(
            SetBool,
            '/chassis/set_motors_enabled',
        )

        self._timeout_timer = self.create_timer(1.0 / max(0.001, self._timeout_pub_rate), self._timeout_check)
        self._compact_mode_client = self.create_client(SetBool, '/set_compact_mode')

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._vy_toggle_button}, '
            f'motor_toggle_button={self._motor_toggle_button}, '
            f'vy_enabled={self._vy_enabled}'
        )

    def _on_joy(self, msg: Joy):
        """Detect button press edges for toggle actions."""
        self._last_joy_msg_time = self._now_seconds()
        if self._timed_out:
            self._timed_out = False
            self.get_logger().info('Joystick input recovered — resuming command forwarding')

        if self._vy_toggle_button < len(msg.buttons):
            current = msg.buttons[self._vy_toggle_button]

            if current == 1 and self._prev_vy_button == 0:
                self._vy_enabled = not self._vy_enabled
                state_str = 'ENABLED' if self._vy_enabled else 'DISABLED'
                self.get_logger().info(f'vy mode: {state_str}')

            self._prev_vy_button = current

        if self._motor_toggle_button < len(msg.buttons):
            current = msg.buttons[self._motor_toggle_button]
            if current == 1 and self._prev_motor_button == 0:
                self._toggle_motors()
            self._prev_motor_button = current
        
        if self._compact_toggle_button < len(msg.buttons):
            current = msg.buttons[self._compact_toggle_button]
            if current == 1 and self._prev_compact_button == 0:
                self._toggle_compact_mode()
            self._prev_compact_button = current

    def _on_raw_cmd_vel(self, msg: Twist):
        self._last_twist_time = self._now_seconds()

        if self._timed_out:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if not self._vy_enabled:
            vy = 0.0

        wz = self._apply_swerve_wz_correction(vx, vy, wz)

        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)

    def _timeout_check(self):
        now = self._now_seconds()
        dt = now - self._last_joy_msg_time if self._last_joy_msg_time > 0.0 else float('inf')

        if dt > self._cmd_timeout:
            if not self._timed_out:
                self._timed_out = True
                self.get_logger().warn('Joystick input timed out — publishing zeros to /cmd_vel')
            
            self._publish_zero_cmd()
            
    def _publish_zero_cmd(self):
        out = Twist()
        out.linear.x = 0.0
        out.linear.y = 0.0
        out.angular.z = 0.0
        self._cmd_vel_pub.publish(out)

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _toggle_motors(self):
        if self._motor_toggle_pending:
            self.get_logger().warn('Motor toggle request is already in flight')
            return

        target_enabled = not self._motors_enabled

        if not self._motor_enable_client.service_is_ready():
            self.get_logger().warn('Motor enable service is not available yet')
            return

        self._motor_toggle_pending = True
        request = SetBool.Request()
        request.data = target_enabled
        future = self._motor_enable_client.call_async(request)
        future.add_done_callback(
            lambda completed_future, desired_state=target_enabled: self._on_motor_toggle_result(
                completed_future,
                desired_state,
            )
        )

    def _toggle_compact_mode(self):
        target = not self._compact_mode
        if not self._compact_mode_client.service_is_ready():
            self.get_logger().warn('/set_compact_mode service not available')
            return
        req = SetBool.Request()
        req.data = target
        future = self._compact_mode_client.call_async(req)
        future.add_done_callback(
            lambda f, desired=target: self._on_compact_mode_result(f, desired))

    def _on_compact_mode_result(self, future, desired: bool):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Compact mode service call failed: {exc!r}')
            return
        if response.success:
            self._compact_mode = desired
        self.get_logger().info(
            f'Compact mode {"ENABLED" if self._compact_mode else "DISABLED"}: {response.message}')

    def _on_motor_toggle_result(self, future, desired_state: bool):
        try:
            response = future.result()
        except Exception as exc:
            self._motor_toggle_pending = False
            self.get_logger().error(f'Motor enable service call failed: {exc!r}')
            return

        if response.success:
            self._motors_enabled = desired_state

        self._motor_toggle_pending = False

        status = 'ENABLED' if self._motors_enabled else 'DISABLED'
        self.get_logger().info(f'Motors {status}: {response.message}')

    def _apply_swerve_wz_correction(
        self, vx: float, vy: float, wz: float
    ) -> float:
        """
        Invert wz when the rover is moving 'backwaard' relative to its heading.

        Uses the dominant axis to decide direction so diagonal motion
        (e.g. vx=-0.1, vy=0.9) is handled intuitively — mostly sideways
        motion does not trigger inversion.
        """
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
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