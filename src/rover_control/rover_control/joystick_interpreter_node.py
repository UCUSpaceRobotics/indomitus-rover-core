#!/usr/bin/env python3
"""
Joystick Interpreter Node.

Sits between teleop_twist_joy and the rest of the stack.
Subscribes to raw cmd_vel from teleop and /joy for button events,
applies swerve-aware wz inversion and vy toggle, then publishes to /cmd_vel.

Timeout/watchdog behavior:
    - The watchdog is based on /joy freshness (not /joy_raw_cmd_vel).
    - If no /joy message is received for longer than cmd_timeout,
      forwarding from /joy_raw_cmd_vel is blocked and zero Twist is published.
    - While timed out, zero Twist continues to be published at timeout_pub_rate.
    - On the next /joy message, timeout state is cleared and forwarding resumes.

Subscriptions:
    /joy_raw_cmd_vel  (geometry_msgs/Twist)  — raw output from teleop_twist_joy
    /joy              (sensor_msgs/Joy)       — raw joystick for button handling

Publications:
    /cmd_vel          (geometry_msgs/Twist)   — processed output

Parameters:
    vy_toggle_button      (int, default: 8)  — button index to toggle vy mode
    motor_toggle_button   (int, default: 9)  — button index to toggle chassis motors
    compact_mode_button   (int, default: 1)  — button index to toggle compact mode
    vy_enabled_default (bool, default: false) — initial state of vy mode
    cmd_timeout        (float, default: 0.5)  — /joy staleness threshold in seconds
    timeout_pub_rate   (float, default: 10.0) — zero-command publish/check rate while timed out (Hz)
    initial_timed_out  (bool, default: true)  — startup state; safe if true

Services used:
    /chassis/set_motors_enabled (std_srvs/SetBool) — explicit chassis motor enable/disable
    /set_compact_mode           (std_srvs/SetBool) — compact mode toggle
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from indomitus_interfaces.srv import SetTrafficLight

class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        self.declare_parameter('vy_toggle_button', 4)
        self.declare_parameter('motor_toggle_button', 6)
        self.declare_parameter('compact_mode_button', 1)
        self.declare_parameter('vy_enabled_default', False)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('timeout_pub_rate', 10.0)
        self.declare_parameter('initial_timed_out', True)

        # Lights
        self.declare_parameter('spotlight_button',      9)   # L1
        self.declare_parameter('beautiful_button',      10)  # R1

        self.declare_parameter('traffic_red_button',    11)  # ←
        self.declare_parameter('traffic_yellow_button', 12)  # →
        self.declare_parameter('traffic_green_button',  13)  # ↑
        self.declare_parameter('traffic_blue_button',   14)  # ↓

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

        # Lights
        self._traffic_red_button    = self.get_parameter('traffic_red_button').value
        self._traffic_yellow_button = self.get_parameter('traffic_yellow_button').value
        self._traffic_green_button  = self.get_parameter('traffic_green_button').value
        self._traffic_blue_button   = self.get_parameter('traffic_blue_button').value
        self._prev_traffic_red_button    = 0
        self._prev_traffic_yellow_button = 0
        self._prev_traffic_green_button  = 0
        self._prev_traffic_blue_button   = 0

        self._spotlight_on  = False
        self._beautiful_on  = False
        self._traffic_red   = False
        self._traffic_yellow = False
        self._traffic_green = False
        self._traffic_blue  = False

        self._spotlight_button = self.get_parameter('spotlight_button').value
        self._beautiful_button = self.get_parameter('beautiful_button').value
        self._prev_spotlight_button      = 0
        self._prev_beautiful_button      = 0

        self._spotlight_pending = False
        self._beautiful_pending = False
        self._traffic_pending   = False

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

        self._spotlight_client = self.create_client(SetBool, '/lights/spotlight')
        self._beautiful_client = self.create_client(SetBool, '/lights/beautiful')
        self._traffic_client   = self.create_client(SetTrafficLight, '/lights/traffic_light')

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._vy_toggle_button}, '
            f'motor_toggle_button={self._motor_toggle_button}, '
            f'vy_enabled={self._vy_enabled}'
        )

    def _on_joy(self, msg: Joy):
        """
        Detect button press edges for toggle actions and refresh watchdog timestamp.

        Any /joy message marks joystick input as alive. If we were in timed-out mode,
        this callback clears timeout and re-enables forwarding from /joy_raw_cmd_vel.
        """
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
        
        def _check_btn(buttons, idx, prev):
            cur = buttons[idx] if idx < len(buttons) else 0
            pressed = (cur == 1 and prev == 0)
            return cur, pressed

        cur, pressed = _check_btn(msg.buttons, self._spotlight_button, self._prev_spotlight_button)
        if pressed: self._toggle_spotlight()
        self._prev_spotlight_button = cur

        cur, pressed = _check_btn(msg.buttons, self._beautiful_button, self._prev_beautiful_button)
        if pressed: self._toggle_beautiful()
        self._prev_beautiful_button = cur
        
        cur, pressed = _check_btn(msg.buttons, self._traffic_red_button, self._prev_traffic_red_button)
        if pressed:
            self._traffic_red = not self._traffic_red
            self._send_traffic()
        self._prev_traffic_red_button = cur

        cur, pressed = _check_btn(msg.buttons, self._traffic_yellow_button, self._prev_traffic_yellow_button)
        if pressed:
            self._traffic_yellow = not self._traffic_yellow
            self._send_traffic()
        self._prev_traffic_yellow_button = cur

        cur, pressed = _check_btn(msg.buttons, self._traffic_green_button, self._prev_traffic_green_button)
        if pressed:
            self._traffic_green = not self._traffic_green
            self._send_traffic()
        self._prev_traffic_green_button = cur

        cur, pressed = _check_btn(msg.buttons, self._traffic_blue_button, self._prev_traffic_blue_button)
        if pressed:
            self._traffic_blue = not self._traffic_blue
            self._send_traffic()
        self._prev_traffic_blue_button = cur

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
        """Apply /joy freshness timeout and publish safe zero commands when stale."""
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
    
    def _toggle_spotlight(self):
        self.get_logger().info(f'DEBUG _toggle_spotlight called, pending={self._spotlight_pending}, ready={self._spotlight_client.service_is_ready()}')
        if self._spotlight_pending or not self._spotlight_client.service_is_ready():
            return
        self._spotlight_on = not self._spotlight_on
        self._spotlight_pending = True
        req = SetBool.Request()
        req.data = self._spotlight_on
        self._spotlight_client.call_async(req).add_done_callback(
            lambda f: self._on_light_result(f, 'spotlight', self._spotlight_on,
                                            '_spotlight_pending'))

    def _toggle_beautiful(self):
        self.get_logger().info(f'DEBUG _toggle_beautiful called, pending={self._beautiful_pending}, ready={self._beautiful_client.service_is_ready()}')
        if self._beautiful_pending or not self._beautiful_client.service_is_ready():
            return
        self._beautiful_on = not self._beautiful_on
        self._beautiful_pending = True
        req = SetBool.Request()
        req.data = self._beautiful_on
        self._beautiful_client.call_async(req).add_done_callback(
            lambda f: self._on_light_result(f, 'beautiful', self._beautiful_on,
                                            '_beautiful_pending'))

    def _send_traffic(self):
        self.get_logger().info(f'DEBUG _send_traffic called, pending={self._traffic_pending}, ready={self._traffic_client.service_is_ready()}, R={self._traffic_red} Y={self._traffic_yellow} G={self._traffic_green} B={self._traffic_blue}')
        if self._traffic_pending or not self._traffic_client.service_is_ready():
            return
        self._traffic_pending = True
        req = SetTrafficLight.Request()
        req.red    = self._traffic_red
        req.yellow = self._traffic_yellow
        req.green  = self._traffic_green
        req.blue   = self._traffic_blue
        self._traffic_client.call_async(req).add_done_callback(
            lambda f: self._on_traffic_result(f))

    def _on_light_result(self, future, name: str, desired: bool, pending_attr: str):
        setattr(self, pending_attr, False)
        try:
            response = future.result()
            state = 'ON' if desired else 'OFF'
            self.get_logger().info(f'{name} {state}: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'{name} service call failed: {exc!r}')

    def _on_traffic_result(self, future):
        self._traffic_pending = False
        try:
            response = future.result()
            self.get_logger().info(f'traffic_light: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'traffic_light service call failed: {exc!r}')

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