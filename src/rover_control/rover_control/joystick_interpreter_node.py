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
    vy_toggle_button      (int, default: 8)  — button index to toggle vy mode
    motor_toggle_button   (int, default: 9)  — button index to toggle chassis motors
    compact_mode_button   (int, default: 1)  — button index to toggle compact mode
    vy_enabled_default (bool, default: false) — initial state of vy mode

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

class ButtonEdgeDetector:
    def __init__(self, index: int):
        self.index = index
        self._prev = 0
    
    def is_pressed(self, buttons: list[int]) -> bool:
        current = buttons[self.index] if 0 <= self.index < len(buttons) else 0
        pressed = current == 1 and self._prev == 0
        self._prev = current
        return pressed


class ToggleServiceCaller:
    def __init__(self, client, name: str, logger):
        self._client = client
        self._name = name
        self._logger = logger
        self._state = False
        self._pending = False

    def toggle(self):
        if self._pending or not self._client.service_is_ready():
            return
        self._state = not self._state
        self._pending = True
        req = SetBool.Request()
        req.data = self._state
        self._client.call_async(req).add_done_callback(self._on_result)

    def _on_result(self, future):
        self._pending = False
        try:
            response = future.result()
            self._logger.info(f'{self._name} {"ON" if self._state else "OFF"}: {response.message}')
        except Exception as exc:
            self._logger.error(f'{self._name} service call failed: {exc!r}')


class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        self.declare_parameter('vy_toggle_button', 4)
        self.declare_parameter('motor_toggle_button', 6)
        self.declare_parameter('compact_mode_button', 1)
        self.declare_parameter('vy_enabled_default', False)

        # Lights
        self.declare_parameter('spotlight_button',      9)   # L1
        self.declare_parameter('beautiful_button',      10)  # R1

        self.declare_parameter('traffic_red_button',    11)  # ←
        self.declare_parameter('traffic_yellow_button', 12)  # →
        self.declare_parameter('traffic_green_button',  13)  # ↑
        self.declare_parameter('traffic_blue_button',   14)  # ↓

        self._vy_toggle_btn    = ButtonEdgeDetector(self.get_parameter('vy_toggle_button').value)
        self._motor_toggle_btn = ButtonEdgeDetector(self.get_parameter('motor_toggle_button').value)
        self._vy_enabled: bool = self.get_parameter('vy_enabled_default').value
        self._motors_enabled: bool = False
        self._motor_toggle_pending: bool = False

        self._compact_mode: bool = False
        self._compact_toggle_btn = ButtonEdgeDetector(self.get_parameter('compact_mode_button').value)

        # Lights
        self._traffic_btns = {
            'red':    ButtonEdgeDetector(self.get_parameter('traffic_red_button').value),
            'yellow': ButtonEdgeDetector(self.get_parameter('traffic_yellow_button').value),
            'green':  ButtonEdgeDetector(self.get_parameter('traffic_green_button').value),
            'blue':   ButtonEdgeDetector(self.get_parameter('traffic_blue_button').value),
        }
        self._traffic_state = {'red': False, 'yellow': False, 'green': False, 'blue': False}

        # Sptolight and beafitul
        self._spotlight_btn = ButtonEdgeDetector(self.get_parameter('spotlight_button').value)
        self._beautiful_btn = ButtonEdgeDetector(self.get_parameter('beautiful_button').value)
        self._spotlight = ToggleServiceCaller(
            self.create_client(SetBool, '/lights/spotlight'), 'spotlight', self.get_logger())
        self._beautiful = ToggleServiceCaller(
            self.create_client(SetBool, '/lights/beautiful'), 'beautiful', self.get_logger())

        self._traffic_pending   = False

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

        self._compact_mode_client = self.create_client(SetBool, '/set_compact_mode')

        self._spotlight_client = self.create_client(SetBool, '/lights/spotlight')
        self._beautiful_client = self.create_client(SetBool, '/lights/beautiful')
        self._traffic_client   = self.create_client(SetTrafficLight, '/lights/traffic_light')

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._vy_toggle_btn.index}, '
            f'motor_toggle_button={self._motor_toggle_btn.index}, '
            f'vy_enabled={self._vy_enabled}'
        )

    def _on_joy(self, msg: Joy):
        """
        Process incoming joystick messages.
        """

        if self._vy_toggle_btn.is_pressed(msg.buttons):
            self._vy_enabled = not self._vy_enabled
            state_str = 'ENABLED' if self._vy_enabled else 'DISABLED'
            self.get_logger().info(f'vy mode: {state_str}')

        if self._motor_toggle_btn.is_pressed(msg.buttons):
            self._toggle_motors()
        
        if self._compact_toggle_btn.is_pressed(msg.buttons):
            self._toggle_compact_mode()

        if self._spotlight_btn.is_pressed(msg.buttons):
            self._spotlight.toggle()

        if self._beautiful_btn.is_pressed(msg.buttons):
            self._beautiful.toggle()
        
        changed = False
        for color, btn in self._traffic_btns.items():
            if btn.is_pressed(msg.buttons):
                self._traffic_state[color] = not self._traffic_state[color]
                changed = True
        if changed:
            self._send_traffic()

    def _on_raw_cmd_vel(self, msg: Twist):
        self._last_twist_time = self._now_seconds()

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

    def _send_traffic(self):
        self.get_logger().info(f'DEBUG _send_traffic called, pending={self._traffic_pending}, ready={self._traffic_client.service_is_ready()}, R={self._traffic_red} Y={self._traffic_yellow} G={self._traffic_green} B={self._traffic_blue}')
        if self._traffic_pending or not self._traffic_client.service_is_ready():
            return
        self._traffic_pending = True
        req = SetTrafficLight.Request()
        req.red    = self._traffic_state['red']
        req.yellow = self._traffic_state['yellow']
        req.green  = self._traffic_state['green']
        req.blue   = self._traffic_state['blue']
        self._traffic_client.call_async(req).add_done_callback(self._on_traffic_result)

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
        Invert wz when the rover is moving 'backward' relative to its heading.
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