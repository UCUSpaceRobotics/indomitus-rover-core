#!/usr/bin/env python3
"""
Joystick Interpreter Node.

Reads joystick axes directly from /joy, applies swerve-aware wz correction
and vy toggle, then publishes to /cmd_vel.

Subscriptions:
    /joy              (sensor_msgs/Joy)       — raw joystick input

Publications:
    /cmd_vel          (geometry_msgs/Twist)   — processed output

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
    def __init__(self, client, name: str, logger, optimistic: bool = True):
        self._client = client
        self._name = name
        self._logger = logger
        self._state = False
        self._pending = False
        self._optimistic = optimistic

    def toggle(self):
        if self._pending or not self._client.service_is_ready():
            return
        target = not self._state
        if self._optimistic:
            self._state = target
        self._pending = True
        req = SetBool.Request()
        req.data = target
        self._client.call_async(req).add_done_callback(
            lambda f, t=target: self._on_result(f, t))

    def _on_result(self, future, target: bool):
        self._pending = False
        try:
            response = future.result()
            if not self._optimistic and response.success:
                self._state = target
            self._logger.info(f'{self._name} {"ON" if self._state else "OFF"}: {response.message}')
        except Exception as exc:
            self._logger.error(f'{self._name} service call failed: {exc!r}')


class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        # Motion parameters
        self.declare_parameter('axis_linear_x',    1)
        self.declare_parameter('axis_linear_y',    0)
        self.declare_parameter('axis_angular_yaw', 2)
        self.declare_parameter('scale_linear_x',   0.5)
        self.declare_parameter('scale_linear_y',   0.5)
        self.declare_parameter('scale_angular_yaw', 1.0)

        # Button parameters
        self.declare_parameter('vy_toggle_button',    4)
        self.declare_parameter('motor_toggle_button', 6)
        self.declare_parameter('compact_mode_button', 1)
        self.declare_parameter('vy_enabled_default',  False)

        # Lights
        self.declare_parameter('spotlight_button',      9)
        self.declare_parameter('beautiful_button',      10)
        self.declare_parameter('traffic_red_button',    11)
        self.declare_parameter('traffic_yellow_button', 12)
        self.declare_parameter('traffic_green_button',  13)
        self.declare_parameter('traffic_blue_button',   14)

        # Motion axes and scales
        self._axis_linear_x    = self.get_parameter('axis_linear_x').value
        self._axis_linear_y    = self.get_parameter('axis_linear_y').value
        self._axis_angular_yaw = self.get_parameter('axis_angular_yaw').value
        self._scale_linear_x   = self.get_parameter('scale_linear_x').value
        self._scale_linear_y   = self.get_parameter('scale_linear_y').value
        self._scale_angular_yaw = self.get_parameter('scale_angular_yaw').value

        # State
        self._vy_enabled: bool = self.get_parameter('vy_enabled_default').value

        # Button detectors
        self._vy_toggle_btn           = ButtonEdgeDetector(self.get_parameter('vy_toggle_button').value)
        self._motor_enabled_toggle_btn = ButtonEdgeDetector(self.get_parameter('motor_toggle_button').value)
        self._compact_mode_toggle_btn  = ButtonEdgeDetector(self.get_parameter('compact_mode_button').value)
        self._spotlight_btn = ButtonEdgeDetector(self.get_parameter('spotlight_button').value)
        self._beautiful_btn = ButtonEdgeDetector(self.get_parameter('beautiful_button').value)
        self._traffic_btns = {
            'red':    ButtonEdgeDetector(self.get_parameter('traffic_red_button').value),
            'yellow': ButtonEdgeDetector(self.get_parameter('traffic_yellow_button').value),
            'green':  ButtonEdgeDetector(self.get_parameter('traffic_green_button').value),
            'blue':   ButtonEdgeDetector(self.get_parameter('traffic_blue_button').value),
        }

        # Service callers
        self._motors_enabled = ToggleServiceCaller(
            self.create_client(SetBool, '/chassis/set_motors_enabled'),
            'motors', self.get_logger(), optimistic=False)
        self._compact_mode = ToggleServiceCaller(
            self.create_client(SetBool, '/set_compact_mode'),
            'compact_mode', self.get_logger(), optimistic=False)
        self._spotlight = ToggleServiceCaller(
            self.create_client(SetBool, '/lights/spotlight'),
            'spotlight', self.get_logger())
        self._beautiful = ToggleServiceCaller(
            self.create_client(SetBool, '/lights/beautiful'),
            'beautiful', self.get_logger())

        # Traffic light
        self._traffic_state   = {'red': False, 'yellow': False, 'green': False, 'blue': False}
        self._traffic_pending = False
        self._traffic_client  = self.create_client(SetTrafficLight, '/lights/traffic_light')

        # Pub/sub
        self._joy_sub = self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._vy_toggle_btn.index}, '
            f'motor_toggle_button={self._motor_enabled_toggle_btn.index}, '
            f'vy_enabled={self._vy_enabled}'
        )

    def _on_joy(self, msg: Joy):
        self._handle_buttons(msg.buttons)
        self._handle_motion(msg.axes)

    def _handle_buttons(self, buttons: list[int]):
        if self._vy_toggle_btn.is_pressed(buttons):
            self._vy_enabled = not self._vy_enabled
            self.get_logger().info(f'vy mode: {"ENABLED" if self._vy_enabled else "DISABLED"}')

        if self._motor_enabled_toggle_btn.is_pressed(buttons):
            self._motors_enabled.toggle()

        if self._compact_mode_toggle_btn.is_pressed(buttons):
            self._compact_mode.toggle()

        if self._spotlight_btn.is_pressed(buttons):
            self._spotlight.toggle()

        if self._beautiful_btn.is_pressed(buttons):
            self._beautiful.toggle()

        changed = False
        for color, btn in self._traffic_btns.items():
            if btn.is_pressed(buttons):
                self._traffic_state[color] = not self._traffic_state[color]
                changed = True
        if changed:
            self._send_traffic()

    def _handle_motion(self, axes: list[float]):
        vx = self._get_axis(axes, self._axis_linear_x)    * self._scale_linear_x
        vy = self._get_axis(axes, self._axis_linear_y)    * self._scale_linear_y
        wz = self._get_axis(axes, self._axis_angular_yaw) * self._scale_angular_yaw

        if not self._vy_enabled:
            vy = 0.0

        wz = self._apply_swerve_wz_correction(vx, vy, wz)

        out = Twist()
        out.linear.x  = vx
        out.linear.y  = vy
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)

    def _get_axis(self, axes: list[float], index: int) -> float:
        return axes[index] if 0 <= index < len(axes) else 0.0

    def _send_traffic(self):
        if self._traffic_pending or not self._traffic_client.service_is_ready():
            return
        self._traffic_pending = True
        req = SetTrafficLight.Request()
        req.red    = self._traffic_state['red']
        req.yellow = self._traffic_state['yellow']
        req.green  = self._traffic_state['green']
        req.blue   = self._traffic_state['blue']
        self._traffic_client.call_async(req).add_done_callback(self._on_traffic_result)

    def _on_traffic_result(self, future):
        self._traffic_pending = False
        try:
            response = future.result()
            self.get_logger().info(f'traffic_light: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'traffic_light service call failed: {exc!r}')

    def _apply_swerve_wz_correction(self, vx: float, vy: float, wz: float) -> float:
        """
        Invert wz when the rover is moving backward relative to its heading.

        Uses the dominant axis to decide direction so diagonal motion
        (e.g. vx=-0.1, vy=0.9) is handled intuitively.
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