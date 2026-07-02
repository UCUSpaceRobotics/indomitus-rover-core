#!/usr/bin/env python3
"""
Joystick Interpreter Node.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from indomitus_interfaces.srv import SetTrafficLight
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State


class ButtonToggle:
    """
    Rising-edge detector for a single joystick button.
    """

    def __init__(self, button_index: int, on_press):
        self.button_index = button_index
        self._on_press = on_press
        self._prev_state = 0

    def update(self, buttons) -> bool:
        """Feed the latest Joy.buttons array in; returns True if this was a press edge."""
        current = buttons[self.button_index] if self.button_index < len(buttons) else 0
        pressed = current == 1 and self._prev_state == 0
        self._prev_state = current
        if pressed:
            self._on_press()
        return pressed

    def reset(self):
        """Clear remembered state, e.g. after a timeout/disconnect."""
        self._prev_state = 0


class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        self.declare_parameter('vy_toggle_button', 8)
        self.declare_parameter('motor_toggle_button', 9)
        self.declare_parameter('compact_mode_button', 1)
        self.declare_parameter('vy_enabled_default', False)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('timeout_pub_rate', 10.0)
        self.declare_parameter('initial_timed_out', True)

        # Lights
        self.declare_parameter('spotlight_button',      4)   # L1
        self.declare_parameter('beautiful_button',      5)  # R1

        self.declare_parameter('traffic_red_button',    11)  # ←
        self.declare_parameter('traffic_yellow_button', 12)  # →
        self.declare_parameter('traffic_green_button',  13)  # ↑
        self.declare_parameter('traffic_blue_button',   14)  # ↓

        self.declare_parameter('granny_button', 10)
        self.declare_parameter('granny_speed_scale', 0.1)

        self._granny_scale: float = float(self.get_parameter('granny_speed_scale').value)
        self._granny_mode: bool = False

        self._vy_enabled: bool = self.get_parameter('vy_enabled_default').value
        self._motors_enabled: bool = False
        self._motor_toggle_pending: bool = False

        self._cmd_timeout: float = float(self.get_parameter('cmd_timeout').value)
        self._timeout_pub_rate: float = float(self.get_parameter('timeout_pub_rate').value)
        self._timed_out: bool = bool(self.get_parameter('initial_timed_out').value)

        self._last_joy_msg_time: float = 0.0

        self._compact_mode: bool = False

        # Lights
        self._spotlight_on  = False
        self._beautiful_on  = False
        self._traffic_red   = False
        self._traffic_yellow = False
        self._traffic_green = False
        self._traffic_blue  = False

        self._spotlight_pending = False
        self._beautiful_pending = False
        self._traffic_pending   = False

        self._toggles = [
            ButtonToggle(self.get_parameter('vy_toggle_button').value, self._on_vy_toggle_pressed),
            ButtonToggle(self.get_parameter('motor_toggle_button').value, self._toggle_motors),
            ButtonToggle(self.get_parameter('compact_mode_button').value, self._toggle_compact_mode),
            ButtonToggle(self.get_parameter('granny_button').value, self._on_granny_toggle_pressed),
            ButtonToggle(self.get_parameter('spotlight_button').value, self._toggle_spotlight),
            ButtonToggle(self.get_parameter('beautiful_button').value, self._toggle_beautiful),
            ButtonToggle(self.get_parameter('traffic_red_button').value, lambda: self._toggle_traffic('red')),
            ButtonToggle(self.get_parameter('traffic_yellow_button').value, lambda: self._toggle_traffic('yellow')),
            ButtonToggle(self.get_parameter('traffic_green_button').value, lambda: self._toggle_traffic('green')),
            ButtonToggle(self.get_parameter('traffic_blue_button').value, lambda: self._toggle_traffic('blue')),
        ]

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
            SetHardwareComponentState,
            '/controller_manager/set_hardware_component_state',
        )

        self._controller_state_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
        )

        self._timeout_timer = self.create_timer(1.0 / max(0.001, self._timeout_pub_rate), self._timeout_check)
        self._compact_mode_client = self.create_client(SetBool, '/swerve_controller/set_compact_mode')

        self._spotlight_client = self.create_client(SetBool, '/lights/spotlight')
        self._beautiful_client = self.create_client(SetBool, '/lights/beautiful')
        self._traffic_client   = self.create_client(SetTrafficLight, '/lights/traffic_light')

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._toggles[0].button_index}, '
            f'motor_toggle_button={self._toggles[1].button_index}, '
            f'vy_enabled={self._vy_enabled}'
        )

    def _on_joy(self, msg: Joy):
        """
        Refresh the watchdog timestamp and run every button's edge detector.
        """
        self._last_joy_msg_time = self._now_seconds()
        if self._timed_out:
            self._timed_out = False
            self.get_logger().info('Joystick input recovered — resuming command forwarding')

        for toggle in self._toggles:
            toggle.update(msg.buttons)

    def _on_vy_toggle_pressed(self):
        self._vy_enabled = not self._vy_enabled
        state_str = 'ENABLED' if self._vy_enabled else 'DISABLED'
        self.get_logger().info(f'vy mode: {state_str}')

    def _on_granny_toggle_pressed(self):
        self._granny_mode = not self._granny_mode

    def _toggle_traffic(self, color: str):
        attr = f'_traffic_{color}'
        setattr(self, attr, not getattr(self, attr))
        self._send_traffic()

    def _on_raw_cmd_vel(self, msg: Twist):
        self._last_twist_time = self._now_seconds()

        if self._timed_out:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        if not self._vy_enabled:
            vy = 0.0

        if not self._vy_enabled:
            wz = self._apply_swerve_wz_correction(vx, vy, wz)

        if self._granny_mode:
            vx *= self._granny_scale
            vy *= self._granny_scale
            wz *= self._granny_scale

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

        if not self._motor_enable_client.service_is_ready():
            self.get_logger().warn('Motor enable service is not available yet')
            return

        target_enabled = not self._motors_enabled
        self._motor_toggle_pending = True

        request = SetHardwareComponentState.Request()
        request.name = 'RoverHardware'
        request.target_state.id = (
            State.PRIMARY_STATE_ACTIVE if target_enabled else State.PRIMARY_STATE_INACTIVE
        )

        future = self._motor_enable_client.call_async(request)
        future.add_done_callback(
            lambda f, desired=target_enabled: self._on_motor_toggle_result(f, desired)
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
        self._motor_toggle_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Motor toggle failed: {exc!r}')
            return

        if response.ok:
            self._motors_enabled = desired_state
            if desired_state:
                self._set_swerve_controller_state(True)
            else:
                self._set_swerve_controller_state(False)

        status = 'ENABLED' if self._motors_enabled else 'DISABLED'
        self.get_logger().info(f'Motors {status}')

    def _set_swerve_controller_state(self, activate: bool):
        if not self._controller_state_client.service_is_ready():
            self.get_logger().warn('switch_controller service not available')
            return
        req = SwitchController.Request()
        if activate:
            req.activate_controllers = ['swerve_controller']
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = ['swerve_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        self._controller_state_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'swerve_controller → {"active" if activate else "inactive"}'))

    def _toggle_spotlight(self):
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
        Invert wz when the rover is moving 'backward' relative to its heading.
        """
        if vx < -1e-3:
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


if __name__ == '__main__':
    main()
