#!/usr/bin/env python3
"""
Joystick Interpreter Node.
"""
import math

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


class GuardedCall:
    def __init__(self, client):
        self._client = client
        self.pending = False

    def call(self, request, on_done) -> bool:
        if self.pending or not self._client.service_is_ready():
            return False

        self.pending = True

        def _wrapped(future):
            self.pending = False
            on_done(future)

        self._client.call_async(request).add_done_callback(_wrapped)
        return True


class BoolLight:
    def __init__(self, node: Node, client, name: str):
        self._node = node
        self._guard = GuardedCall(client)
        self.name = name
        self.on = False

    def toggle(self):
        desired = not self.on
        req = SetBool.Request()
        req.data = desired
        if not self._guard.call(req, lambda f: self._on_result(f, desired)):
            self._node.get_logger().warn(f'{self.name} service busy or not available')

    def _on_result(self, future, desired: bool):
        try:
            response = future.result()
        except Exception as exc:
            self._node.get_logger().error(f'{self.name} service call failed: {exc!r}')
            return
        
        if response.success:
            self.on = desired

        state = 'ON' if desired else 'OFF'
        self._node.get_logger().info(f'{self.name} {state}: {response.message}')


class JoystickInterpreterNode(Node):

    def __init__(self):
        super().__init__('joystick_interpreter')

        def declare_and_get(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self._axis_vx  =   int(declare_and_get('axis_linear.x', 1))
        self._axis_vy  =   int(declare_and_get('axis_linear.y', 0))
        self._axis_wz  =   int(declare_and_get('axis_angular.yaw', 2))
        self._scale_vx = float(declare_and_get('scale_linear.x', 0.5))
        self._scale_vy = float(declare_and_get('scale_linear.y', 0.5))
        self._scale_wz = float(declare_and_get('scale_angular.yaw', 1.0))

        # L2 / R2 — spin-in-place while in curvature mode.
        self._axis_l2 = int(declare_and_get('axis_trigger.l2', 4))
        self._axis_r2 = int(declare_and_get('axis_trigger.r2', 5))
        self._trigger_deadzone = float(declare_and_get('trigger_deadzone', 0.15))
        self._scale_rotate     = float(declare_and_get('scale_rotate', 1.0))

        # Tightest turn the right stick can ask for, as curvature 1/R at full
        # deflection. 2.0 means R = 0.5 m, an ICR inside the wheelbase.
        self._max_curvature = float(declare_and_get('max_curvature', 2.0))

        # Token speed used to command a wheel angle while standing still —
        # see the RIDING branch of _publish_timer_cb.
        self._angle_probe_speed = float(declare_and_get('angle_probe_speed', 1e-5))

        # Which swerve controller this joystick drives. Switching to
        # 'swerve_controller_test' also repoints the compact-mode service.
        self._controller_name = str(declare_and_get('controller_name', 'swerve_controller'))

        self._granny_scale = float(declare_and_get('granny_speed_scale', 0.1))
        self._granny_mode = False

        self._vy_enabled = bool(declare_and_get('vy_enabled_default', False))
        self._motors_enabled = False

        self._cmd_timeout = float(declare_and_get('cmd_timeout', 0.5))
        self._timeout_pub_rate = float(declare_and_get('timeout_pub_rate', 10.0))
        self._timed_out = bool(declare_and_get('initial_timed_out', True))

        self._cmd_pub_rate = float(declare_and_get('cmd_pub_rate', 20.0))
        self._active       =  bool(declare_and_get('active_default', True))

        self._last_joy_msg_time: float = 0.0

        self.raw_vx: float = 0.0
        self.raw_vy: float = 0.0
        self.raw_wz: float = 0.0
        self.raw_steer: float = 0.0
        self.raw_rot: float = 0.0

        self._row_twist_mode: bool = True
        self._compact_mode: bool = False

        self._traffic_red    = False
        self._traffic_yellow = False
        self._traffic_green  = False
        self._traffic_blue   = False

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
        self._publish_timer = self.create_timer(1.0 / max(0.001, self._cmd_pub_rate), self._publish_timer_cb)
        self._compact_mode_client = self.create_client(
            SetBool, f'/{self._controller_name}/set_compact_mode')

        self._spotlight = BoolLight(self, self.create_client(SetBool, '/lights/spotlight'), 'spotlight')
        self._beautiful = BoolLight(self, self.create_client(SetBool, '/lights/beautiful'), 'beautiful')
        self._traffic_client = self.create_client(SetTrafficLight, '/lights/traffic_light')

        self._motor_guard = GuardedCall(self._motor_enable_client)
        self._compact_mode_guard = GuardedCall(self._compact_mode_client)
        self._traffic_guard = GuardedCall(self._traffic_client)

        self._toggles = [
            ButtonToggle(declare_and_get('vy_toggle_button', 8), self._on_vy_toggle_pressed),
            ButtonToggle(declare_and_get('motor_toggle_button', 9), self._toggle_motors),
            ButtonToggle(declare_and_get('raw_twist_mode_button', 3), self._on_raw_twist_mode_toggle_pressed),
            ButtonToggle(declare_and_get('compact_mode_button', 1), self._toggle_compact_mode),
            ButtonToggle(declare_and_get('granny_button', 10), self._on_granny_toggle_pressed),
            ButtonToggle(declare_and_get('spotlight_button', 4), self._spotlight.toggle),
            ButtonToggle(declare_and_get('beautiful_button', 5), self._beautiful.toggle),
            ButtonToggle(declare_and_get('traffic_red_button',   11), lambda: self._toggle_traffic('red')),
            ButtonToggle(declare_and_get('traffic_yellow_button', 12), lambda: self._toggle_traffic('yellow')),
            ButtonToggle(declare_and_get('traffic_green_button',  13), lambda: self._toggle_traffic('green')),
            ButtonToggle(declare_and_get('traffic_blue_button',   14), lambda: self._toggle_traffic('blue')),
            ButtonToggle(declare_and_get('active_toggle_button', 2), self._on_active_toggle_pressed),
        ]

        self.get_logger().info(
            f'JoystickInterpreter started — '
            f'vy_toggle_button={self._toggles[0].button_index}, '
            f'motor_toggle_button={self._toggles[1].button_index}, '
            f'vy_enabled={self._vy_enabled}, '
            f'controller={self._controller_name}'
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
        
        self.raw_vx = msg.axes[self._axis_vx] * self._scale_vx
        self.raw_vy = msg.axes[self._axis_vy] * self._scale_vy if self._vy_enabled else 0.0
        self.raw_wz = msg.axes[self._axis_wz] * self._scale_wz
        # Kept unscaled: in curvature mode this stick sets a radius, not a
        # yaw rate, so scale_angular does not apply to it.
        self.raw_steer = msg.axes[self._axis_wz]
        self.raw_rot = self._trigger_diff(msg.axes)

    def _trigger_diff(self, axes) -> float:
        """L2 minus R2, deadzoned. Positive = L2 = counter-clockwise spin.

        Both triggers rest at 0.0 and reach 1.0 fully pressed. Reading the
        *difference* rather than either trigger alone also means a driver that
        rests them somewhere else can't cause trouble: a shared rest offset
        cancels out, so a wrong rest value can never produce a standing spin
        command — only a halved or mirrored response.
        """
        def value(index: int) -> float:
            return axes[index] if index < len(axes) else 0.0

        diff = value(self._axis_r2) - value(self._axis_l2)
        return 0.0 if abs(diff) < self._trigger_deadzone else diff

    def _publish_timer_cb(self):
        """Publish the latest known command at a fixed rate (default 20 Hz),
        regardless of how often /joy actually fires. Suppressed while timed out —
        _timeout_check takes over publishing zeros in that case."""
        if self._timed_out or not self._active:
            return

        vx = self.raw_vx
        vy = self.raw_vy

        if self._row_twist_mode:
            wz = self.raw_wz
            if not self._vy_enabled:
                # Raw mode only. Same intent as v_signed in the RIDING branch
                # below — keep the turn centre on the same side of the rover
                # when reversing — but done as a correction after the fact,
                # because here wz comes straight off the stick and was never
                # derived from a speed. No controller can infer this for us:
                # by the time the Twist exists the driver's intent is gone.
                wz = self._apply_swerve_wz_correction(vx, vy, wz)

        elif self.raw_rot != 0.0:
            # ROTATING — a trigger is held, so the left stick is ignored and
            # vx/vy are driven to *exactly* zero. That is what places the twist
            # on the spin-in-place pole; a leftover 0.01 of stick would ask for
            # a very tight turn instead of a spin.
            vx = 0.0
            vy = 0.0
            wz = self.raw_rot * self._scale_rotate

        else:
            # RIDING — right stick sets the turn radius and only the radius.
            target_curvature = self.raw_steer * self._max_curvature

            # Signed, not hypot(): the yaw rate has to reverse along with the
            # direction of travel. hypot() is always positive, so reversing
            # would keep wz pointing the same way and swing the turn centre
            # across to the other side of the rover — the wheels mirror and the
            # radius appears to flip sign. A car does not do that: hold the
            # wheel still, back up, and the steering stays exactly where it is.
            v_total  = math.hypot(vx, vy)
            v_signed = -v_total if vx < 0.0 else v_total

            if v_total < 1e-3 and target_curvature != 0.0:
                # Standing still with a radius dialled in. Send a token speed
                # so the twist still carries a *direction*:
                # swerve_controller_test normalises the twist, recovering the
                # same wheel geometry at any scale, and holds the drives at
                # zero below park_speed. Net effect is the wheels steer to the
                # commanded radius with the rover staying put.
                vx = self._angle_probe_speed
                vy = 0.0
                wz = vx * target_curvature
            else:
                wz = v_signed * target_curvature

        if self._granny_mode:
            vx *= self._granny_scale
            vy *= self._granny_scale
            wz *= self._granny_scale

        self._publish_cmd(vx, vy, wz)

    def _on_vy_toggle_pressed(self):
        self._vy_enabled = not self._vy_enabled
        state_str = 'ENABLED' if self._vy_enabled else 'DISABLED'
        self.get_logger().info(f'vy mode: {state_str}')

    def _on_granny_toggle_pressed(self):
        self._granny_mode = not self._granny_mode
        state_str = 'ENABLED' if self._granny_mode else 'DISABLED'
        self.get_logger().info(f'granny mode: {state_str}')

    def _toggle_traffic(self, color: str):
        desired = not getattr(self, f'_traffic_{color}')

        req = SetTrafficLight.Request()
        req.red = self._traffic_red
        req.yellow = self._traffic_yellow
        req.green = self._traffic_green
        req.blue = self._traffic_blue

        setattr(req, color, desired)

        started = self._traffic_guard.call(
            req,
            lambda f: self._on_traffic_result(f, color, desired),
        )

        if not started:
            self.get_logger().warn('traffic_light service busy or not available')

    def _timeout_check(self):
        """Apply /joy freshness timeout and publish safe zero commands when stale."""
        if not self._active:
            return

        now = self._now_seconds()
        dt = now - self._last_joy_msg_time if self._last_joy_msg_time > 0.0 else float('inf')

        if dt > self._cmd_timeout:
            if not self._timed_out:
                self._timed_out = True
                self.get_logger().warn('Joystick input timed out — publishing zeros to /cmd_vel')

            self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_cmd(self, vx: float, vy: float, wz: float):
        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)

    def _on_active_toggle_pressed(self):
        self._active = not self._active
        state_str = 'ACTIVE (publishing to /cmd_vel)' if self._active else 'INACTIVE (yielding to nav)'
        self.get_logger().info(f'Joystick control: {state_str}')

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _toggle_motors(self):
        target_enabled = not self._motors_enabled

        request = SetHardwareComponentState.Request()
        request.name = 'RoverHardware'
        request.target_state.id = (
            State.PRIMARY_STATE_ACTIVE if target_enabled else State.PRIMARY_STATE_INACTIVE
        )

        started = self._motor_guard.call(
            request, lambda f: self._on_motor_toggle_result(f, target_enabled))
        if not started:
            self.get_logger().warn('Motor enable service busy or not available yet')
    
    def _on_raw_twist_mode_toggle_pressed(self):
        self._row_twist_mode = not self._row_twist_mode
        state_str = 'RAW TWIST (Direct)' if self._row_twist_mode else 'PROCESSED (Curvature)'
        self.get_logger().info(f'Switching to {state_str} mode')

    def _toggle_compact_mode(self):
        target = not self._compact_mode
        req = SetBool.Request()
        req.data = target
        started = self._compact_mode_guard.call(
            req, lambda f: self._on_compact_mode_result(f, target))
        if not started:
            self.get_logger().warn('/set_compact_mode service busy or not available')

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
            req.activate_controllers = [self._controller_name]
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = [self._controller_name]
        req.strictness = SwitchController.Request.BEST_EFFORT
        self._controller_state_client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'{self._controller_name} → {"active" if activate else "inactive"}'))

    def _on_traffic_result(self, future, color: str, desired: bool):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'traffic_light service call failed: {exc!r}')
            return

        if response.success:
            setattr(self, f'_traffic_{color}', desired)

        self.get_logger().info(f'traffic_light: {response.message}')

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
