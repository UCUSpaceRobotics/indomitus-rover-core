#!/usr/bin/env python3
"""
Joystick Interpreter Node.

Turns a DualSense into a twist on cmd_vel_joy, and its buttons into requests
somebody else acts on. What this node owns is exactly what shapes its own
output — strafe, granny, curvature vs raw twist, and whether it is publishing
at all. Everything that is hardware state is owned elsewhere and read back:
drive_manager_node holds motors and the controller, lights_can_node holds the
lights. That split is what lets the ground station drive the same rover
without the two operators' idea of the state drifting apart.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from indomitus_interfaces.msg import DriveState

from rover_teleop.controller_led import LED_NODE_DOWN, ControllerLed, led_colour
from rover_teleop.drive_kinematics import (
    DriveModes,
    JoyInput,
    KinematicsParams,
    twist_from_input,
)
from rover_teleop.joy_input import (
    DISABLED,
    ButtonToggle,
    apply_deadzone,
    shared_buttons,
    trigger_diff,
    triggers_held,
)
from rover_teleop.service_call import GuardedCall
from rover_teleop.teleop_state import JoyWatchdog


LED_REPAINT_PERIOD = 0.5

#: Matches the publishers on drive/state and teleop/joystick_active: latched,
#: so state that changed before this node started is still delivered.
STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class GamepadInterpreterNode(Node):

    def __init__(self, node_name: str = 'gamepad_interpreter', **node_kwargs):
        # node_name / node_kwargs: keyboard_teleop_node runs a second instance
        # in its own process, with its own name, parameters and remappings.
        super().__init__(node_name, **node_kwargs)

        def declare_and_get(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self._axis_vx  =   int(declare_and_get('axis_linear.x', 1))
        self._axis_vy  =   int(declare_and_get('axis_linear.y', 0))
        self._axis_wz  =   int(declare_and_get('axis_angular.yaw', 2))
        self._scale_vx = float(declare_and_get('scale_linear.x', 0.5))
        self._scale_vy = float(declare_and_get('scale_linear.y', 0.5))
        self._scale_wz = float(declare_and_get('scale_angular.yaw', 1.0))

        # Stick deadzone, applied here instead of in the driver — see
        # joy_input.apply_deadzone(). Clamped below 1.0 so the rescaling never
        # divides by zero.
        self._deadzone = min(max(float(declare_and_get('deadzone', 0.05)), 0.0), 0.99)

        # L2 / R2 — spin-in-place while in curvature mode.
        self._axis_l2 = int(declare_and_get('axis_trigger.l2', 4))
        self._axis_r2 = int(declare_and_get('axis_trigger.r2', 5))
        self._trigger_deadzone = float(declare_and_get('trigger_deadzone', 0.15))

        self._params = KinematicsParams(
            scale_rotate=float(declare_and_get('scale_rotate', 1.0)),
            rot_probe_wz=float(declare_and_get('rot_probe_wz', 1e-5)),
            max_curvature=float(declare_and_get('max_curvature', 2.0)),
            angle_probe_speed=float(declare_and_get('angle_probe_speed', 1e-5)),
            granny_scale=float(declare_and_get('granny_speed_scale', 0.1)),
        )

        # --- local state: everything here only shapes this node's own output.
        self._granny_mode = False
        self._vy_enabled = bool(declare_and_get('vy_enabled_default', False))
        self._raw_twist_mode = True
        self._active = bool(declare_and_get('active_default', True))

        # --- drive state, owned by drive_manager_node and read back for the LED.
        self._drive_state = DriveState()

        self._input = JoyInput()

        self._timeout_pub_rate = float(declare_and_get('timeout_pub_rate', 10.0))
        self._watchdog = JoyWatchdog(
            timeout=float(declare_and_get('cmd_timeout', 0.5)),
            timed_out=bool(declare_and_get('initial_timed_out', True)),
            # A stale /joy must stop the wheels and then let go of the topic:
            # cmd_vel_joy outranks every other source in twist_mux, so holding
            # it with zeros would lock the ground station out of a rover with
            # nobody at the gamepad. See JoyWatchdog.
            zero_burst=int(declare_and_get('timeout_zero_burst', 3)),
        )

        self._cmd_pub_rate = float(declare_and_get('cmd_pub_rate', 20.0))

        self._joy_sub = self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # The ground station operator's answer to "why is the rover ignoring
        # my sticks?" — this node outranks them in twist_mux whenever it is
        # publishing. Latched so it is readable without waiting for a change.
        self._active_pub = self.create_publisher(
            Bool, 'teleop/joystick_active', STATE_QOS)
        self._drive_state_sub = self.create_subscription(
            DriveState, 'drive/state', self._on_drive_state, STATE_QOS)

        # Every button below is a momentary press asking somebody else to
        # invert a state they own, which is why they are all Trigger and none
        # of them is SetBool: this node has no copy to compute a target from.
        self._requests = {
            name: GuardedCall(self.create_client(Trigger, name))
            for name in (
                'drive/power/toggle',
                'drive/compact/toggle',
                'drive/clear_errors',
                'lights/spotlight/toggle',
                'lights/beautiful/toggle',
                'cameras/capture',
                'cameras/record/toggle',
            )
        }

        # Off for keyboard_teleop_node: there is no gamepad to paint, and one
        # plugged into the same laptop belongs to another interpreter.
        self._led = (ControllerLed(self.get_logger())
                     if bool(declare_and_get('controller_led', True)) else None)
        # Recomputes the colour each tick, so this also carries state changes
        # that no button press announces.
        self._led_timer = self.create_timer(LED_REPAINT_PERIOD, self._refresh_led)

        self._timeout_timer = self.create_timer(
            1.0 / max(0.001, self._timeout_pub_rate), self._timeout_check)
        self._publish_timer = self.create_timer(
            1.0 / max(0.001, self._cmd_pub_rate), self._publish_timer_cb)

        # Button layout comes only from gamepad.yaml: a function the yaml does
        # not give a button (or gives -1) is off, so niche ones cost nothing
        # on a pad with few buttons. No fallback indices here on purpose -
        # they drifted from the yaml and nobody noticed.
        bindings = {
            'vy_toggle_button': self._on_vy_toggle_pressed,
            'motor_toggle_button': lambda: self._request('drive/power/toggle'),
            'raw_twist_mode_button': self._on_raw_twist_mode_toggle_pressed,
            'compact_mode_button': lambda: self._request('drive/compact/toggle'),
            'granny_button': self._on_granny_toggle_pressed,
            'spotlight_button': lambda: self._request('lights/spotlight/toggle'),
            'beautiful_button': lambda: self._request('lights/beautiful/toggle'),
            'active_toggle_button': self._on_active_toggle_pressed,
            'clear_errors_button': lambda: self._request('drive/clear_errors'),
            'dataset_capture_button': lambda: self._request('cameras/capture'),
            'dataset_record_button': lambda: self._request('cameras/record/toggle'),
        }
        layout = {name: int(declare_and_get(name, DISABLED)) for name in bindings}
        self._toggles = [ButtonToggle(layout[name], on_press)
                         for name, on_press in bindings.items()]
        self._log_layout(layout)

        self.get_logger().info(
            f'GamepadInterpreter started — '
            f'vy_enabled={self._vy_enabled}, '
            f'deadzone={self._deadzone}'
        )

        self._publish_active()
        self._refresh_led()

    def _log_layout(self, layout: dict):
        bound = ', '.join(f'{name}={index}' for name, index in layout.items() if index >= 0)
        off = ', '.join(name for name, index in layout.items() if index < 0)
        self.get_logger().info(f'Buttons: {bound or "none"}')
        if off:
            self.get_logger().info(f'Not bound (off): {off}')
        for index, names in shared_buttons(layout).items():
            self.get_logger().warn(
                f'Button {index} is bound to several functions, one press fires '
                f'all of them: {", ".join(names)}')

    # =======================================================================
    # Input
    # =======================================================================

    def _on_joy(self, msg: Joy):
        """Refresh the watchdog timestamp and run every button's edge detector."""
        if self._watchdog.on_message(self._now_seconds()):
            self.get_logger().info('Joystick input recovered — resuming command forwarding')
            # A controller that dropped and came back has lost whatever colour
            # it was wearing, so repaint it.
            self._refresh_led()

        for toggle in self._toggles:
            toggle.update(msg.buttons)

        wz_axis = self._deadzoned(msg.axes[self._axis_wz])

        self._input = JoyInput(
            vx=self._deadzoned(msg.axes[self._axis_vx]) * self._scale_vx,
            vy=self._deadzoned(msg.axes[self._axis_vy]) * self._scale_vy,
            wz=wz_axis * self._scale_wz,
            steer=wz_axis,
            rot=trigger_diff(msg.axes, self._axis_l2, self._axis_r2,
                             self._trigger_deadzone),
            triggers_held=triggers_held(msg.axes, self._axis_l2, self._axis_r2,
                                        self._trigger_deadzone),
        )

    def _deadzoned(self, value: float) -> float:
        return apply_deadzone(value, self._deadzone)

    # =======================================================================
    # Output
    # =======================================================================

    def _publish_timer_cb(self):
        """Publish the latest known command at a fixed rate (default 20 Hz),
        regardless of how often /joy actually fires. Suppressed while timed out —
        _timeout_check takes over publishing the stop burst in that case."""
        if self._watchdog.timed_out or not self._active:
            return

        modes = DriveModes(
            raw_twist=self._raw_twist_mode,
            vy_enabled=self._vy_enabled,
            granny=self._granny_mode,
        )
        self._publish_cmd(*twist_from_input(self._input, modes, self._params))

    def _timeout_check(self):
        """Apply /joy freshness timeout and hand the topic back when stale.

        Timeout *detection* runs whether or not the joystick holds control:
        while yielding to nav there is nothing to publish, but a controller
        that drops and comes back still needs its light bar repainted, and that
        repaint hangs off the timed-out → recovered edge in _on_joy(). Only the
        stop burst is conditional on _active.
        """
        tick = self._watchdog.tick(self._now_seconds(), self._active)

        if tick.went_stale:
            self.get_logger().warn(
                'Joystick input timed out — stopping and releasing cmd_vel_joy'
                if self._active else
                'Joystick input timed out (inactive — nav holds /cmd_vel)')

        if tick.publish_zero:
            self._publish_cmd(0.0, 0.0, 0.0)

    def publish_stop(self):
        """One zero twist, e.g. from keyboard_teleop_node on its way out."""
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_cmd(self, vx: float, vy: float, wz: float):
        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.angular.z = wz
        self._cmd_vel_pub.publish(out)

    def _publish_active(self):
        msg = Bool()
        msg.data = self._active
        self._active_pub.publish(msg)

    # =======================================================================
    # Buttons
    # =======================================================================

    def _request(self, name: str):
        """Ask the owner of some state to invert it.

        Fire and forget: what actually happened comes back on drive/state or
        lights/state, which is also where the ground station reads it.
        """
        if not self._requests[name].call(Trigger.Request(),
                                         lambda future: self._on_request_result(future, name)):
            self.get_logger().warn(f'{name} busy or not available')

    def _on_request_result(self, future, name: str):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'{name} call failed: {exc!r}')
            return

        # Separate call sites: rclpy pins a severity to each one and raises if the
        # same line logs at another.
        if result.success:
            self.get_logger().info(f'{name}: {result.message}')
        else:
            self.get_logger().warn(f'{name}: {result.message}')

    def _on_vy_toggle_pressed(self):
        self._vy_enabled = not self._vy_enabled
        self.get_logger().info(
            f'vy mode: {"ENABLED" if self._vy_enabled else "DISABLED"}')

    def _on_granny_toggle_pressed(self):
        self._granny_mode = not self._granny_mode
        self.get_logger().info(
            f'granny mode: {"ENABLED" if self._granny_mode else "DISABLED"}')

    def _on_raw_twist_mode_toggle_pressed(self):
        self._raw_twist_mode = not self._raw_twist_mode
        self.get_logger().info(
            f'Switching to {"RAW TWIST (Direct)" if self._raw_twist_mode else "PROCESSED (Curvature)"} mode')

    def _on_active_toggle_pressed(self):
        self._active = not self._active
        self.get_logger().info(
            f'Joystick control: '
            f'{"ACTIVE (publishing to /cmd_vel)" if self._active else "INACTIVE (yielding)"}')
        self._publish_active()
        self._refresh_led()

    # =======================================================================
    # Feedback
    # =======================================================================

    def _on_drive_state(self, msg: DriveState):
        self._drive_state = msg
        self._refresh_led()

    def mark_led_offline(self):
        if self._led is not None:
            self._led.set(LED_NODE_DOWN)

    def _refresh_led(self):
        """Paint the controller's light bar with the current drive state.

        With drive_manager_node not running nothing ever arrives on drive/state,
        the default message reads motors-off, and the bar goes red — which is
        the truth: no controller has been activated, so the rover cannot move.
        """
        if self._led is None:
            return
        self._led.set(led_colour(
            motors_enabled=self._drive_state.motors_enabled,
            motors_inhibited=self._drive_state.motors_inhibited,
            controller_active=self._drive_state.controller_active,
            joystick_active=self._active,
        ))

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = GamepadInterpreterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C arrives as KeyboardInterrupt. SIGTERM — 'systemctl stop rover',
        # 'docker stop', ros2 launch tearing down its children — is handled by
        # rclpy itself, which shuts the context down and makes spin() raise
        # ExternalShutdownException. Catching it keeps the exit clean; either
        # way the finally block below still runs.
        pass
    finally:
        node.mark_led_offline()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
