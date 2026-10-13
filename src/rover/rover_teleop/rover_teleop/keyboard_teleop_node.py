#!/usr/bin/env python3
"""
Keyboard teleop for the rover, from a terminal (ground station laptop).

    ros2 run rover_teleop keyboard_teleop_node

Keys are read from THIS terminal only (stdin), so typing in another window
never moves the rover. They become the same sensor_msgs/Joy the DualSense
produces (keyboard_input.KeyboardJoy) and go into a gamepad_interpreter
running in this process, so the keyboard shares every mode, toggle and safety
rule of the gamepad; button indices come from the same config/gamepad.yaml.

Must be started with ros2 run, not ros2 launch: launch does not hand the
terminal to its children. Namespace: $ROVER_NAMESPACE, default 'rover'.
"""

import os
import select
import signal
import sys
import termios
import tty

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy

from rover_teleop.gamepad_interpreter_node import GamepadInterpreterNode
from rover_teleop.keyboard_input import BUTTON_KEYS, HELP, KeyboardJoy

STOP_BURST = 3   # zero twists sent on exit, like the interpreter's timeout burst


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def _interpreter_parameters() -> dict:
    """gamepad_interpreter's parameters from config/gamepad.yaml, flattened."""
    path = os.path.join(get_package_share_directory('rover_teleop'), 'config', 'gamepad.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['/**/gamepad_interpreter']['ros__parameters']

    flat = {}

    def walk(prefix, value):
        if isinstance(value, dict):
            for key, sub in value.items():
                walk(f'{prefix}{key}.', sub)
        else:
            flat[prefix[:-1]] = value

    walk('', params)
    return flat


class KeyboardJoyNode(Node):
    """Reads the terminal and publishes Joy for the in-process interpreter."""

    def __init__(self, button_indices: dict, **node_kwargs):
        super().__init__('keyboard_joy', **node_kwargs)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_gs')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('initial_hold', 0.6)
        self.declare_parameter('repeat_hold', 0.15)

        self.keys = KeyboardJoy(
            button_indices,
            speed=float(self.get_parameter('speed').value),
            initial_hold=float(self.get_parameter('initial_hold').value),
            repeat_hold=float(self.get_parameter('repeat_hold').value),
        )
        self._pub = self.create_publisher(Joy, 'keyboard/joy', 10)
        self._status = None
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    @property
    def cmd_vel_topic(self) -> str:
        return str(self.get_parameter('cmd_vel_topic').value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        now = self._now()
        while select.select([sys.stdin], [], [], 0.0)[0]:
            ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
            if ch:
                self.keys.on_key(ch, now)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = self.keys.axes(now)
        msg.buttons = self.keys.buttons()
        self._pub.publish(msg)
        self._show_status(now)

    def _show_status(self, now: float):
        held = self.keys.held_key(now)
        status = (held, self.keys.speed)
        if status != self._status:
            self._status = status
            sys.stdout.write(f'\r  key: {held or "-":<5} speed: {self.keys.speed * 100:3.0f} %   ')
            sys.stdout.flush()


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop_node needs a terminal: run it with ros2 run, '
              'not from a launch file.', file=sys.stderr)
        sys.exit(1)

    # Ctrl+C is turned into KeyboardInterrupt below instead of rclpy shutting
    # the context down, so a zero twist can still be sent on the way out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    namespace = os.environ.get('ROVER_NAMESPACE', 'rover')
    params = _interpreter_parameters()
    button_indices = {name: params[name] for name in BUTTON_KEYS.values() if name in params}

    keyboard = KeyboardJoyNode(button_indices, namespace=namespace)
    overrides = [Parameter(name, value=value) for name, value in params.items()]
    overrides.append(Parameter('controller_led', value=False))
    interpreter = GamepadInterpreterNode(
        node_name='keyboard_interpreter',
        namespace=namespace,
        parameter_overrides=overrides,
        cli_args=[
            '--ros-args',
            '-r', 'joy:=keyboard/joy',
            '-r', f'cmd_vel:={keyboard.cmd_vel_topic}',
            # The gamepad's interpreter owns teleop/joystick_active.
            '-r', 'teleop/joystick_active:=teleop/keyboard_active',
        ],
    )

    executor = SingleThreadedExecutor()
    executor.add_node(keyboard)
    executor.add_node(interpreter)

    stdin_fd = sys.stdin.fileno()
    saved_tty = termios.tcgetattr(stdin_fd)
    print(HELP)
    print(f'  -> /{namespace}/{keyboard.cmd_vel_topic}\n')
    try:
        tty.setcbreak(stdin_fd)   # keys arrive one by one, no echo; Ctrl+C still works
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, saved_tty)
        print('\nStopping the rover and exiting.')
        keyboard.keys.stop()
        for _ in range(STOP_BURST):
            interpreter.publish_stop()
        executor.shutdown()
        keyboard.destroy_node()
        interpreter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
