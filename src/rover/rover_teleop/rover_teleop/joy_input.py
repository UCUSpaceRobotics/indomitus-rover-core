#!/usr/bin/env python3
"""Turning a raw sensor_msgs/Joy into something worth acting on.

Deadzoning happens here rather than in the joy driver: game_controller_node
stops publishing /joy entirely when its own deadzone swallows resting-stick
noise (ros-drivers/joystick_drivers#304), so joy.launch.py keeps the driver
deadzone at 0.0 and this filters instead.

No ROS import anywhere in here — joy_input is deliberately standalone.
"""

import math


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero out small stick values, rescaling the rest so the response stays
    continuous at the deadzone edge.
    """
    if deadzone <= 0.0:
        return value
    if abs(value) < deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def trigger_diff(axes, l2_index: int, r2_index: int, deadzone: float) -> float:
    """L2 minus R2, deadzoned. Positive = L2 held = counter-clockwise spin."""
    def value(index: int) -> float:
        return axes[index] if 0 <= index < len(axes) else 0.0

    diff = value(r2_index) - value(l2_index)
    return 0.0 if abs(diff) < deadzone else diff


def triggers_held(axes, l2_index: int, r2_index: int, deadzone: float) -> bool:
    """Whether either trigger is pulled at all.

    trigger_diff() reads zero both when nothing is touched and when both
    triggers are held to a draw; this tells those two apart. abs() because the
    triggers rest at 0.0 and read -1.0 when pulled.
    """
    def value(index: int) -> float:
        return axes[index] if 0 <= index < len(axes) else 0.0

    return max(abs(value(l2_index)), abs(value(r2_index))) > deadzone


class ButtonToggle:
    """Rising-edge detector for a single joystick button.

    The buttons are momentary and /joy repeats at 20 Hz, so acting on the level
    would fire the action twenty times a second for as long as a finger rests
    on the button.
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
