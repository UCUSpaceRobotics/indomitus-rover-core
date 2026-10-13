#!/usr/bin/env python3
"""Terminal keys -> the same sensor_msgs/Joy layout the DualSense produces.

keyboard_teleop_node feeds this into its own gamepad_interpreter, so the
keyboard gets every mode, toggle and safety rule of the gamepad for free.

A terminal only delivers characters, never "key released": holding a key gives
one character, a pause (the autorepeat delay), then a stream of repeats. A key
therefore counts as held for `initial_hold` after its first character (to
bridge that delay) and for `repeat_hold` after each repeat. Only the last key
pressed repeats, so two movement keys cannot be combined; the diagonal keys
(q/e/z/c) cover forward/back while turning instead.

No ROS import anywhere in here — keyboard_input is deliberately standalone.
"""

from dataclasses import dataclass

# Joy axes, as gamepad_interpreter reads them (config/gamepad.yaml).
AXIS_VY = 0   # left stick horizontal, +1 = left
AXIS_VX = 1   # left stick vertical, +1 = forward
AXIS_WZ = 2   # right stick horizontal, +1 = left (counter-clockwise)
NUM_AXES = 6  # 4/5 are the triggers, resting at 0.0
NUM_BUTTONS = 16

# key -> (vx, vy, wz) stick deflection, scaled by the speed level.
MOVE_KEYS = {
    'w': (1.0, 0.0, 0.0),
    'x': (-1.0, 0.0, 0.0),
    'a': (0.0, 0.0, 1.0),
    'd': (0.0, 0.0, -1.0),
    'q': (1.0, 0.0, 1.0),
    'e': (1.0, 0.0, -1.0),
    'z': (-1.0, 0.0, 1.0),
    'c': (-1.0, 0.0, -1.0),
    'j': (0.0, 1.0, 0.0),    # strafe; only acts while vy mode is on
    'l': (0.0, -1.0, 0.0),
}

# key -> gamepad.yaml parameter holding that button's index. One press = one
# rising edge = one toggle, exactly like tapping the gamepad button.
BUTTON_KEYS = {
    'm': 'motor_toggle_button',
    'g': 'granny_button',
    't': 'raw_twist_mode_button',
    'v': 'vy_toggle_button',
    'p': 'compact_mode_button',
    'o': 'active_toggle_button',
    'X': 'clear_errors_button',   # capital on purpose: a recovery action
    'f': 'spotlight_button',
    'b': 'beautiful_button',
}

STOP_KEYS = (' ', 's')

# '1'..'9' -> 10..90 %, '0' -> 100 % stick deflection.
SPEED_KEYS = {str(d): d / 10.0 for d in range(1, 10)}
SPEED_KEYS['0'] = 1.0

HELP = """\
Rover keyboard teleop (this terminal must have focus)
  w / x      forward / back          a / d   turn left / right
  q / e      forward-left / -right   z / c   back-left / -right
  j / l      strafe left / right (only with vy mode on: v)
  s, space   stop                    1..9,0  speed 10..100 %
  m  motors on/off    t  raw twist / curvature    g  granny
  v  vy (strafe)      p  compact mode             o  active (yield to others)
  f  spotlight        b  beautiful lights         X  clear motor errors
  Ctrl+C  quit (stops the rover)
"""


@dataclass
class _HeldKey:
    key: str
    first_seen: float
    last_seen: float
    repeated: bool = False


class KeyboardJoy:
    """Turns timestamped key characters into Joy axes and buttons."""

    def __init__(self, button_indices: dict, speed: float = 0.5,
                 initial_hold: float = 0.6, repeat_hold: float = 0.15):
        """button_indices: gamepad.yaml parameter name -> button index."""
        self._buttons = {key: int(button_indices[param])
                         for key, param in BUTTON_KEYS.items() if param in button_indices}
        self.speed = speed
        self._initial_hold = initial_hold
        self._repeat_hold = repeat_hold
        self._held = None
        self._pending_presses = []
        self._release_next = False

    def on_key(self, key: str, now: float) -> str:
        """Feed one character. Returns what it did: 'move', 'button', 'speed',
        'stop' or '' (ignored)."""
        if key in MOVE_KEYS:
            if self._held is not None and self._held.key == key:
                self._held.last_seen = now
                self._held.repeated = True
            else:
                self._held = _HeldKey(key, now, now)
            return 'move'
        if key in STOP_KEYS:
            self._held = None
            return 'stop'
        if key in SPEED_KEYS:
            self.speed = SPEED_KEYS[key]
            return 'speed'
        if key in self._buttons:
            self._pending_presses.append(self._buttons[key])
            return 'button'
        return ''

    def held_key(self, now: float):
        """The movement key still considered held, or None."""
        if self._held is None:
            return None
        hold = self._repeat_hold if self._held.repeated else self._initial_hold
        if now - self._held.last_seen > hold:
            self._held = None
            return None
        return self._held.key

    def axes(self, now: float) -> list:
        axes = [0.0] * NUM_AXES
        key = self.held_key(now)
        if key is not None:
            vx, vy, wz = MOVE_KEYS[key]
            axes[AXIS_VX] = vx * self.speed
            axes[AXIS_VY] = vy * self.speed
            axes[AXIS_WZ] = wz * self.speed
        return axes

    def buttons(self) -> list:
        """Buttons for the next Joy message. A press is reported in exactly one
        message and always followed by an all-zero one, so pressing the same
        key twice gives two rising edges; further presses queue behind it."""
        buttons = [0] * max(NUM_BUTTONS, max(self._buttons.values(), default=0) + 1)
        if self._release_next:
            self._release_next = False
        elif self._pending_presses:
            buttons[self._pending_presses.pop(0)] = 1
            self._release_next = True
        return buttons

    def stop(self):
        self._held = None
        self._pending_presses.clear()
