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

STOP_KEYS = (' ', 's')

# '1'..'9' -> 10..90 %, '0' -> 100 % stick deflection.
SPEED_KEYS = {str(d): d / 10.0 for d in range(1, 10)}
SPEED_KEYS['0'] = 1.0

# Keys the keyboard uses for driving; config/keyboard.yaml cannot bind them.
RESERVED_KEYS = set(MOVE_KEYS) | set(STOP_KEYS) | set(SPEED_KEYS)

# Every interpreter function a key can press (gamepad.yaml parameter names),
# with its line in the help text. Which key, if any, comes from
# config/keyboard.yaml - see parse_keymap.
FUNCTIONS = {
    'motor_toggle_button': 'motors on/off',
    'raw_twist_mode_button': 'raw twist / curvature',
    'granny_button': 'granny',
    'vy_toggle_button': 'vy (strafe)',
    'compact_mode_button': 'compact mode',
    'active_toggle_button': 'active (yield to others)',
    'spotlight_button': 'spotlight',
    'beautiful_button': 'beautiful lights',
    'clear_errors_button': 'clear motor errors',
    'dataset_capture_button': 'dataset: save one frame',
    'dataset_record_button': 'dataset: record on/off',
}

_HELP_HEAD = """\
Rover keyboard teleop (this terminal must have focus)
  w / x      forward / back          a / d   turn left / right
  q / e      forward-left / -right   z / c   back-left / -right
  j / l      strafe left / right (only with vy mode on)
  s, space   stop                    1..9,0  speed 10..100 %
"""


def parse_keymap(raw: dict) -> dict:
    """config/keyboard.yaml `keys` (function -> key) -> {key: function}.

    An empty or missing key leaves the function off. Every mistake is
    collected and raised together, so one run shows all of them.
    """
    keymap, problems = {}, []
    for function, key in (raw or {}).items():
        if function not in FUNCTIONS:
            problems.append(f'unknown function {function!r} '
                            f'(known: {", ".join(FUNCTIONS)})')
            continue
        if key is None or key == '':
            continue
        if not isinstance(key, str) or len(key) != 1:
            problems.append(f'{function}: key must be one character in quotes, got {key!r}')
        elif key in RESERVED_KEYS:
            problems.append(f'{function}: {key!r} is a driving key (move/stop/speed)')
        elif key in keymap:
            problems.append(f'{function}: {key!r} is already {keymap[key]}')
        else:
            keymap[key] = function
    if problems:
        raise ValueError('config/keyboard.yaml: ' + '; '.join(problems))
    return keymap


def button_indices(keymap: dict) -> dict:
    """The keyboard's own virtual Joy layout: {function: button index}, one
    button per bound function in keymap order. keyboard_teleop_node hands this
    to its interpreter, so it is independent of the gamepad's layout."""
    return {function: index for index, function in enumerate(keymap.values())}


def help_text(keymap: dict) -> str:
    lines = [f'  {key}  {FUNCTIONS[function]}' for key, function in keymap.items()]
    return _HELP_HEAD + '\n'.join(lines) + '\n  Ctrl+C  quit (stops the rover)\n'


@dataclass
class _HeldKey:
    key: str
    first_seen: float
    last_seen: float
    repeated: bool = False


class KeyboardJoy:
    """Turns timestamped key characters into Joy axes and buttons."""

    def __init__(self, keymap: dict, speed: float = 0.5,
                 initial_hold: float = 0.6, repeat_hold: float = 0.15):
        """keymap: {key: function}, from parse_keymap."""
        indices = button_indices(keymap)
        self._buttons = {key: indices[function] for key, function in keymap.items()}
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
