"""Terminal keys -> Joy: hold detection, layout, and button edges.

A terminal never reports a key release, so "held" is inferred from autorepeat.
Getting that wrong either stops the rover in the autorepeat gap after the
first character, or keeps it driving after the operator let go.

No ROS import anywhere in here — keyboard_input is deliberately standalone.
"""

from rover_teleop.joy_input import ButtonToggle
from rover_teleop.keyboard_input import AXIS_VX, AXIS_VY, AXIS_WZ, KeyboardJoy

INDICES = {
    'motor_toggle_button': 6,
    'granny_button': 0,
    'raw_twist_mode_button': 3,
    'vy_toggle_button': 4,
    'compact_mode_button': 1,
    'active_toggle_button': 2,
    'clear_errors_button': 13,
    'spotlight_button': 9,
    'beautiful_button': 10,
}


def make(**kwargs):
    return KeyboardJoy(INDICES, speed=0.5, initial_hold=0.6, repeat_hold=0.15, **kwargs)


# ── hold detection ───────────────────────────────────────────────────────────

def test_first_character_bridges_the_autorepeat_delay():
    k = make()
    k.on_key('w', 0.0)
    assert k.held_key(0.5) == 'w'          # inside the ~0.5 s autorepeat delay
    assert k.held_key(0.61) is None


def test_repeats_keep_it_held_and_stopping_them_releases_quickly():
    k = make()
    k.on_key('w', 0.0)
    for t in (0.5, 0.53, 0.56, 0.59):     # autorepeat stream
        k.on_key('w', t)
    assert k.held_key(0.70) == 'w'
    assert k.held_key(0.75) is None        # 0.16 s after the last repeat


def test_a_new_movement_key_replaces_the_old_one():
    k = make()
    k.on_key('w', 0.0)
    k.on_key('a', 0.1)
    assert k.held_key(0.2) == 'a'


def test_s_and_space_stop_immediately():
    for stop in ('s', ' '):
        k = make()
        k.on_key('w', 0.0)
        k.on_key(stop, 0.1)
        assert k.held_key(0.1) is None
        assert k.axes(0.1) == [0.0] * 6


# ── layout ───────────────────────────────────────────────────────────────────

def test_forward_and_turn_use_the_gamepad_axes():
    k = make()
    k.on_key('w', 0.0)
    assert k.axes(0.0)[AXIS_VX] == 0.5
    k.on_key('d', 0.0)
    axes = k.axes(0.0)
    assert axes[AXIS_VX] == 0.0 and axes[AXIS_WZ] == -0.5   # right = clockwise


def test_diagonals_drive_and_turn_together():
    k = make()
    k.on_key('q', 0.0)
    axes = k.axes(0.0)
    assert axes[AXIS_VX] == 0.5 and axes[AXIS_WZ] == 0.5 and axes[AXIS_VY] == 0.0


def test_speed_keys_scale_the_deflection():
    k = make()
    k.on_key('0', 0.0)
    k.on_key('x', 0.0)
    assert k.axes(0.0)[AXIS_VX] == -1.0
    k.on_key('2', 0.0)
    assert k.axes(0.0)[AXIS_VX] == -0.2


def test_triggers_rest_at_zero():
    k = make()
    k.on_key('w', 0.0)
    axes = k.axes(0.0)
    assert axes[4] == 0.0 and axes[5] == 0.0


# ── buttons ──────────────────────────────────────────────────────────────────

def test_a_press_is_one_edge_on_the_configured_index():
    k = make()
    k.on_key('m', 0.0)
    first, second = k.buttons(), k.buttons()
    assert first[6] == 1 and sum(first) == 1
    assert sum(second) == 0


def test_pressing_twice_gives_two_edges_to_the_interpreter():
    k = make()
    presses = []
    toggle = ButtonToggle(6, lambda: presses.append(1))
    k.on_key('m', 0.0)
    k.on_key('m', 0.01)                    # both before the next Joy message
    for _ in range(5):
        toggle.update(k.buttons())
    assert len(presses) == 2


def test_clear_errors_needs_the_capital_key():
    k = make()
    k.on_key('x', 0.0)                     # lower case is "back", not a button
    assert sum(k.buttons()) == 0
    k.on_key('X', 0.0)
    assert k.buttons()[13] == 1


def test_unknown_keys_are_ignored():
    k = make()
    assert k.on_key('?', 0.0) == ''
    assert k.axes(0.0) == [0.0] * 6 and sum(k.buttons()) == 0
