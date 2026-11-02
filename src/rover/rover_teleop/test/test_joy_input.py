"""Deadzone, triggers, and button edges.

None of this is interesting on its own; all of it is interesting to get wrong.
A deadzone that steps rather than ramps makes the rover twitch off centre, and
a button read as a level rather than an edge fires its action twenty times a
second for as long as a finger rests on it.

No ROS import anywhere in here — joy_input is deliberately standalone.
"""

import math

from rover_teleop.joy_input import (
    ButtonToggle,
    apply_deadzone,
    trigger_diff,
    triggers_held,
)


# ── deadzone ─────────────────────────────────────────────────────────────────

def test_resting_noise_is_swallowed():
    assert apply_deadzone(0.03, 0.05) == 0.0
    assert apply_deadzone(-0.03, 0.05) == 0.0


def test_the_response_is_continuous_at_the_deadzone_edge():
    # A plain threshold would jump straight to 0.05 here. On a swerve rover
    # that step is a visible flick of all four wheels the moment the stick
    # leaves centre.
    assert apply_deadzone(0.05, 0.05) == 0.0
    assert apply_deadzone(0.0500001, 0.05) < 1e-5


def test_full_deflection_still_reaches_full_scale():
    # The rescaling must not cost the operator top speed.
    assert apply_deadzone(1.0, 0.05) == 1.0
    assert apply_deadzone(-1.0, 0.05) == -1.0


def test_the_sign_of_the_stick_survives():
    assert apply_deadzone(-0.5, 0.05) < 0.0
    assert math.isclose(apply_deadzone(-0.5, 0.05), -apply_deadzone(0.5, 0.05))


def test_a_zero_deadzone_is_a_pass_through():
    # joy.launch.py pins the driver deadzone at 0.0 and filters here instead;
    # a config that also zeroes this one must not start dividing by 1-0.
    assert apply_deadzone(0.01, 0.0) == 0.01


# ── triggers ─────────────────────────────────────────────────────────────────

def test_untouched_triggers_ask_for_nothing():
    axes = [0.0] * 6

    assert trigger_diff(axes, 4, 5, 0.15) == 0.0
    assert triggers_held(axes, 4, 5, 0.15) is False


def test_one_trigger_spins_one_way_and_the_other_spins_back():
    left = [0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
    right = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]

    assert trigger_diff(left, 4, 5, 0.15) > 0.0
    assert trigger_diff(right, 4, 5, 0.15) < 0.0


def test_both_triggers_held_to_a_draw_reads_as_no_rotation_but_still_held():
    # The pair that has to be told apart: the operator is asking to hold the
    # spin-in-place wheel shape without turning. Reading this as 'nothing is
    # touched' lets the wheels home while they are still being held.
    axes = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0]

    assert trigger_diff(axes, 4, 5, 0.15) == 0.0
    assert triggers_held(axes, 4, 5, 0.15) is True


def test_a_small_imbalance_is_deadzoned_away():
    axes = [0.0, 0.0, 0.0, 0.0, -1.0, -0.95]

    assert trigger_diff(axes, 4, 5, 0.15) == 0.0


def test_an_axis_index_off_the_end_reads_as_untouched():
    # Not every pad reports six axes, and a controller that enumerates with
    # fewer must not take teleop down with an IndexError.
    axes = [0.0, 0.0]

    assert trigger_diff(axes, 4, 5, 0.15) == 0.0
    assert triggers_held(axes, 4, 5, 0.15) is False


# ── button edges ─────────────────────────────────────────────────────────────

class Counter:
    def __init__(self):
        self.count = 0

    def __call__(self):
        self.count += 1


def test_holding_a_button_fires_once():
    # /joy repeats at 20 Hz. Acting on the level would toggle the light twenty
    # times a second for as long as the button is down.
    pressed = Counter()
    toggle = ButtonToggle(2, pressed)

    for _ in range(20):
        toggle.update([0, 0, 1])

    assert pressed.count == 1


def test_releasing_and_pressing_again_fires_again():
    pressed = Counter()
    toggle = ButtonToggle(0, pressed)

    toggle.update([1])
    toggle.update([0])
    toggle.update([1])

    assert pressed.count == 2


def test_the_press_edge_is_reported_to_the_caller():
    toggle = ButtonToggle(0, lambda: None)

    assert toggle.update([1]) is True
    assert toggle.update([1]) is False
    assert toggle.update([0]) is False


def test_a_button_index_past_the_end_never_fires():
    # clear_errors_button used to sit at 20, past the end of any real gamepad.
    # A pad that reports fewer buttons must read as 'not pressed', not crash.
    pressed = Counter()
    toggle = ButtonToggle(20, pressed)

    toggle.update([1, 1, 1])

    assert pressed.count == 0


def test_reset_forgets_a_button_that_was_down():
    # After a disconnect the pad's last known state is meaningless; without the
    # reset, a button that was held when the cable went would fire on reconnect.
    pressed = Counter()
    toggle = ButtonToggle(0, pressed)
    toggle.update([1])

    toggle.reset()
    toggle.update([1])

    assert pressed.count == 2
