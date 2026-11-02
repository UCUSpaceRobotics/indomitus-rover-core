"""Sticks to a twist, in all three drive modes.

Every branch here exists because a swerve platform needs a non-obvious command
to do an obvious thing: steer while parked, hold a spin shape without spinning,
turn the right way in reverse. They are cheap to get subtly wrong and expensive
to test by holding a gamepad in front of a rover.

No ROS import anywhere in here — drive_kinematics is deliberately standalone.
"""

import math

from rover_teleop.drive_kinematics import (
    DriveModes,
    JoyInput,
    KinematicsParams,
    swerve_wz_correction,
    twist_from_input,
)


PARAMS = KinematicsParams(
    scale_rotate=1.0,
    rot_probe_wz=1e-5,
    max_curvature=2.0,
    angle_probe_speed=1e-5,
    granny_scale=0.1,
)

RAW = DriveModes(raw_twist=True)
CURVATURE = DriveModes(raw_twist=False)


def resolve(inputs, modes=RAW, params=PARAMS):
    return twist_from_input(inputs, modes, params)


# ── raw twist mode ───────────────────────────────────────────────────────────

def test_raw_twist_passes_the_sticks_straight_through():
    vx, vy, wz = resolve(JoyInput(vx=0.5, wz=0.3),
                         DriveModes(raw_twist=True, vy_enabled=True))

    assert (vx, wz) == (0.5, 0.3)


def test_reverse_inverts_the_yaw_rate():
    # The yaw rate that turns the rover left going forward turns it right going
    # back. Without this the stick steers the wrong way the moment the operator
    # backs up, which is exactly when they are least able to watch the wheels.
    _, _, forward = resolve(JoyInput(vx=0.5, wz=0.3))
    _, _, reverse = resolve(JoyInput(vx=-0.5, wz=0.3))

    assert forward == 0.3
    assert reverse == -0.3


def test_the_reverse_correction_stops_at_the_parked_threshold():
    # A stick resting just below zero must not flip the steering.
    assert swerve_wz_correction(-1e-4, 0.3) == 0.3
    assert swerve_wz_correction(-1e-2, 0.3) == -0.3


def test_strafing_disables_the_reverse_correction():
    # With strafe on, the operator is placing the body directly; the correction
    # would fight them rather than help.
    _, _, wz = resolve(JoyInput(vx=-0.5, wz=0.3),
                       DriveModes(raw_twist=True, vy_enabled=True))

    assert wz == 0.3


# ── strafe gating ────────────────────────────────────────────────────────────

def test_strafe_is_dropped_unless_it_was_enabled():
    # The stick still reports sideways deflection; the rover must ignore it
    # until the operator asks for strafing.
    _, vy, _ = resolve(JoyInput(vx=0.5, vy=0.4))
    assert vy == 0.0

    _, vy, _ = resolve(JoyInput(vx=0.5, vy=0.4),
                       DriveModes(raw_twist=True, vy_enabled=True))
    assert vy == 0.4


def test_a_disabled_strafe_is_left_out_of_the_curvature_speed():
    # The curvature branch scales the yaw rate by total speed. Letting a
    # strafe the rover will never execute into hypot() would inflate that
    # speed and over-turn.
    _, _, wz = resolve(JoyInput(vx=0.5, vy=0.9, steer=0.5), CURVATURE)

    assert math.isclose(wz, 0.5 * (0.5 * 2.0))


# ── spin in place ────────────────────────────────────────────────────────────

def test_a_pulled_trigger_spins_and_ignores_the_sticks():
    vx, vy, wz = resolve(
        JoyInput(vx=0.9, vy=0.9, steer=0.5, rot=0.4, triggers_held=True), CURVATURE)

    assert (vx, vy) == (0.0, 0.0)
    assert wz == 0.4


def test_triggers_held_to_a_draw_hold_the_spin_shape_without_driving():
    # rot is zero but the triggers are down: the operator wants the wheels kept
    # in the spin pose. An empty twist would let the controller home them.
    vx, vy, wz = resolve(JoyInput(rot=0.0, triggers_held=True), CURVATURE)

    assert (vx, vy) == (0.0, 0.0)
    assert wz == PARAMS.rot_probe_wz


def test_the_probe_yaw_rate_is_small_enough_not_to_drive():
    # It has to stay under the controller's park_speed or 'hold the shape'
    # becomes 'creep in a circle'.
    assert 0.0 < PARAMS.rot_probe_wz < 1e-3


def test_triggers_do_nothing_in_raw_twist_mode():
    # Spin-in-place is a curvature-mode affordance; in raw twist the sticks are
    # already able to command it directly.
    vx, _, wz = resolve(JoyInput(vx=0.5, wz=0.2, rot=0.4, triggers_held=True), RAW)

    assert vx == 0.5
    assert wz == 0.2


# ── curvature ────────────────────────────────────────────────────────────────

def test_curvature_scales_the_yaw_rate_by_how_fast_the_rover_is_going():
    # A radius is a radius: the same stick has to mean a bigger yaw rate at
    # speed and a smaller one when crawling.
    _, _, slow = resolve(JoyInput(vx=0.2, steer=1.0), CURVATURE)
    _, _, fast = resolve(JoyInput(vx=0.8, steer=1.0), CURVATURE)

    assert math.isclose(slow, 0.2 * 2.0)
    assert math.isclose(fast, 0.8 * 2.0)


def test_curvature_in_reverse_turns_the_same_way_as_forward():
    # v_signed carries the sign, so the stick keeps meaning 'this side' rather
    # than swapping under the operator.
    _, _, wz = resolve(JoyInput(vx=-0.5, steer=1.0), CURVATURE)

    assert math.isclose(wz, -0.5 * 2.0)


def test_steering_while_parked_commands_an_angle_with_a_token_speed():
    # Scaling a zero speed by a curvature gives zero, and the wheels would
    # never turn. Pre-steering before moving off is a thing operators do
    # constantly, so this branch is not an edge case.
    vx, vy, wz = resolve(JoyInput(vx=0.0, steer=1.0), CURVATURE)

    assert vx == PARAMS.angle_probe_speed
    assert vy == 0.0
    assert wz != 0.0
    assert math.isclose(wz / vx, 2.0)


def test_the_probe_speed_is_small_enough_not_to_drive():
    assert 0.0 < PARAMS.angle_probe_speed < 1e-3


def test_parked_with_the_stick_centred_asks_for_nothing_at_all():
    # The probe must not creep the rover whenever nobody is touching anything.
    assert resolve(JoyInput(), CURVATURE) == (0.0, 0.0, 0.0)


def test_the_curvature_stick_is_not_scaled_by_scale_angular():
    # steer is deliberately the unscaled axis: in this mode it sets a radius,
    # not a yaw rate, so scale_angular has no business in it.
    _, _, half = resolve(JoyInput(vx=0.5, steer=0.5), CURVATURE)

    assert math.isclose(half, 0.5 * 0.5 * 2.0)


# ── granny ───────────────────────────────────────────────────────────────────

def test_granny_scales_every_component_together():
    # Scaling the linear terms alone would change the turn radius, so the rover
    # would follow a different path slowly rather than the same path slowly.
    fast = resolve(JoyInput(vx=0.5, vy=0.4, wz=0.3),
                   DriveModes(raw_twist=True, vy_enabled=True))
    slow = resolve(JoyInput(vx=0.5, vy=0.4, wz=0.3),
                   DriveModes(raw_twist=True, vy_enabled=True, granny=True))

    assert all(math.isclose(s, f * 0.1) for s, f in zip(slow, fast))


def test_granny_applies_in_curvature_mode_too():
    _, _, wz = resolve(JoyInput(vx=0.5, steer=1.0),
                       DriveModes(raw_twist=False, granny=True))

    assert math.isclose(wz, 0.5 * 2.0 * 0.1)


def test_granny_does_not_wake_the_rover_up_when_nothing_is_pressed():
    assert resolve(JoyInput(), DriveModes(raw_twist=True, granny=True)) == (0.0, 0.0, 0.0)
