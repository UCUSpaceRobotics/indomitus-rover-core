"""Arm power transitions and the release gate (arm_power_state, no ROS)."""
from arm_power.arm_power_state import (
    ArmPower, after_power_on, after_release, can_move, in_park_pose, observed,
    release_refusal, request_started, stiffness_ready,
)

PARK = {'shoulder': -0.97, 'elbow': 1.36}
RAMP = 15.0


def test_motors_seen_on_start_the_stiffness_clock_then():
    state = observed(ArmPower(), True, True, now=100.0)
    assert state.activated_at == 100.0
    assert not stiffness_ready(state, 110.0, RAMP)
    assert stiffness_ready(state, 115.0, RAMP) and can_move(state, 115.0, RAMP)


def test_a_later_poll_keeps_the_original_activation_time():
    state = observed(observed(ArmPower(), True, True, 100.0), True, True, 120.0)
    assert state.activated_at == 100.0


def test_motors_off_clear_the_clock():
    state = observed(observed(ArmPower(), True, True, 100.0), False, False, 120.0)
    assert state.activated_at is None and not stiffness_ready(state, 200.0, RAMP)


def test_no_ramp_in_simulation_means_ready_at_once():
    state = observed(ArmPower(), True, True, 100.0)
    assert stiffness_ready(state, 100.0, 0.0)


def test_power_on_success_and_failure():
    busy = request_started(ArmPower(last_error='old'))
    assert busy.busy and busy.last_error == ''
    on = after_power_on(busy, True, now=50.0)
    assert on.motors_enabled and on.activated_at == 50.0 and not on.busy
    failed = after_power_on(busy, False, 50.0, error='boom')
    assert not failed.motors_enabled and failed.last_error == 'boom' and not failed.busy


def test_release_success_turns_everything_off():
    state = after_release(request_started(ArmPower(True, True, 10.0)), True)
    assert state == ArmPower()


def test_aborted_release_keeps_the_motors_on_and_says_why():
    state = after_release(request_started(ArmPower(True, True, 10.0)), False, 'moved')
    assert state.motors_enabled and state.last_error == 'moved' and not state.busy


def test_release_gate():
    on = ArmPower(True, True, 0.0)
    assert release_refusal(on, PARK, {}, PARK, 0.08, 0.15) is None
    assert 'park' in release_refusal(on, {**PARK, 'shoulder': 0.0}, {}, PARK, 0.08, 0.15)
    assert 'moving' in release_refusal(on, PARK, {'elbow': 0.5}, PARK, 0.08, 0.15)
    assert 'progress' in release_refusal(request_started(on), PARK, {}, PARK, 0.08, 0.15)
    assert 'no park pose' in release_refusal(on, PARK, {}, {}, 0.08, 0.15)
    assert release_refusal(ArmPower(), {}, {}, PARK, 0.08, 0.15) is None   # already off


def test_in_park_pose():
    assert in_park_pose(PARK, PARK, 0.08)
    assert not in_park_pose({**PARK, 'elbow': 1.0}, PARK, 0.08)
    assert not in_park_pose(PARK, {}, 0.08)
