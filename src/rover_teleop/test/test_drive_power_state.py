"""Drive capability: what each service reply is allowed to claim.

These four booleans used to be private attributes on the joystick node, which
made the joystick the only thing in the system that knew whether the rover
could move. The rules that matter are the pessimistic ones — every state here
gates something an operator reads as "ready", and a state that over-claims is
a rover that looks drivable and is not.

No ROS import anywhere in here — drive_power_state is deliberately standalone.
"""

from rover_teleop.drive_power_state import (
    DrivePower,
    after_compact_result,
    after_controller_result,
    after_errors_cleared,
    after_power_result,
)


def powered():
    """The ordinary drivable state: motors on, controller active, no faults."""
    return DrivePower(motors_enabled=True, controller_active=True)


# ── can_drive ────────────────────────────────────────────────────────────────

def test_a_fresh_rover_cannot_drive():
    # Bringup spawns the swerve controller inactive, so this is the real state
    # at startup and not a defensive default.
    assert DrivePower().can_drive is False


def test_can_drive_needs_every_condition_at_once():
    assert powered().can_drive is True

    assert DrivePower(motors_enabled=True).can_drive is False
    assert DrivePower(controller_active=True).can_drive is False


def test_an_inhibited_rover_cannot_drive_however_enabled_it_looks():
    # This is the state that most wants to lie: motors report enabled and the
    # controller is active, but the hardware was told to drop its latched
    # faults and will refuse to move until power is cycled.
    inhibited = DrivePower(
        motors_enabled=True, controller_active=True, motors_inhibited=True)

    assert inhibited.can_drive is False


def test_compact_mode_has_no_say_over_whether_the_rover_can_drive():
    # Compact mode changes the wheel geometry, not the power path. Letting it
    # into can_drive would black out the ready indicator for a legal pose.
    assert powered().can_drive is True
    assert after_compact_result(powered(), ok=True, desired=True).can_drive is True


# ── power replies ────────────────────────────────────────────────────────────

def test_a_successful_power_reply_applies_what_was_asked_for():
    after = after_power_result(DrivePower(), ok=True, desired=True)

    assert after.motors_enabled is True


def test_a_refused_power_reply_leaves_the_state_alone():
    # controller_manager answered and said no. Recording the request as though
    # it had worked is how the light bar goes green on a dead rover.
    before = DrivePower()

    assert after_power_result(before, ok=False, desired=True) == before


def test_powering_up_does_not_by_itself_activate_the_controller():
    # The controller switch is a second call with its own reply; until it
    # lands, /cmd_vel still reaches nothing.
    after = after_power_result(DrivePower(), ok=True, desired=True)

    assert after.controller_active is False
    assert after.can_drive is False


# ── controller replies ───────────────────────────────────────────────────────

def test_a_successful_switch_applies_and_completes_the_power_sequence():
    after = after_power_result(DrivePower(), ok=True, desired=True)
    after = after_controller_result(after, ok=True, activate=True)

    assert after.can_drive is True


def test_a_refused_switch_leaves_the_state_alone():
    before = after_power_result(DrivePower(), ok=True, desired=True)

    assert after_controller_result(before, ok=False, activate=True) == before


def test_powering_down_deactivates_the_controller_too():
    state = after_controller_result(
        after_power_result(powered(), ok=True, desired=False),
        ok=True, activate=False)

    assert state.motors_enabled is False
    assert state.controller_active is False


# ── the clear-errors recovery cycle ──────────────────────────────────────────

def test_clearing_errors_inhibits_rather_than_disabling():
    # Nothing tells us the motors dropped out, and clearing motors_enabled
    # would cut across the recovery the operator is halfway through. The
    # rover must stop looking drivable without the state machine lying about
    # what the hardware reported.
    after = after_errors_cleared(powered())

    assert after.motors_enabled is True
    assert after.controller_active is True
    assert after.motors_inhibited is True
    assert after.can_drive is False


def test_cycling_power_lifts_the_inhibit():
    # The documented recovery: clear, then cycle power off and back on.
    state = after_errors_cleared(powered())

    state = after_power_result(state, ok=True, desired=False)
    assert state.motors_inhibited is False
    assert state.motors_enabled is False

    state = after_power_result(state, ok=True, desired=True)
    state = after_controller_result(state, ok=True, activate=True)
    assert state.can_drive is True


def test_a_failed_power_cycle_does_not_lift_the_inhibit():
    # Only a reply the hardware actually acknowledged may clear it — otherwise
    # a button press against an unreachable service "recovers" the rover.
    state = after_errors_cleared(powered())

    assert after_power_result(state, ok=False, desired=False).motors_inhibited is True


# ── compact mode ─────────────────────────────────────────────────────────────

def test_compact_mode_follows_a_successful_reply_and_ignores_a_failed_one():
    on = after_compact_result(DrivePower(), ok=True, desired=True)
    assert on.compact_mode is True

    assert after_compact_result(on, ok=False, desired=False) == on


# ── immutability ─────────────────────────────────────────────────────────────

def test_transitions_never_mutate_the_state_they_are_given():
    # The node commits a new state only once a reply confirms it, so a
    # transition that mutated in place would apply changes the hardware
    # refused.
    before = powered()

    after_power_result(before, ok=True, desired=False)
    after_controller_result(before, ok=True, activate=False)
    after_errors_cleared(before)
    after_compact_result(before, ok=True, desired=True)

    assert before == powered()
