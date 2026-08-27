"""Light state: the bitmask, and what a partial request is allowed to touch.

The reason this module exists at all is that two operators command the same
lights from different kinds of control — momentary buttons on the joystick,
latching switches on the ground station — and the old all-four-bools request
forced every caller to keep its own copy of the state to build one. Two copies
is one too many. These tests pin the behaviour that lets callers stop keeping
one: an unfilled field changes nothing.

No ROS import anywhere in here — lights_state is deliberately standalone.
"""

import pytest

from rover_peripherals.lights_state import (
    KEEP,
    OFF,
    ON,
    LightsState,
    apply_tri_state,
    describe_traffic,
    resolve_traffic,
    traffic_mask,
)


def all_traffic_on():
    return LightsState(
        traffic_red=True, traffic_yellow=True,
        traffic_green=True, traffic_blue=True,
    )


# ── the CAN bitmask ──────────────────────────────────────────────────────────

def test_nothing_lit_is_an_empty_mask():
    assert traffic_mask(LightsState()) == 0x00


def test_each_colour_owns_the_bit_the_firmware_expects():
    # R=bit0 Y=bit1 G=bit2 B=bit3. The ESP32 has no idea what our dataclass
    # looks like, so this ordering is a wire contract, not an implementation
    # detail — reordering the fields must not silently repaint the rover.
    assert traffic_mask(LightsState(traffic_red=True))    == 0x01
    assert traffic_mask(LightsState(traffic_yellow=True)) == 0x02
    assert traffic_mask(LightsState(traffic_green=True))  == 0x04
    assert traffic_mask(LightsState(traffic_blue=True))   == 0x08


def test_colours_combine_into_one_mask():
    assert traffic_mask(all_traffic_on()) == 0x0F
    assert traffic_mask(
        LightsState(traffic_red=True, traffic_blue=True)) == 0x09


def test_the_mask_ignores_the_lights_that_are_not_on_the_traffic_head():
    # spotlight and beautiful ride their own CAN commands; leaking them into
    # the traffic mask would light colours nobody asked for.
    assert traffic_mask(LightsState(spotlight=True, beautiful=True)) == 0x00


# ── tri-state resolution ─────────────────────────────────────────────────────

def test_keep_leaves_the_colour_exactly_as_it_was():
    assert apply_tri_state(True, KEEP) is True
    assert apply_tri_state(False, KEEP) is False


def test_on_and_off_are_absolute_regardless_of_the_current_value():
    assert apply_tri_state(False, ON) is True
    assert apply_tri_state(True, ON) is True
    assert apply_tri_state(True, OFF) is False
    assert apply_tri_state(False, OFF) is False


def test_an_unknown_command_is_rejected_rather_than_guessed():
    # Better a failed service call than a colour that quietly picks a side.
    with pytest.raises(ValueError):
        apply_tri_state(False, 7)


# ── partial requests: the whole point ────────────────────────────────────────

def test_a_zero_initialised_request_changes_nothing():
    # rosidl zero-fills a request, and KEEP is 0. A caller that forgets to set
    # a field, or a client generated from a stale interface, must not blank the
    # traffic head — that failure mode is invisible until the rover is in front
    # of the judges.
    before = all_traffic_on()

    assert resolve_traffic(before) == before


def test_switching_one_colour_leaves_the_other_three_alone():
    before = all_traffic_on()

    after = resolve_traffic(before, blue=OFF)

    assert after.traffic_blue is False
    assert after.traffic_red is True
    assert after.traffic_yellow is True
    assert after.traffic_green is True


def test_several_colours_can_move_in_one_request():
    after = resolve_traffic(LightsState(), red=ON, green=ON)

    assert traffic_mask(after) == 0x05


def test_colours_can_move_in_opposite_directions_at_once():
    # Autonomy handing over to teleop is exactly this: one colour off, another
    # on, and it has to happen as a single CAN frame or the rover flashes a
    # state it was never in.
    before = LightsState(traffic_blue=True)

    after = resolve_traffic(before, blue=OFF, green=ON)

    assert after.traffic_blue is False
    assert after.traffic_green is True


def test_resolving_never_touches_the_non_traffic_lights():
    before = LightsState(spotlight=True, beautiful=True)

    after = resolve_traffic(before, red=ON)

    assert after.spotlight is True
    assert after.beautiful is True


def test_the_input_state_is_left_untouched():
    # The node commits the returned state only once the ESP32 acknowledges, so
    # resolve_traffic must not have already mutated the state it was handed —
    # a failed CAN transaction has to leave the old state standing.
    before = LightsState()

    resolve_traffic(before, red=ON, yellow=ON, green=ON, blue=ON)

    assert traffic_mask(before) == 0x00


def test_a_bad_value_names_the_colour_that_was_wrong():
    with pytest.raises(ValueError, match='yellow'):
        resolve_traffic(LightsState(), yellow=42)


# ── log/response text ────────────────────────────────────────────────────────

def test_describe_traffic_reads_out_every_colour():
    assert describe_traffic(LightsState(traffic_red=True, traffic_blue=True)) == (
        'R=1 Y=0 G=0 B=1')
