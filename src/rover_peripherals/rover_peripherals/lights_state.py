#!/usr/bin/env python3
"""The authoritative light state and the CAN payload it turns into.

The rover has two operators and they ask for lights differently. The onboard
joystick has momentary buttons, so it asks for a toggle; the ground station has
latching switches, so it asks for an absolute value. Neither can keep its own
copy of the state without the two drifting apart the moment the other one
touches a light — so the state lives here, next to the hardware, and both read
it back off `lights/state`.

No ROS import anywhere in here — lights_state is deliberately standalone.
"""

from dataclasses import dataclass, replace


#: Tri-state command for one traffic-light colour.
#:
#: KEEP is 0 on purpose. rosidl zero-initialises a request, so a caller that
#: fills in one colour and leaves the rest alone gets exactly that. The old
#: all-four-bools request did the opposite: it switched the other three off
#: unless the caller tracked the full state itself, which is precisely the
#: bookkeeping we are trying to delete from the callers.
KEEP = 0
OFF = 1
ON = 2

TRAFFIC_COLOURS = ('red', 'yellow', 'green', 'blue')

#: Bit positions in the CAN payload the ESP32 expects.
TRAFFIC_BITS = {'red': 0, 'yellow': 1, 'green': 2, 'blue': 3}


@dataclass(frozen=True)
class LightsState:
    """Every light the rover can switch, as the node last confirmed it.

    Frozen so a failed CAN transaction cannot leave a half-applied state
    behind: the handlers build the state they want, and only swap it in once
    the ESP32 has acknowledged.
    """

    spotlight: bool = False
    beautiful: bool = False
    traffic_red: bool = False
    traffic_yellow: bool = False
    traffic_green: bool = False
    traffic_blue: bool = False


def traffic_mask(state: LightsState) -> int:
    """Pack the four traffic colours into the firmware's bitmask."""
    mask = 0
    for colour, bit in TRAFFIC_BITS.items():
        if getattr(state, f'traffic_{colour}'):
            mask |= 1 << bit
    return mask


def apply_tri_state(current: bool, command: int) -> bool:
    """Resolve one tri-state command against the colour's current value."""
    if command == KEEP:
        return current
    if command == ON:
        return True
    if command == OFF:
        return False
    raise ValueError(
        f'expected KEEP ({KEEP}), OFF ({OFF}) or ON ({ON}), got {command}')


def resolve_traffic(
    state: LightsState,
    *,
    red: int = KEEP,
    yellow: int = KEEP,
    green: int = KEEP,
    blue: int = KEEP,
) -> LightsState:
    """Apply a tri-state request, leaving every KEEP colour untouched.

    This is what makes each colour independently switchable: a caller that
    wants blue on says so about blue, and says nothing about the rest.
    """
    commands = {'red': red, 'yellow': yellow, 'green': green, 'blue': blue}
    changes = {}
    for colour, command in commands.items():
        try:
            changes[f'traffic_{colour}'] = apply_tri_state(
                getattr(state, f'traffic_{colour}'), command)
        except ValueError as exc:
            raise ValueError(f'{colour}: {exc}') from None
    return replace(state, **changes)


def describe_traffic(state: LightsState) -> str:
    """Human-readable colour summary for log lines and service messages."""
    return ' '.join(
        f'{colour[0].upper()}={int(getattr(state, f"traffic_{colour}"))}'
        for colour in TRAFFIC_COLOURS
    )
