#!/usr/bin/env python3
"""What the rover's drive is capable of, and how each service reply moves it.

This used to be four loose booleans on the joystick node, which meant the
joystick was the only thing that knew whether the rover could move. With a
ground station commanding the same hardware, one owner has to hold this and
publish it; these are that owner's transition rules, with the ROS calls taken
out so the ordering can be tested without a controller_manager.

No ROS import anywhere in here — drive_power_state is deliberately standalone.
"""

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class DrivePower:
    """Drive capability as the owner last confirmed it with the hardware.

    Frozen: a reply that failed must leave the previous state standing, and
    the only way to be sure of that is to build the new one and swap it in.
    """

    #: Hardware component RoverHardware is in the ACTIVE lifecycle state.
    motors_enabled: bool = False
    #: The swerve controller is active, i.e. /cmd_vel reaches the wheels.
    controller_active: bool = False
    #: Latched faults were cleared. The motors still report enabled, but the
    #: hardware will not drive until power is cycled — see after_errors_cleared.
    motors_inhibited: bool = False
    #: The swerve controller is in compact mode.
    compact_mode: bool = False

    @property
    def can_drive(self) -> bool:
        """True only when a twist on /cmd_vel actually moves the rover.

        Every consumer that says 'ready' to an operator — the light bar, the
        ground station indicator — has to agree on this, and each of them
        re-deriving it is how they end up disagreeing.
        """
        return (
            self.motors_enabled
            and self.controller_active
            and not self.motors_inhibited
        )


def seeded(state: DrivePower, motors_enabled: bool, controller_active: bool) -> DrivePower:
    """Adopt the state controller_manager reports at startup.

    Without this the node would assume everything is off, which is right on
    hardware — bringup spawns the controller inactive — and wrong in
    simulation, where both come up active. The consequence of guessing is not
    cosmetic: the light bar would read red on a rover that drives, and the
    operator's first press of the motor button would ask for the state the
    hardware is already in.

    It also covers this node being restarted underneath a powered rover.

    compact_mode is deliberately not seeded: the swerve controller offers no
    way to read it back, so it stays at its own default of off.
    """
    return replace(
        state, motors_enabled=motors_enabled, controller_active=controller_active)


def after_power_result(state: DrivePower, ok: bool, desired: bool) -> DrivePower:
    """Apply a set_hardware_component_state reply.

    Cycling power is also the recovery step that lifts a post-clear inhibit,
    so a successful reply in either direction clears it.
    """
    if not ok:
        return state
    return replace(state, motors_enabled=desired, motors_inhibited=False)


def after_controller_result(state: DrivePower, ok: bool, activate: bool) -> DrivePower:
    """Apply a switch_controller reply."""
    if not ok:
        return state
    return replace(state, controller_active=activate)


def after_compact_result(state: DrivePower, ok: bool, desired: bool) -> DrivePower:
    """Apply a set_compact_mode reply."""
    if not ok:
        return state
    return replace(state, compact_mode=desired)


def after_errors_cleared(state: DrivePower) -> DrivePower:
    """Apply a successful clear_motor_errors.

    Nothing tells us the motors dropped out, and clearing motors_enabled here
    would cut across the recovery sequence the operator is halfway through.
    Track it as its own state instead, so nothing claims the rover is drivable
    while the hardware is refusing to drive.
    """
    return replace(state, motors_inhibited=True)
