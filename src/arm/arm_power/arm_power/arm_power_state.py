"""What the arm's motors are capable of, and how each event moves it.

The arm's counterpart of rover_power's drive_power_state: arm_power owns this
and publishes it on /arm/state; these are its transition rules, with the ROS
calls taken out so they can be tested without a controller_manager.

No ROS import anywhere in here — arm_power_state is deliberately standalone.
"""

from dataclasses import dataclass, replace

from arm_power.park import release_blockers


@dataclass(frozen=True)
class ArmPower:
    """Arm power as the owner last confirmed it with controller_manager."""

    #: Hardware component ArmSystem is ACTIVE.
    motors_enabled: bool = False
    #: The arm's trajectory or streaming controller is active.
    controller_active: bool = False
    #: Monotonic time the motors were last seen coming on: the stiffness
    #: ramp (ArmCanSystem gain_ramp_secs) runs from here. None while off.
    activated_at: float | None = None
    #: A power on / off request is in flight.
    busy: bool = False
    #: Why the last request failed; empty after a success.
    last_error: str = ''


def observed(state: ArmPower, motors_enabled: bool, controller_active: bool,
             now: float) -> ArmPower:
    """Fold in a controller_manager poll. Motors seen on without a request
    of ours (started active, or switched by hand) start the stiffness clock
    now — the conservative guess."""
    if not motors_enabled:
        activated_at = None
    elif state.activated_at is None:
        activated_at = now
    else:
        activated_at = state.activated_at
    return replace(state, motors_enabled=motors_enabled,
                   controller_active=controller_active, activated_at=activated_at)


def stiffness_ready(state: ArmPower, now: float, ramp_secs: float) -> bool:
    return (state.motors_enabled and state.activated_at is not None
            and now - state.activated_at >= ramp_secs)


def can_move(state: ArmPower, now: float, ramp_secs: float) -> bool:
    return state.controller_active and stiffness_ready(state, now, ramp_secs)


def request_started(state: ArmPower) -> ArmPower:
    return replace(state, busy=True, last_error='')


def after_power_on(state: ArmPower, ok: bool, now: float, error: str = '') -> ArmPower:
    if not ok:
        return replace(state, busy=False, last_error=error or 'activation failed')
    return replace(state, busy=False, motors_enabled=True, activated_at=now, last_error='')


def after_release(state: ArmPower, ok: bool, error: str = '') -> ArmPower:
    if not ok:
        return replace(state, busy=False, last_error=error or 'release failed')
    return replace(state, busy=False, motors_enabled=False, controller_active=False,
                   activated_at=None, last_error='')


def release_refusal(state: ArmPower, positions: dict, velocities: dict, park: dict,
                    pose_tol: float, velocity_tol: float) -> str | None:
    """Why the motors must not be turned off now, or None if they may."""
    if state.busy:
        return 'a power request is already in progress'
    if not state.motors_enabled:
        return None   # already off: nothing to release
    if not park:
        return 'no park pose configured (poses.json "park")'
    blockers = release_blockers(positions, velocities, park, pose_tol, velocity_tol)
    return '; '.join(blockers) if blockers else None


def in_park_pose(positions: dict, park: dict, pose_tol: float) -> bool:
    return bool(park) and not release_blockers(positions, {}, park, pose_tol, float('inf'))
