"""Park / Release: the joint targets and the release gate, no ROS.

Park lays the arm on its rest support (poses.json "park", recorded with the
arm lying there limp): from far away a collision-checked MoveIt move to just
above it (shoulder and elbow lifted a little), then a short direct JTC
trajectory down. The lowering joints are aimed press_rad past the recorded
rest, so the arm presses lightly on the support and the support, not the
motors, carries its weight.

Release then turns the motors off; ArmCanSystem ramps the stiffness down and
refuses (restoring it) if a joint moves meanwhile. The gate here only keeps
the operator from asking for that anywhere but in the park pose, at rest.
"""


def park_approach(park: dict, lifting: dict, lift_rad: float) -> dict:
    """Just above the park pose: ``lifting`` maps a joint to the sign that
    lifts it off the support; those joints are raised by ``lift_rad``."""
    return {name: value + lifting.get(name, 0.0) * lift_rad for name, value in park.items()}


def park_targets(park: dict, lowering: dict, press_rad: float) -> dict:
    """Final park targets: ``lowering`` maps a joint to the sign that moves it
    down onto the support; those joints go ``press_rad`` further that way."""
    return {name: value + lowering.get(name, 0.0) * press_rad for name, value in park.items()}


def max_deviation(current: dict, target: dict) -> tuple:
    """(joint, |error|) of the joint furthest from ``target``; a joint missing
    from ``current`` counts as infinitely far."""
    worst = (None, 0.0)
    for name, value in target.items():
        error = abs(current[name] - value) if name in current else float('inf')
        if error > worst[1]:
            worst = (name, error)
    return worst


def release_blockers(current: dict, velocities: dict, park: dict,
                     pose_tol: float, velocity_tol: float) -> list:
    """Reasons the arm must not be released now; empty when it may."""
    reasons = []
    joint, error = max_deviation(current, park)
    if error > pose_tol:
        reasons.append(f'not in the park pose ({joint} is {error:.3f} rad off, '
                       f'tolerance {pose_tol:.3f}) - park first')
    moving = {name: round(v, 3) for name, v in velocities.items()
              if name in park and abs(v) > velocity_tol}
    if moving:
        reasons.append(f'joints still moving: {moving}')
    return reasons
