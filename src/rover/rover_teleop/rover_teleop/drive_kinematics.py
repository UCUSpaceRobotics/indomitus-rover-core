#!/usr/bin/env python3
"""Sticks to a twist, in each of the three drive modes.

The rover is a swerve platform, so the same two sticks can mean three
different things depending on the mode the operator picked, and two of the
three need a token non-zero command to hold a wheel shape without driving.
That is a lot of arithmetic to only ever exercise by holding a real gamepad,
hence its own module.

No ROS import anywhere in here — drive_kinematics is deliberately standalone.
"""

import math
from dataclasses import dataclass


#: Below this the rover counts as parked, and steering has to be commanded by
#: the angle-probe branch rather than by scaling a speed that is already zero.
PARKED_SPEED = 1e-3


@dataclass(frozen=True)
class JoyInput:
    """One /joy sample, deadzoned and scaled."""

    #: Forward/back, already multiplied by scale_linear.x.
    vx: float = 0.0
    #: Strafe, already multiplied by scale_linear.y. Ignored unless vy_enabled.
    vy: float = 0.0
    #: Yaw rate, already multiplied by scale_angular.yaw. Raw-twist mode only.
    wz: float = 0.0
    #: The same right-stick axis, left unscaled: in curvature mode it sets a
    #: radius rather than a yaw rate, so scale_angular does not apply to it.
    steer: float = 0.0
    #: L2 minus R2, deadzoned.
    rot: float = 0.0
    #: Whether either trigger is pulled, which is not the same as rot != 0.
    triggers_held: bool = False


@dataclass(frozen=True)
class DriveModes:
    """The operator's latched choices about their own output."""

    #: True = sticks are a twist directly. False = the right stick is curvature.
    raw_twist: bool = True
    #: Whether the rover is allowed to strafe.
    vy_enabled: bool = False
    #: Everything scaled down for close work.
    granny: bool = False


@dataclass(frozen=True)
class KinematicsParams:
    scale_rotate: float = 1.0
    #: Token yaw rate that holds the spin-in-place wheel shape while both
    #: triggers are held but balanced.
    rot_probe_wz: float = 1e-5
    #: Tightest turn the right stick can ask for, as curvature 1/R at full
    #: deflection. 2.0 means R = 0.5 m, an ICR inside the wheelbase.
    max_curvature: float = 2.0
    #: Token speed used to command a wheel angle while standing still.
    angle_probe_speed: float = 1e-5
    granny_scale: float = 0.1


def swerve_wz_correction(vx: float, wz: float) -> float:
    """Invert wz when the rover is moving backward relative to its heading.

    Without this the right stick steers the wrong way in reverse, because the
    yaw rate that turns the rover left going forward turns it right going back.
    """
    return -wz if vx < -PARKED_SPEED else wz


def twist_from_input(
    inputs: JoyInput,
    modes: DriveModes,
    params: KinematicsParams,
) -> tuple[float, float, float]:
    """Resolve one sample to (vx, vy, wz)."""
    vx = inputs.vx
    # Strafe is gated here rather than at the stick, so that everything below —
    # including the curvature branch's speed magnitude — sees the same zero.
    vy = inputs.vy if modes.vy_enabled else 0.0

    if modes.raw_twist:
        wz = inputs.wz
        if not modes.vy_enabled:
            # With strafing on, the operator is placing the body directly and
            # the reverse correction would fight them.
            wz = swerve_wz_correction(vx, wz)

    elif inputs.triggers_held or inputs.rot != 0.0:
        # Spin in place: the sticks are ignored entirely.
        vx = 0.0
        vy = 0.0
        wz = inputs.rot * params.scale_rotate

        if wz == 0.0:
            # Held to a draw. Keep the spin shape without driving: below
            # park_speed the controller steers with the drives at zero, and a
            # tiny — rather than empty — twist stops idle homing.
            wz = params.rot_probe_wz

    else:
        # Curvature: the right stick asks for a radius, and the yaw rate that
        # produces it depends on how fast the rover is going.
        target_curvature = inputs.steer * params.max_curvature

        v_total = math.hypot(vx, vy)
        v_signed = -v_total if vx < 0.0 else v_total

        if v_total < PARKED_SPEED and target_curvature != 0.0:
            # Parked but steering: scaling a zero speed by a curvature gives
            # zero, so drive the wheels to the angle with a token speed instead.
            vx = params.angle_probe_speed
            vy = 0.0
            wz = vx * target_curvature
        else:
            wz = v_signed * target_curvature

    if modes.granny:
        vx *= params.granny_scale
        vy *= params.granny_scale
        wz *= params.granny_scale

    return vx, vy, wz
