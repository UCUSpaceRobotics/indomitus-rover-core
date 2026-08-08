"""
Tests for the pure logic in joystick_interpreter_node.

Everything here is deliberately free of rclpy: the node itself needs a running
context, but the sign conventions do not, and the sign conventions are what go
wrong silently.
"""

import pytest

from rover_teleop.joystick_interpreter_node import trigger_diff


L2 = 4
R2 = 5
DEADZONE = 0.15


def axes(l2: float = 0.0, r2: float = 0.0):
    """Build a Joy.axes array with the two triggers at the given deflections."""
    values = [0.0] * 6
    values[L2] = l2
    values[R2] = r2
    return values


def test_l2_spins_counter_clockwise():
    """
    L2 must produce a positive value.

    This feeds Twist.angular.z, and ROS yaw is positive counter-clockwise. A
    sign error here spins the rover the opposite way from what the driver
    asked, which is cheap to catch on a bench and expensive in the field.
    """
    assert trigger_diff(axes(l2=1.0), L2, R2, DEADZONE) == pytest.approx(1.0)


def test_r2_spins_clockwise():
    assert trigger_diff(axes(r2=1.0), L2, R2, DEADZONE) == pytest.approx(-1.0)


def test_partial_press_is_proportional():
    assert trigger_diff(axes(l2=0.6), L2, R2, DEADZONE) == pytest.approx(0.6)
    assert trigger_diff(axes(r2=0.6), L2, R2, DEADZONE) == pytest.approx(-0.6)


def test_both_triggers_cancel():
    assert trigger_diff(axes(l2=1.0, r2=1.0), L2, R2, DEADZONE) == 0.0


def test_shared_rest_offset_cancels():
    """
    A controller that rests its triggers off zero must not command a spin.

    The whole reason the value is a difference rather than either trigger on
    its own.
    """
    assert trigger_diff(axes(l2=-1.0, r2=-1.0), L2, R2, DEADZONE) == 0.0
    assert trigger_diff(axes(l2=0.3, r2=0.3), L2, R2, DEADZONE) == 0.0


def test_deadzone_suppresses_small_differences():
    assert trigger_diff(axes(l2=0.1), L2, R2, DEADZONE) == 0.0
    assert trigger_diff(axes(r2=0.1), L2, R2, DEADZONE) == 0.0
    assert trigger_diff(axes(l2=0.2), L2, R2, DEADZONE) == pytest.approx(0.2)


def test_missing_axes_are_treated_as_rest():
    """A joystick reporting fewer axes than configured must not crash or spin."""
    assert trigger_diff([0.0, 0.0], L2, R2, DEADZONE) == 0.0
    assert trigger_diff([], L2, R2, DEADZONE) == 0.0
    assert trigger_diff(axes(l2=1.0), L2, 99, DEADZONE) == pytest.approx(1.0)
