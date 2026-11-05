"""CommandState: when Servo is fed its last command, and when the real state."""

from arm_teleop.command_state import CommandState

JOINTS = ['a', 'b']


def _state():
    s = CommandState(JOINTS, max_lead_rad=0.1)
    s.set_forward_active(True)
    return s


def test_reports_last_command_for_arm_joints_only():
    s = _state()
    s.on_command([0.5, -0.2])
    assert s.resolve({'a': 0.45, 'b': -0.15, 'finger': 0.01}) == {'a': 0.5, 'b': -0.2, 'finger': 0.01}


def test_command_beyond_max_lead_falls_back_to_measured_and_stays_dropped():
    s = _state()
    s.on_command([0.5, 0.0])
    measured = {'a': 0.3, 'b': 0.0}
    assert s.resolve(measured) == measured
    assert s.last_drop_joint == 'a'
    # Within the limit again, but the dropped command must not come back.
    assert s.resolve({'a': 0.48, 'b': 0.0}) == {'a': 0.48, 'b': 0.0}


def test_forward_controller_inactive_passes_measured_through():
    s = _state()
    s.on_command([0.5, 0.0])
    s.set_forward_active(False)       # JTC took over (home / teach)
    s.set_forward_active(True)        # back to teleop: start from the real pose
    assert s.resolve({'a': 0.49, 'b': 0.0}) == {'a': 0.49, 'b': 0.0}
    s.set_forward_active(False)
    s.on_command([0.5, 0.0])          # commands while inactive are ignored
    assert not s.tracking
