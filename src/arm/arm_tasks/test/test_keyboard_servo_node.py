"""Gripper sync/gating/limits and safe-pose gripper reset.

Internal methods (_on_joint_state, _publish_gripper, _handle_safe_pose) are
called directly rather than round-tripped through real pub/sub or a running
Gazebo/controller_manager, since the behavior under test lives entirely in
this node's own state machine — feeding it through the real /joint_states
topic would also reintroduce a genuine flakiness trap found while verifying
this by hand: against a busy topic (e.g. a live sim publishing at 100Hz),
rclpy.spin_once can starve this node's own publish timer indefinitely and
report a false failure.
"""
import time

import pytest
import rclpy
from sensor_msgs.msg import JointState

from arm_tasks.keyboard_servo_node import (
    GRIPPER_JOINT_NAME,
    KeyboardInputLoop,
    ServoController,
    ecodes,
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def controller():
    node = ServoController()
    yield node
    node.destroy_node()


def make_joint_state(position, name=GRIPPER_JOINT_NAME):
    msg = JointState()
    msg.name = [name]
    msg.position = [position]
    return msg


def publish_ticks(controller, n=5, dt=0.05):
    """Call _publish_gripper() n times with a fixed, simulated tick interval.

    The very first call after a velocity is set never moves anything (no
    prior tick to diff a dt against — see _publish_gripper), so this backdates
    _last_gripper_tick_time by exactly ``dt`` before every later call instead
    of relying on the real (near-zero, runner-speed-dependent) wall-clock gap
    between back-to-back calls in a tight loop — otherwise this is flaky:
    locally the gap may happen to accumulate enough dt to reach the clamp,
    but on a faster/slower CI runner it may not.
    """
    for _ in range(n):
        if controller._last_gripper_tick_time is not None:
            controller._last_gripper_tick_time = time.monotonic() - dt
        controller._publish_gripper()


# ── gripper sync (restart while partially/fully open) ──────────────────────

def test_gripper_starts_unsynced(controller):
    assert controller._gripper_state_received is False
    assert controller._gripper_position == 0.0


def test_gripper_syncs_from_first_joint_state(controller):
    controller._on_joint_state(make_joint_state(0.008))
    assert controller._gripper_state_received is True
    assert controller._gripper_position == pytest.approx(0.008)


def test_gripper_ignores_state_after_first_sync(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller._gripper_position = 0.003  # simulate an in-progress move
    controller._on_joint_state(make_joint_state(0.011))
    assert controller._gripper_position == pytest.approx(0.003)


def test_gripper_state_without_gripper_joint_is_ignored(controller):
    controller._on_joint_state(make_joint_state(1.0, name='some_other_joint'))
    assert controller._gripper_state_received is False


# ── commands withheld before sync (the restart-mid-open scenario) ──────────

def test_gripper_commands_withheld_before_sync(controller, monkeypatch):
    right_seen = []
    left_seen = []
    monkeypatch.setattr(controller._gripper_right_pub, 'publish', right_seen.append)
    monkeypatch.setattr(controller._gripper_left_pub, 'publish', left_seen.append)

    controller.set_gripper_velocity(-0.006)
    publish_ticks(controller)

    assert right_seen == []
    assert left_seen == []
    assert controller._gripper_position == 0.0  # never moved: no sync yet


def test_gripper_publishes_once_synced(controller, monkeypatch):
    right_seen = []
    monkeypatch.setattr(controller._gripper_right_pub, 'publish', right_seen.append)

    controller._on_joint_state(make_joint_state(0.008))
    controller._publish_gripper()

    assert right_seen


def test_gripper_first_tick_after_sync_holds_instead_of_jumping(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller.set_gripper_velocity(-0.006)
    controller._publish_gripper()
    # No prior tick to diff a dt against yet, so this tick only records the
    # timestamp — it does not guess a dt and move (that would risk a jump
    # from the synced value on a slow first tick).
    assert controller._gripper_position == pytest.approx(0.008)


def test_gripper_moves_gradually_from_synced_value_not_from_zero(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller.set_gripper_velocity(-0.006)
    controller._publish_gripper()  # establishes _last_gripper_tick_time
    controller._last_gripper_tick_time = time.monotonic() - 0.1
    controller._publish_gripper()  # dt ~= 0.1s
    assert controller._gripper_position == pytest.approx(0.008 - 0.006 * 0.1, abs=1e-3)


# ── limits ──────────────────────────────────────────────────────────────

def test_gripper_position_clamps_to_stroke_bounds(controller):
    controller._on_joint_state(make_joint_state(0.006))
    controller.set_gripper_velocity(-10.0)  # absurdly fast close
    publish_ticks(controller)
    assert controller._gripper_position == 0.0

    controller.set_gripper_velocity(10.0)  # absurdly fast open
    publish_ticks(controller)
    assert controller._gripper_position == pytest.approx(controller._gripper_stroke)


def test_gripper_stroke_is_configurable(controller):
    controller._gripper_stroke = 0.02  # simulate a different bringup's parameter
    controller._on_joint_state(make_joint_state(0.0))
    controller.set_gripper_velocity(10.0)
    publish_ticks(controller)
    assert controller._gripper_position == pytest.approx(0.02)


def test_gripper_left_command_is_negated_right(controller, monkeypatch):
    right_seen = []
    left_seen = []
    monkeypatch.setattr(controller._gripper_right_pub, 'publish', right_seen.append)
    monkeypatch.setattr(controller._gripper_left_pub, 'publish', left_seen.append)

    controller._on_joint_state(make_joint_state(0.006))
    controller._publish_gripper()

    assert right_seen and left_seen
    assert list(left_seen[0].data) == [-right_seen[0].data[0]]


# ── keyboard: gripper key combination -> velocity ───────────────────────

def test_keyboard_open_key_sets_positive_velocity(controller):
    loop = KeyboardInputLoop(controller)
    loop._gripper_pressed.add(ecodes.KEY_B)
    loop._recompute_gripper_velocity()
    assert controller.gripper_vel == pytest.approx(controller.gripper_speed)


def test_keyboard_close_key_sets_negative_velocity(controller):
    loop = KeyboardInputLoop(controller)
    loop._gripper_pressed.add(ecodes.KEY_V)
    loop._recompute_gripper_velocity()
    assert controller.gripper_vel == pytest.approx(-controller.gripper_speed)


def test_keyboard_both_gripper_keys_cancel_out(controller):
    loop = KeyboardInputLoop(controller)
    loop._gripper_pressed.add(ecodes.KEY_B)
    loop._gripper_pressed.add(ecodes.KEY_V)
    loop._recompute_gripper_velocity()
    assert controller.gripper_vel == 0.0


def test_stop_zeroes_gripper_velocity(controller):
    controller.set_gripper_velocity(0.7)
    controller.stop()
    assert controller.gripper_vel == 0.0


def test_keyboard_safe_pose_clears_gripper_state(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    controller.set_gripper_velocity(0.5)
    loop._gripper_pressed.add(30)  # arbitrary key code standing in for 'b'/'v'

    loop._handle_safe_pose()

    assert loop._gripper_pressed == set()
    assert controller.gripper_vel == 0.0
