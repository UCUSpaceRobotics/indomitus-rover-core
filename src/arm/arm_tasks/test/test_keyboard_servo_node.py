"""Gripper sync/gating/limits, safe-pose lockout, and joy timeout.

Internal methods (_on_joint_state, _publish, _handle_safe_pose, _on_joy,
_check_joy_timeout) are called directly rather than round-tripped through
real pub/sub or a running Gazebo/controller_manager, since the behavior
under test lives entirely in this node's own state machine — feeding it
through the real /joint_states topic would also reintroduce a genuine
flakiness trap found while verifying this by hand: against a busy topic
(e.g. a live sim publishing at 100Hz), rclpy.spin_once can starve this
node's own publish timer indefinitely and report a false failure.
"""
import pytest
import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, Joy

from arm_tasks.keyboard_servo_node import (
    GamepadInputLoop,
    GRIPPER_JOINT_NAME,
    KeyboardInputLoop,
    ROLL_JOINT_NAME,
    ServoController,
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


def make_joy(axes=None, buttons=None):
    msg = Joy()
    # index: 0/1 left stick, 2/3 right stick, 4/5 L2/R2 (rest at +1.0)
    msg.axes = axes if axes is not None else [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
    msg.buttons = buttons if buttons is not None else [0] * 8
    return msg


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

def test_gripper_commands_withheld_before_sync(controller):
    controller.set_velocity(gripper_vel=-0.006)
    for _ in range(5):
        controller._publish()
    assert controller._gripper_position == 0.0  # never moved: no sync yet


def test_gripper_moves_gradually_from_synced_value_not_from_zero(controller):
    controller._on_joint_state(make_joint_state(0.008))
    controller.set_velocity(gripper_vel=-0.006)
    controller._publish()
    # First tick after sync has no measured interval yet, so it falls back
    # to one nominal tick (1/publish_rate) — a small step, not a jump to 0.
    assert controller._gripper_position < 0.008
    assert controller._gripper_position > 0.008 - 0.003


# ── limits ──────────────────────────────────────────────────────────────

def test_gripper_position_clamps_to_stroke_bounds(controller):
    # _last_gripper_tick_time is reset before each _publish() call so every
    # tick uses the nominal 1/publish_rate fallback dt instead of the real
    # (near-zero, runner-speed-dependent) wall-clock gap between back-to-back
    # calls in this tight loop — otherwise this test is flaky: locally the
    # gap may happen to accumulate enough dt to reach the clamp, but on a
    # faster/slower CI runner it may not.
    controller._on_joint_state(make_joint_state(0.006))
    controller.set_velocity(gripper_vel=-10.0)  # absurdly fast close
    for _ in range(5):
        controller._last_gripper_tick_time = None
        controller._publish()
    assert controller._gripper_position == 0.0

    controller.set_velocity(gripper_vel=10.0)  # absurdly fast open
    for _ in range(5):
        controller._last_gripper_tick_time = None
        controller._publish()
    assert controller._gripper_position == pytest.approx(controller.gripper_stroke)


def test_gripper_stroke_is_configurable(controller):
    controller._gripper_stroke = 0.02  # simulate a different bringup's parameter
    controller._on_joint_state(make_joint_state(0.0))
    controller.set_velocity(gripper_vel=10.0)
    for _ in range(5):
        controller._publish()
    assert controller._gripper_position == pytest.approx(0.02)


# ── simultaneous inputs: roll + gripper together ────────────────────────

def test_roll_and_gripper_publish_independently_in_same_tick(controller, monkeypatch):
    controller._on_joint_state(make_joint_state(0.006))

    grip_seen = []
    monkeypatch.setattr(controller._gripper_pub, 'publish', grip_seen.append)
    jog_seen = []
    monkeypatch.setattr(controller._joint_jog_pub, 'publish', jog_seen.append)

    controller.set_velocity(wx=0.3, gripper_vel=-0.006)
    controller._publish()

    assert grip_seen, 'gripper command should still publish while roll is active'
    assert jog_seen and jog_seen[0].joint_names == [ROLL_JOINT_NAME]


# ── safe-pose gating (keyboard) ─────────────────────────────────────────

def test_keyboard_starts_locked_out(controller):
    loop = KeyboardInputLoop(controller)
    assert loop._servo_started is False


def test_keyboard_safe_pose_starts_servo_only_on_success(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True))
    loop._handle_safe_pose()
    assert loop._servo_started is True
    assert started == [True]


def test_keyboard_safe_pose_does_not_start_servo_on_failure(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: False)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True))
    loop._handle_safe_pose()
    assert loop._servo_started is False
    assert started == []


# ── safe-pose gating + joy timeout (gamepad) ────────────────────────────

def test_gamepad_starts_teleop_locked(controller):
    loop = GamepadInputLoop(controller)
    assert loop._teleop_locked is True


def test_gamepad_stick_input_ignored_while_locked(controller):
    loop = GamepadInputLoop(controller)
    controller.vx = 999.0  # sentinel: only stop() would clear this
    loop._on_joy(make_joy(axes=[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]))  # right stick forward
    assert controller.vx == 0.0


def test_gamepad_safe_pose_unlocks_teleop_on_success(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: None)
    loop._handle_safe_pose()
    assert loop._teleop_locked is False


def test_gamepad_safe_pose_stays_locked_on_failure(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: False)
    loop._handle_safe_pose()
    assert loop._teleop_locked is True


def test_gamepad_simultaneous_stick_and_gripper(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: None)
    loop._handle_safe_pose()  # unlocks teleop

    # Right stick forward (index 3) + L1 held (gripper close, button 4)
    loop._on_joy(make_joy(axes=[0.0, 0.0, 0.0, 1.0, 1.0, 1.0], buttons=[0, 0, 0, 0, 1, 0]))
    assert controller.vx != 0.0
    assert controller.gripper_vel < 0.0


def test_gamepad_joy_timeout_stops_the_arm(controller):
    loop = GamepadInputLoop(controller)
    controller.vx = 999.0
    loop._last_joy_time = controller.get_clock().now() - Duration(seconds=1.0)
    loop._check_joy_timeout()
    assert controller.vx == 0.0


def test_gamepad_no_timeout_when_joy_recently_seen(controller):
    loop = GamepadInputLoop(controller)
    controller.vx = 999.0
    loop._last_joy_time = controller.get_clock().now()
    loop._check_joy_timeout()
    assert controller.vx == 999.0  # untouched: no timeout yet
