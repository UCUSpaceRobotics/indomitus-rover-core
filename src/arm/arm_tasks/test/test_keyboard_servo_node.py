"""Gripper sync/gating/limits, safe-pose gripper reset, and panel-align memory.

Internal methods (_on_joint_state, _publish_gripper, _handle_safe_pose,
_check_panel_visibility, ...) are called directly rather than round-tripped
through real pub/sub or a running Gazebo/controller_manager, since the
behavior under test lives entirely in this node's own state machine —
feeding it through the real /joint_states topic would also reintroduce a
genuine flakiness trap found while verifying this by hand: against a busy
topic (e.g. a live sim publishing at 100Hz), rclpy.spin_once can starve
this node's own publish timer indefinitely and report a false failure.
"""
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, Joy

from arm_tasks.keyboard_servo_node import (
    GRIPPER_JOINT_NAME,
    HOME_POSE_JOINTS,
    GamepadInputLoop,
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


def make_arm_joint_state(values):
    """A /joint_states message covering all 6 HOME_POSE_JOINTS."""
    msg = JointState()
    msg.name = list(HOME_POSE_JOINTS)
    msg.position = list(values)
    return msg


def make_joy(axes=None, buttons=None):
    msg = Joy()
    # index: 0/1 left stick, 2/3 right stick, 4/5 L2/R2; wide enough to
    # cover the panel buttons (11/12) without every caller specifying them.
    msg.axes = axes if axes is not None else [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
    msg.buttons = buttons if buttons is not None else [0] * 13
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


class _FakeFuture:
    """Stand-in for an rclpy Future that resolves synchronously."""

    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


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


# ── panel visibility / remembered-position state ────────────────────────

def test_is_panel_visible_false_initially(controller):
    assert controller.is_panel_visible() is False


def test_is_panel_visible_true_after_pose(controller):
    controller._on_panel_pose(PoseStamped())
    assert controller.is_panel_visible() is True


def test_is_panel_visible_false_once_stale(controller):
    controller._on_panel_pose(PoseStamped())
    controller._last_panel_visible_time = (
        controller.get_clock().now()
        - Duration(seconds=controller._panel_visible_max_age_sec + 1.0)
    )
    assert controller.is_panel_visible() is False


def test_has_remembered_panel_position_reflects_state(controller):
    assert controller.has_remembered_panel_position is False
    controller._panel_target_positions = [0.0] * len(HOME_POSE_JOINTS)
    assert controller.has_remembered_panel_position is True


# ── align_to_panel(): the three cases ────────────────────────────────────

def test_align_fails_without_calling_service_when_nothing_known(controller, monkeypatch):
    service_checked = []
    monkeypatch.setattr(
        controller._panel_align_client, 'wait_for_service',
        lambda timeout_sec=0: service_checked.append(True) or True,
    )
    assert controller.align_to_panel() is False
    assert service_checked == []  # no memory, not visible: never even asked


def test_align_replays_remembered_position_without_calling_service(controller, monkeypatch):
    controller._panel_target_positions = [0.1] * len(HOME_POSE_JOINTS)
    service_checked = []
    monkeypatch.setattr(
        controller._panel_align_client, 'wait_for_service',
        lambda timeout_sec=0: service_checked.append(True) or True,
    )
    monkeypatch.setattr(controller, 'stop_servo', lambda: True)
    monkeypatch.setattr(controller, 'use_trajectory_controller', lambda: True)
    moved = []
    monkeypatch.setattr(
        controller, '_move_to_joint_positions',
        lambda positions, label: moved.append((positions, label)) or True,
    )

    assert controller.align_to_panel() is True
    assert service_checked == []
    assert moved == [([0.1] * len(HOME_POSE_JOINTS), 'remembered panel position')]


def test_align_replays_remembered_position_even_when_panel_not_visible(controller, monkeypatch):
    controller._panel_target_positions = [0.2] * len(HOME_POSE_JOINTS)
    monkeypatch.setattr(controller, 'stop_servo', lambda: True)
    monkeypatch.setattr(controller, 'use_trajectory_controller', lambda: True)
    monkeypatch.setattr(controller, '_move_to_joint_positions', lambda p, l: True)

    assert controller.is_panel_visible() is False
    assert controller.align_to_panel() is True


def test_align_calls_service_and_learns_position_on_first_success(controller, monkeypatch):
    controller._on_panel_pose(PoseStamped())
    controller._on_joint_state(make_arm_joint_state([0.3] * len(HOME_POSE_JOINTS)))

    monkeypatch.setattr(controller._panel_align_client, 'wait_for_service', lambda timeout_sec=0: True)

    class _Result:
        success = True
        message = 'aligned'

    monkeypatch.setattr(
        controller._panel_align_client, 'call_async', lambda req: _FakeFuture(_Result())
    )

    assert controller.align_to_panel() is True
    assert controller._panel_target_positions == [0.3] * len(HOME_POSE_JOINTS)


def test_align_does_not_learn_position_on_service_failure(controller, monkeypatch):
    controller._on_panel_pose(PoseStamped())
    controller._on_joint_state(make_arm_joint_state([0.3] * len(HOME_POSE_JOINTS)))

    monkeypatch.setattr(controller._panel_align_client, 'wait_for_service', lambda timeout_sec=0: True)

    class _Result:
        success = False
        message = 'planning failed'

    monkeypatch.setattr(
        controller._panel_align_client, 'call_async', lambda req: _FakeFuture(_Result())
    )

    assert controller.align_to_panel() is False
    assert controller._panel_target_positions is None


# ── keyboard: panel prompt / align handlers ──────────────────────────────

def test_keyboard_check_panel_visibility_prompts_on_rising_edge(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'is_panel_visible', lambda: True)
    monkeypatch.setattr(controller, 'stop', lambda: None)
    loop._check_panel_visibility()
    assert loop._panel_prompt_pending is True
    assert loop._panel_was_visible is True


def test_keyboard_handle_panel_align_resumes_servo_regardless_of_outcome(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'align_to_panel', lambda: False)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True))
    loop._handle_panel_align()
    assert started == [True]


# ── safe-pose gating (keyboard) ─────────────────────────────────────────

def test_keyboard_starts_locked_out(controller):
    loop = KeyboardInputLoop(controller)
    assert loop._servo_started is False


def test_keyboard_safe_pose_starts_servo_only_on_success(controller, monkeypatch):
    loop = KeyboardInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    started = []
    monkeypatch.setattr(controller, 'start_servo', lambda: started.append(True) or True)
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


# ── gamepad: panel align button ─────────────────────────────────────────

def test_gamepad_panel_align_button_replays_remembered_position(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: True)
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    loop._handle_safe_pose()  # unlocks teleop

    controller._panel_target_positions = [0.0] * len(HOME_POSE_JOINTS)
    aligned = []
    monkeypatch.setattr(loop, '_handle_panel_align', lambda: aligned.append(True))

    loop._on_joy(make_joy())  # establishes a button baseline
    buttons = [0] * 13
    buttons[loop.BUTTON_PANEL_ALIGN] = 1
    loop._on_joy(make_joy(buttons=buttons))

    assert aligned == [True]


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
    monkeypatch.setattr(controller, 'start_servo', lambda: True)
    loop._handle_safe_pose()
    assert loop._teleop_locked is False


def test_gamepad_safe_pose_stays_locked_on_failure(controller, monkeypatch):
    loop = GamepadInputLoop(controller)
    monkeypatch.setattr(controller, 'move_to_safe_pose', lambda: False)
    loop._handle_safe_pose()
    assert loop._teleop_locked is True


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
