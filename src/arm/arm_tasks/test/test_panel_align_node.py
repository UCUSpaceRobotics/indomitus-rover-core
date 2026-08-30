"""panel_align_node.py lifecycle/safety tests — the dangerous cases flagged
in review: hung execution + cancellation, cross-process motion exclusion,
and collision-checked remembered-position replay.

Internal methods are called/monkeypatched directly rather than round-tripped
through real action servers, mirroring test_keyboard_servo_node.py's own
rationale for the same choice.
"""
import rclpy
import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_teleop.arm_motion_lock import ArmMotionBusy
from arm_tasks.panel_align_node import JOINT_LIMITS, PanelAlignNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = PanelAlignNode()
    yield n
    n.destroy_node()


class _FakeFuture:
    """Stand-in for an rclpy Future that resolves synchronously."""

    def __init__(self, result=None):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


class _NeverResolvingFuture:
    """Simulates a goal handle whose result callback never arrives."""

    def add_done_callback(self, cb):
        pass


class _FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0

    def get_result_async(self):
        return _NeverResolvingFuture()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _FakeFuture(None)


def _valid_joint_positions():
    """Mid-range for every joint — well clear of any limit."""
    return {name: (lo + hi) / 2 for name, (lo, hi) in JOINT_LIMITS.items()}


def _fake_plan_result(joint_positions: dict):
    class _Result:
        error_code = MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)
        planned_trajectory = type('T', (), {'joint_trajectory': JointTrajectory(
            joint_names=list(joint_positions.keys()),
            points=[JointTrajectoryPoint(positions=list(joint_positions.values()))],
        )})()
    return _Result()


# ── Issue 1: hung execution must be cancelled, not left running ─────────

def test_execute_cancels_goal_and_waits_on_timeout(node, monkeypatch):
    node.set_parameters([Parameter('execution_timeout_sec', value=0.05)])
    monkeypatch.setattr(node._execute_client, 'wait_for_server', lambda timeout_sec=0: True)
    gh = _FakeGoalHandle(accepted=True)
    monkeypatch.setattr(node._execute_client, 'send_goal_async', lambda goal: _FakeFuture(gh))

    ok, reason = node._execute(RobotTrajectory(joint_trajectory=JointTrajectory()))

    assert ok is False
    assert 'cancelled' in reason
    assert gh.cancel_calls == 1


def test_execute_does_not_cancel_a_goal_that_was_never_accepted(node, monkeypatch):
    node.set_parameters([Parameter('execution_timeout_sec', value=0.05)])
    monkeypatch.setattr(node._execute_client, 'wait_for_server', lambda timeout_sec=0: True)
    gh = _FakeGoalHandle(accepted=False)
    monkeypatch.setattr(node._execute_client, 'send_goal_async', lambda goal: _FakeFuture(gh))

    ok, reason = node._execute(RobotTrajectory(joint_trajectory=JointTrajectory()))

    assert ok is False
    assert gh.cancel_calls == 0  # nothing to cancel — rejected immediately


# ── Issue 2: cross-process exclusion surfaces as a clean failure ────────

def test_align_to_panel_fails_cleanly_when_arm_motion_busy(node, monkeypatch):
    def _raise_busy(*a, **kw):
        raise ArmMotionBusy('busy')
    monkeypatch.setattr('arm_tasks.panel_align_node.arm_motion_lock', _raise_busy)

    assert node.align_to_panel() is False
    assert 'keyboard_servo_node' in node._last_status_message
    # The in-process lock must be released even when the cross-process
    # one raises, or a SECOND call would wrongly report "already in
    # progress" instead of the real busy reason.
    assert node._align_running.acquire(blocking=False)
    node._align_running.release()


# ── align_to_panel() dispatch: remembered target always wins ────────────

def test_dispatches_to_live_align_when_nothing_remembered(node, monkeypatch):
    called = []
    monkeypatch.setattr('arm_tasks.panel_align_node.arm_motion_lock', lambda: _NullContext())
    monkeypatch.setattr(node, '_align_live_locked', lambda: called.append('live') or True)
    monkeypatch.setattr(node, '_align_to_remembered_locked', lambda: called.append('remembered') or True)

    assert node.align_to_panel() is True
    assert called == ['live']


def test_dispatches_to_remembered_replay_once_learned(node, monkeypatch):
    node._remembered_target_joints = _valid_joint_positions()
    monkeypatch.setattr('arm_tasks.panel_align_node.arm_motion_lock', lambda: _NullContext())
    called = []
    monkeypatch.setattr(node, '_align_live_locked', lambda: called.append('live') or True)
    monkeypatch.setattr(node, '_align_to_remembered_locked', lambda: called.append('remembered') or True)

    assert node.align_to_panel() is True
    assert called == ['remembered']


# ── Issue 3: remembered replay is collision-checked, not a raw move ─────

def test_remembered_replay_applies_collision_object_and_uses_joint_constraints(node, monkeypatch):
    target = _valid_joint_positions()
    node._remembered_target_joints = target
    node._remembered_panel_collision_pose = PoseStamped()

    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: None)  # panel not visible now

    collision_calls = []
    monkeypatch.setattr(
        node, '_apply_panel_collision_object', lambda pose: collision_calls.append(pose) or True)

    plan_calls = []

    def _fake_request_plan(goal_constraints):
        plan_calls.append(goal_constraints)
        return _fake_plan_result(target)
    monkeypatch.setattr(node, '_request_plan', _fake_request_plan)
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._align_to_remembered_locked() is True
    # Collision-checked: the panel's CollisionObject was inserted before
    # planning, using the CACHED pose since live detection wasn't available —
    # this is the actual fix for "replay bypasses collision checking".
    assert collision_calls == [node._remembered_panel_collision_pose]
    # Joint-space, not Cartesian: exact final pose guaranteed regardless
    # of the path OMPL finds to get there.
    assert len(plan_calls) == 1
    assert plan_calls[0].joint_constraints
    assert not plan_calls[0].position_constraints
    assert not plan_calls[0].orientation_constraints
    got = {jc.joint_name: jc.position for jc in plan_calls[0].joint_constraints}
    assert got == target


def test_remembered_replay_prefers_live_panel_pose_when_available(node, monkeypatch):
    node._remembered_target_joints = _valid_joint_positions()
    node._remembered_panel_collision_pose = PoseStamped()
    live_pose = PoseStamped()

    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: live_pose)

    collision_calls = []
    monkeypatch.setattr(
        node, '_apply_panel_collision_object', lambda pose: collision_calls.append(pose) or True)
    monkeypatch.setattr(node, '_request_plan', lambda gc: _fake_plan_result(node._remembered_target_joints))
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._align_to_remembered_locked() is True
    assert collision_calls == [live_pose]  # live pose preferred over the cached one


def test_remembered_replay_fails_on_joint_limit_margin(node, monkeypatch):
    # Every joint pinned exactly at its lower limit — 0% margin.
    tight = {name: lo for name, (lo, hi) in JOINT_LIMITS.items()}
    node._remembered_target_joints = tight
    node._remembered_panel_collision_pose = PoseStamped()

    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_current_live_panel_collision_pose', lambda: None)
    monkeypatch.setattr(node, '_apply_panel_collision_object', lambda pose: True)
    monkeypatch.setattr(node, '_request_plan', lambda gc: _fake_plan_result(tight))
    executed = []
    monkeypatch.setattr(node, '_execute', lambda traj: executed.append(True) or (True, ''))

    assert node._align_to_remembered_locked() is False
    assert executed == []  # never even tried to move — rejected before execution


# ── orient_gripper_to_remembered() ('m'): tighter tolerance, no replan ──

def _fake_tf(x, y, z):
    class _T:
        transform = type('X', (), {'translation': type('V', (), {'x': x, 'y': y, 'z': z})()})()
    return _T()


def test_orient_gripper_fails_without_remembered_orientation(node, monkeypatch):
    node._remembered_target_orientation = None
    stop_calls = []
    monkeypatch.setattr(node, '_call_stop_servo', lambda: stop_calls.append(True) or True)

    assert node._orient_gripper_to_remembered_locked() is False
    assert stop_calls == []  # bailed out before touching the arm at all


def test_orient_gripper_uses_tighter_tolerance_and_current_position(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(1.0, 2.0, 3.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)

    plan_calls = []

    def _fake_request_plan(goal_constraints):
        plan_calls.append(goal_constraints)
        return _fake_plan_result(_valid_joint_positions())
    monkeypatch.setattr(node, '_request_plan', _fake_request_plan)
    monkeypatch.setattr(node, '_execute', lambda traj: (True, ''))

    assert node._orient_gripper_to_remembered_locked() is True
    assert len(plan_calls) == 1
    pc = plan_calls[0].position_constraints[0]
    assert (pc.constraint_region.primitive_poses[0].position.x,
            pc.constraint_region.primitive_poses[0].position.y,
            pc.constraint_region.primitive_poses[0].position.z) == (1.0, 2.0, 3.0)
    oc = plan_calls[0].orientation_constraints[0]
    expected = node.get_parameter('orient_gripper_orientation_tolerance').value
    assert oc.absolute_x_axis_tolerance == expected
    assert expected < node.get_parameter('orientation_tolerance').value


def test_orient_gripper_fails_cleanly_on_plan_failure(node, monkeypatch):
    node._remembered_target_orientation = (0.0, 0.0, 0.0, 1.0)
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lambda *a, **kw: _fake_tf(0.0, 0.0, 0.0))
    monkeypatch.setattr(node, '_call_stop_servo', lambda: True)
    monkeypatch.setattr(node, '_use_trajectory_controller', lambda: True)
    monkeypatch.setattr(node, '_request_plan', lambda gc: None)
    executed = []
    monkeypatch.setattr(node, '_execute', lambda traj: executed.append(True) or (True, ''))

    assert node._orient_gripper_to_remembered_locked() is False
    assert executed == []


class _NullContext:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False
