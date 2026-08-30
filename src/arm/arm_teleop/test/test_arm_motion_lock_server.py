"""arm_motion_lock_server.py's own acquire/release arbitration — a
lease-based state machine, tested directly against its internal
_on_acquire/_on_release handlers (same rationale as this repo's other
internal-method tests: no real service transport needed to exercise the
logic that actually matters).
"""
import pytest

from arm_teleop.arm_motion_lock_server import ArmMotionLockServer
from indomitus_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock


@pytest.fixture
def server():
    n = ArmMotionLockServer()
    yield n
    n.destroy_node()


def _acquire(server, holder_id, lease_sec):
    return server._on_acquire(
        AcquireArmMotionLock.Request(holder_id=holder_id, lease_sec=lease_sec),
        AcquireArmMotionLock.Response())


def _release(server, holder_id):
    return server._on_release(
        ReleaseArmMotionLock.Request(holder_id=holder_id), ReleaseArmMotionLock.Response())


def test_first_acquire_is_granted(server):
    assert _acquire(server, 'a', 30.0).granted is True


def test_second_holder_is_rejected_while_lease_active(server):
    _acquire(server, 'a', 30.0)
    resp = _acquire(server, 'b', 30.0)
    assert resp.granted is False
    assert 'a' in resp.message


def test_same_holder_can_renew_its_own_lease(server):
    _acquire(server, 'a', 30.0)
    assert _acquire(server, 'a', 30.0).granted is True


def test_acquire_succeeds_again_after_lease_expires(server, monkeypatch):
    _acquire(server, 'a', lease_sec=1.0)
    # An unreleased lease must expire on its own — this is the whole
    # point over the old flock (tied to a process's open fd forever).
    monkeypatch.setattr(
        'arm_teleop.arm_motion_lock_server.time.monotonic', lambda: 1_000_000.0)
    assert _acquire(server, 'b', 30.0).granted is True


def test_release_by_actual_holder_frees_the_lock(server):
    _acquire(server, 'a', 30.0)
    assert _release(server, 'a').success is True
    assert _acquire(server, 'b', 30.0).granted is True


def test_release_by_a_different_holder_is_a_harmless_no_op(server):
    _acquire(server, 'a', 30.0)
    assert _release(server, 'b').success is True
    # 'a' still holds it — a non-owning release must not free it.
    assert _acquire(server, 'c', 30.0).granted is False
