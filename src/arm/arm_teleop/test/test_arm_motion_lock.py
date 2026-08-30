"""Cross-host arm-motion lock — client-side retry/fail-closed logic,
against fake acquire/release service clients (no real ROS graph needed
here; see test_arm_motion_lock_server.py for the server's own state
machine, and both together are the real cross-process guarantee this
used to get from a single multiprocess flock test).
"""
import pytest

from arm_teleop.arm_motion_lock import ArmMotionBusy, arm_motion_lock


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


def _acquire_response(granted, message=''):
    return type('AcquireResp', (), {'granted': granted, 'message': message})()


def _release_response(success=True, message=''):
    return type('ReleaseResp', (), {'success': success, 'message': message})()


class _FakeClient:
    """Stands in for the acquire/release rclpy service clients."""

    def __init__(self, responses=None, available=True):
        self._responses = list(responses) if responses is not None else []
        self._available = available
        self.requests = []

    def wait_for_service(self, timeout_sec=0):
        return self._available

    def call_async(self, request):
        self.requests.append(request)
        return _FakeFuture(self._responses.pop(0))


def test_acquire_and_release_happy_path():
    acquire = _FakeClient([_acquire_response(True)])
    release = _FakeClient([_release_response(True)])
    entered = []

    with arm_motion_lock(acquire, release, 'me', lease_sec=30.0):
        entered.append(True)

    assert entered == [True]
    assert acquire.requests[0].holder_id == 'me'
    assert acquire.requests[0].lease_sec == 30.0
    assert release.requests[0].holder_id == 'me'


def test_busy_raises_immediately_when_timeout_sec_is_zero():
    acquire = _FakeClient([_acquire_response(False, 'held by someone else')])
    release = _FakeClient([])

    with pytest.raises(ArmMotionBusy, match='held by someone else'):
        with arm_motion_lock(acquire, release, 'me', lease_sec=30.0):
            pass
    assert release.requests == []  # never acquired — nothing to release


def test_unreachable_server_fails_closed():
    """Never silently skip cross-host exclusion just because its own
    arbiter is unreachable — this is the whole point of the fix.
    """
    acquire = _FakeClient(available=False)
    release = _FakeClient()

    with pytest.raises(ArmMotionBusy, match='not reachable'):
        with arm_motion_lock(acquire, release, 'me', lease_sec=30.0):
            pass


def test_retries_until_granted_within_timeout(monkeypatch):
    acquire = _FakeClient([
        _acquire_response(False, 'busy'), _acquire_response(False, 'busy'),
        _acquire_response(True),
    ])
    release = _FakeClient([_release_response(True)])
    monkeypatch.setattr('arm_teleop.arm_motion_lock.time.sleep', lambda s: None)

    with arm_motion_lock(acquire, release, 'me', lease_sec=30.0, timeout_sec=5.0):
        pass
    assert len(acquire.requests) == 3


def test_gives_up_after_timeout_elapses(monkeypatch):
    acquire = _FakeClient([_acquire_response(False, 'busy')] * 10)
    release = _FakeClient([])
    # First monotonic() call sets the deadline; later ones report it as
    # already passed, without a real 1s sleep in this test.
    times = iter([0.0, 0.0, 10.0])
    monkeypatch.setattr(
        'arm_teleop.arm_motion_lock.time.monotonic', lambda: next(times, 10.0))
    monkeypatch.setattr('arm_teleop.arm_motion_lock.time.sleep', lambda s: None)

    with pytest.raises(ArmMotionBusy):
        with arm_motion_lock(acquire, release, 'me', lease_sec=30.0, timeout_sec=1.0):
            pass


def test_release_still_called_when_the_locked_block_raises():
    acquire = _FakeClient([_acquire_response(True)])
    release = _FakeClient([_release_response(True)])

    with pytest.raises(RuntimeError):
        with arm_motion_lock(acquire, release, 'me', lease_sec=30.0):
            raise RuntimeError('boom')
    assert release.requests[0].holder_id == 'me'
