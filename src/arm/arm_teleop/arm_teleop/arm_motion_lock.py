"""Cross-host mutual exclusion for anything that submits a goal to
``indomitus_arm_controller`` (the JTC).

``keyboard_servo_node.py``'s home move / remembered-position replay and
``panel_align_node.py``'s live (CV+MoveIt) align are separate PROCESSES —
and, on the real rover, can run on separate HOSTS (operator control from
the ground station, planning/execution served by the Jetson) — each
perfectly happy to submit a FollowJointTrajectory-family goal to the same
action server. Confirmed live: pressing 'r' while a 'p' live align is
planning/executing let both through, racing two goals onto the same
controller (see PR review — this is what this module fixes). Each side
already has its own in-process ``threading.Lock`` guarding against
racing ITSELF (two 'r' presses, two 'p' presses); a plain
``threading.Lock`` only ever protects within the one process that
created it, and a ``flock`` (this module's original implementation) only
ever protects within the one HOST it runs on — neither covers GS
commanding the arm while Jetson-hosted code also does. A ROS service,
hosted once by ``arm_motion_lock_server.py`` alongside the controller it
actually arbitrates, is reachable from either host.
"""

from __future__ import annotations

import contextlib
import threading
import time

from indomitus_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock

# How long to wait for the lock server itself to answer a single acquire/
# release call — separate from lease_sec (how long the CALLER intends to
# hold the lock) and from timeout_sec (how long to keep retrying a busy
# lock below).
_SERVICE_WAIT_SEC = 2.0
_CALL_TIMEOUT_SEC = 3.0


class ArmMotionBusy(Exception):
    """Raised by arm_motion_lock() when the lock can't be acquired —
    either another holder has it, or the lock server itself couldn't be
    reached (fail closed: never silently skip cross-host exclusion just
    because its arbiter is unreachable).
    """


@contextlib.contextmanager
def arm_motion_lock(acquire_client, release_client, holder_id: str,
                     lease_sec: float, timeout_sec: float = 0.0):
    """Acquire the cross-host arm-motion lock for the ``with`` block.

    Args:
        acquire_client/release_client: this caller's own persistent
            rclpy service clients for arm_motion_lock/acquire and
            .../release (created once in __init__, same convention as
            every other client in these two node classes).
        holder_id: identifies this caller in busy-lock messages and lets
            a caller safely re-acquire/renew its own still-held lease.
        lease_sec: worst-case duration of the motion about to run —
            the server auto-expires the lock after this if release()
            never arrives (crash, network drop).
        timeout_sec: 0.0 (default) tries once and raises ArmMotionBusy
            immediately if busy, mirroring the in-process
            threading.Lock(blocking=False) pattern both callers already
            use for their own local lock. A positive value retries until
            it elapses before giving up the same way.

    Raises:
        ArmMotionBusy: the lock is held by someone else (or the lock
            server itself is unreachable) and timeout_sec elapsed (or
            was 0) without acquiring it.
    """
    deadline = time.monotonic() + timeout_sec
    while True:
        granted, message = _call_acquire(acquire_client, holder_id, lease_sec)
        if granted:
            break
        if time.monotonic() >= deadline:
            raise ArmMotionBusy(message)
        time.sleep(0.05)
    try:
        yield
    finally:
        _call_release(release_client, holder_id)


def _call_acquire(client, holder_id: str, lease_sec: float) -> tuple[bool, str]:
    if not client.wait_for_service(timeout_sec=_SERVICE_WAIT_SEC):
        return False, 'arm_motion_lock_server not reachable — refusing to move without it'

    done = threading.Event()
    result = {}

    def _cb(fut):
        result['r'] = fut.result()
        done.set()

    req = AcquireArmMotionLock.Request(holder_id=holder_id, lease_sec=lease_sec)
    client.call_async(req).add_done_callback(_cb)
    if not done.wait(timeout=_CALL_TIMEOUT_SEC):
        return False, 'arm_motion_lock_server did not respond in time'

    r = result.get('r')
    if r is None:
        return False, 'arm_motion_lock_server call failed'
    return bool(r.granted), r.message


def _call_release(client, holder_id: str) -> None:
    if not client.wait_for_service(timeout_sec=_SERVICE_WAIT_SEC):
        return  # best-effort — the lease expires on its own regardless
    done = threading.Event()
    req = ReleaseArmMotionLock.Request(holder_id=holder_id)
    client.call_async(req).add_done_callback(lambda _fut: done.set())
    done.wait(timeout=_CALL_TIMEOUT_SEC)
