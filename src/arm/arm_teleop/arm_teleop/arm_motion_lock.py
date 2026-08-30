"""Cross-process mutual exclusion for anything that submits a goal to
``indomitus_arm_controller`` (the JTC).

``keyboard_servo_node.py``'s home move / remembered-position replay and
``panel_align_node.py``'s live (CV+MoveIt) align are separate PROCESSES,
each perfectly happy to submit a FollowJointTrajectory-family goal to the
same action server. Confirmed live: pressing 'r' while a 'p' live align
is planning/executing let both through, racing two goals onto the same
controller (see PR review — this is what this module fixes). Each side
already has its own in-process ``threading.Lock`` guarding against
racing ITSELF (two 'r' presses, two 'p' presses); a plain
``threading.Lock`` only ever protects within the one process that
created it, so a second, cross-process layer is needed for the
different-process case.

A POSIX advisory file lock (``flock``) is the standard way to get that:
the kernel honors it across every process on the host holding the same
file open, whether or not they know about each other beyond agreeing on
this path.
"""

from __future__ import annotations

import contextlib
import fcntl
import time

# keyboard_servo_node.py (this package, arm_teleop) and panel_align_node.py
# (arm_tasks) run as separate ros2 run processes, from separate packages —
# /tmp is a simple, always-available rendezvous point for them, same
# tradeoff already accepted for the sim marker-layout file (see
# panel_pose_fuser_node.py's PANEL_MARKER_LAYOUT_SIM_FILE).
ARM_MOTION_LOCK_PATH = '/tmp/indomitus_arm_motion.lock'


class ArmMotionBusy(Exception):
    """Raised by arm_motion_lock() when another process holds the lock."""


@contextlib.contextmanager
def arm_motion_lock(timeout_sec: float = 0.0):
    """Acquire the cross-process arm-motion lock for the ``with`` block.

    Args:
        timeout_sec: 0.0 (default) tries once and raises ``ArmMotionBusy``
            immediately if another process already holds it — mirrors the
            existing ``threading.Lock().acquire(blocking=False)`` pattern
            used for the in-process locks in both callers, so 'r' vs 'p'
            fails fast and cleanly instead of queueing. A positive value
            retries (flock has no native timeout) up to that many seconds
            before giving up the same way.

    Raises:
        ArmMotionBusy: another process holds the lock and timeout_sec
            elapsed (or was 0) without acquiring it.
    """
    fd = open(ARM_MOTION_LOCK_PATH, 'w')
    deadline = time.monotonic() + timeout_sec
    try:
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise ArmMotionBusy(
                        'Another process is currently commanding the arm '
                        f'(lock held on {ARM_MOTION_LOCK_PATH}).'
                    ) from None
                time.sleep(0.05)
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        fd.close()
