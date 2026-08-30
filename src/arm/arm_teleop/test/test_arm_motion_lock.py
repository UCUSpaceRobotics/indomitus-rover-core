"""Cross-process arm-motion lock — pure Python, no ROS graph needed."""
import multiprocessing as mp
import threading

import pytest

from arm_teleop.arm_motion_lock import ArmMotionBusy, arm_motion_lock


def test_acquire_and_release_within_one_process():
    with arm_motion_lock():
        pass
    # Lock released on exit — acquiring again immediately must succeed.
    with arm_motion_lock():
        pass


def test_nonblocking_acquire_raises_when_already_held():
    with arm_motion_lock():
        with pytest.raises(ArmMotionBusy):
            with arm_motion_lock():
                pass


def _hold_lock_in_subprocess(ready: mp.Event, release: mp.Event):
    with arm_motion_lock():
        ready.set()
        release.wait(timeout=5)


def test_lock_is_actually_honored_across_processes():
    """The whole point of this module: a plain threading.Lock would NOT
    catch this — this must use a real cross-process (flock) mechanism.
    """
    ready = mp.Event()
    release = mp.Event()
    holder = mp.Process(target=_hold_lock_in_subprocess, args=(ready, release))
    holder.start()
    try:
        assert ready.wait(timeout=5), 'subprocess never acquired the lock'
        with pytest.raises(ArmMotionBusy):
            with arm_motion_lock():
                pass
    finally:
        release.set()
        holder.join(timeout=5)

    # Released by the subprocess exiting — acquiring now must succeed.
    with arm_motion_lock():
        pass


def test_timeout_seconds_retries_before_giving_up():
    ready = mp.Event()
    release = mp.Event()
    holder = mp.Process(target=_hold_lock_in_subprocess, args=(ready, release))
    holder.start()
    try:
        assert ready.wait(timeout=5)
        threading.Timer(0.1, release.set).start()
        # Should succeed once the holder releases, well within the 2s budget.
        with arm_motion_lock(timeout_sec=2.0):
            pass
    finally:
        release.set()
        holder.join(timeout=5)
