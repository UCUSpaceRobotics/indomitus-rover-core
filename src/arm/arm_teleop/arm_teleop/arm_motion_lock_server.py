#!/usr/bin/env python3
"""Cross-host arm-motion lock server — the network-reachable half of
arm_motion_lock.py's mutual exclusion.

Run once, on whichever host actually owns indomitus_arm_controller
(Jetson, via arm_bringup/launch/arm.launch.py) — NOT per-process, unlike
the old /tmp flock this replaces, which only worked when both callers
happened to run on the same machine. servo_controller.py (often GS)
and panel_align_node.py (GS) both submit goals to the same Jetson-hosted
JTC action server; a plain file lock can't provide mutual exclusion
across that host boundary, a ROS service can.

Lease-based rather than connection-held: a plain acquire/release service
pair can't detect a crashed holder the way flock ties a lock to an open
file descriptor's process lifetime. Every acquire carries a caller-
supplied lease_sec; if the holder never releases (crash, network drop),
the lease simply expires and the lock becomes acquirable again instead
of deadlocking every future request.
"""

import threading
import time

import rclpy
from rclpy.node import Node

from indomitus_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock


class ArmMotionLockServer(Node):
    def __init__(self):
        super().__init__('arm_motion_lock_server')
        self._state_lock = threading.Lock()
        self._holder = None
        self._expires_at = None

        self.create_service(AcquireArmMotionLock, 'arm_motion_lock/acquire', self._on_acquire)
        self.create_service(ReleaseArmMotionLock, 'arm_motion_lock/release', self._on_release)
        self.get_logger().info('arm_motion_lock_server ready.')

    def _on_acquire(self, request, response):
        now = time.monotonic()
        with self._state_lock:
            free = (
                self._holder is None
                or self._expires_at is None
                or now >= self._expires_at
                or self._holder == request.holder_id  # reentrant: renew own lease
            )
            if not free:
                response.granted = False
                response.message = f'arm motion lock held by {self._holder!r}'
                return response
            self._holder = request.holder_id
            self._expires_at = now + max(request.lease_sec, 0.0)
            response.granted = True
            response.message = ''
            return response

    def _on_release(self, request, response):
        with self._state_lock:
            if self._holder == request.holder_id:
                self._holder = None
                self._expires_at = None
                response.success = True
                response.message = ''
            else:
                # Not an error: the lease may have already expired and
                # been reassigned, or never granted — either way the
                # caller's actual goal (not holding it) is satisfied.
                response.success = True
                response.message = 'lock was not held by this holder_id (already expired?)'
            return response


def main():
    rclpy.init()
    node = ArmMotionLockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
