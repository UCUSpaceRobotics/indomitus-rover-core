#!/usr/bin/env python3
"""Liveness check for the ground station's link/state heartbeat.

link_status_node on the ground station publishes /gs/link/state at 2 Hz for as
long as its process is up, whatever the Wi-Fi link's quality - "DOWN" is still
a message. So the message itself, not its content, is what says the ground
station is reachable at all: silence is the failure this watches for (gs_comms
not running, DDS discovery lost, the console powered off). Content still
matters once messages are arriving, because a link that link_status_node has
already given up on is not a connection worth lighting the tower for.

No ROS in it, same split as link_state.py, so this can be tested without a ROS
environment. The clock is injectable for the same reason link_state.py's is: a
Jetson's clock is not settled early after boot, and this must not be defeated
by an NTP step or a paused /clock either.
"""

import threading
import time

DOWN = "DOWN"


class GsLinkWatchdog:
    """Thread-safe: the subscriber callback feeds it, the poll timer reads it."""

    def __init__(self, timeout, clock=time.monotonic):
        self.timeout = float(timeout)
        self._clock = clock
        self._lock = threading.RLock()
        self._last_seen_at = None
        self._last_state = None

    def on_message(self, state):
        with self._lock:
            self._last_seen_at = self._clock()
            self._last_state = state

    def connected(self):
        """True while /gs/link/state is arriving fresh and says the link is up."""
        with self._lock:
            if self._last_seen_at is None:
                return False
            if self._clock() - self._last_seen_at > self.timeout:
                return False
            return self._last_state != DOWN
