#!/usr/bin/env python3
"""Who twist_mux is currently listening to, for the tower's red/green lamp.

twist_mux always outputs the highest-priority input that has not timed out,
independent of whatever the others are doing - see
rover_bringup/config/twist_mux.yaml, which the priorities and timeouts here
must keep matching. This mirrors that rule rather than reading it back off
twist_mux, which exposes it only through free-text /diagnostics fields, not a
stable machine-readable one.

No ROS in it, same split as gs_link_watchdog.py and link_state.py, so the
selection rule is tested without a ROS environment. The clock is injectable
for the same reason theirs is.
"""

import threading
import time
from typing import NamedTuple, Optional


class Source(NamedTuple):
    name: str
    priority: int
    timeout: float
    autonomous: bool


class DriveSourceWatchdog:
    """Thread-safe: subscriber callbacks feed it, the lamp timer reads it."""

    def __init__(self, sources, clock=time.monotonic):
        self._sources = {s.name: s for s in sources}
        self._clock = clock
        self._lock = threading.RLock()
        self._last_seen = {}

    def on_message(self, name: str):
        with self._lock:
            self._last_seen[name] = self._clock()

    def winner(self) -> Optional[str]:
        """Name of the source twist_mux is currently outputting, or None if
        every input has gone stale and twist_mux has nothing to output."""
        with self._lock:
            now = self._clock()
            fresh = [
                s for s in self._sources.values()
                if (seen := self._last_seen.get(s.name)) is not None
                and now - seen <= s.timeout
            ]
            if not fresh:
                return None
            return max(fresh, key=lambda s: s.priority).name

    def autonomous(self) -> Optional[bool]:
        """True while nav2 is winning, False while a manual source is, None
        while nothing is currently fresh enough for twist_mux to output."""
        name = self.winner()
        if name is None:
            return None
        return self._sources[name].autonomous
