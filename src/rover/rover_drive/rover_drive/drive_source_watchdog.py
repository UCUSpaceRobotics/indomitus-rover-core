#!/usr/bin/env python3
"""Who twist_mux is currently listening to, for the tower's red/green lamp.

twist_mux always outputs the highest-priority input that has not timed out,
independent of whatever the others are doing. This mirrors that rule rather
than reading it back off twist_mux, which exposes it only through free-text
/diagnostics fields, not a stable machine-readable one. The inputs themselves
(topics, priorities, timeouts) are read from the same config/twist_mux.yaml
twist_mux loads - see sources_from_twist_mux - so the two cannot drift apart.

No ROS in it, same split as gs_link_watchdog.py and link_state.py, so the
selection rule is tested without a ROS environment. The clock is injectable
for the same reason theirs is.
"""

import threading
import time
from typing import Iterable, NamedTuple, Optional


class Source(NamedTuple):
    name: str
    priority: int
    timeout: float
    autonomous: bool


def sources_from_twist_mux(config: dict, autonomous_topics: Iterable[str]):
    """Sources from a parsed twist_mux parameter file.

    `config` is the whole yaml document; the node entry is found by its last
    path segment (`/**/twist_mux`, `/rover/twist_mux`, plain `twist_mux`).
    twist_mux itself has no notion of autonomous inputs, so which topics count
    as autonomous comes from the caller. An autonomous topic missing from the
    file is an error rather than a silently green lamp.
    """
    params = None
    for key, value in config.items():
        if key.rstrip('/').split('/')[-1] == 'twist_mux':
            params = (value or {}).get('ros__parameters', {})
            break
    if params is None:
        raise ValueError('no twist_mux entry in the twist_mux config')

    topics = params.get('topics') or {}
    if not topics:
        raise ValueError('twist_mux config has no topics')

    autonomous = set(autonomous_topics)
    sources = tuple(
        Source(
            name=entry['topic'],
            priority=int(entry['priority']),
            timeout=float(entry['timeout']),
            autonomous=entry['topic'] in autonomous,
        )
        for entry in topics.values()
    )
    missing = autonomous - {s.name for s in sources}
    if missing:
        raise ValueError(
            f'autonomous topics not in the twist_mux config: {sorted(missing)}')
    return sources


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
