#!/usr/bin/env python3
"""
Timing and ordering state machines used by the joystick interpreter.
"""

from typing import NamedTuple


class WatchdogTick(NamedTuple):
    """What a single watchdog poll asks the caller to do."""

    #: The /joy stream just went stale on this tick — worth one log line.
    #: True on the live → stale edge only, never on subsequent polls.
    went_stale: bool
    #: Publish a zero command now.
    publish_zero: bool


class JoyWatchdog:
    """Tracks /joy freshness and says when to publish safe zero commands."""

    def __init__(self, timeout: float, timed_out: bool = True):
        self._timeout = timeout
        self._timed_out = timed_out
        # 0.0 means 'nothing has ever arrived', which counts as stale rather
        # than as a message at the epoch.
        self._last_message_time = 0.0

    @property
    def timed_out(self) -> bool:
        return self._timed_out

    def on_message(self, now: float) -> bool:
        """Record a fresh /joy message.

        Returns True on the stale -> live edge, i.e. exactly when the caller
        should announce the recovery and repaint the light bar. A controller
        that dropped and came back has lost whatever colour it was wearing.
        """
        self._last_message_time = now
        if not self._timed_out:
            return False
        self._timed_out = False
        return True

    def tick(self, now: float, active: bool) -> WatchdogTick:
        """Age the last message and report what the caller should do."""
        if self._last_message_time > 0.0:
            age = now - self._last_message_time
        else:
            age = float('inf')

        if age <= self._timeout:
            return WatchdogTick(went_stale=False, publish_zero=False)

        went_stale = not self._timed_out
        self._timed_out = True
        return WatchdogTick(went_stale=went_stale, publish_zero=active)


class GenerationGuard:
    """Lets only the reply to the newest request write back state.

    Service replies can land out of order, and an old one must not be allowed
    to overwrite state the newer request has already settled — that is how the
    light bar ends up green with the controller actually inactive.
    """

    def __init__(self):
        self._generation = 0

    @property
    def generation(self) -> int:
        return self._generation

    def start(self) -> int:
        """Claim the newest generation. Hand the value to the reply callback."""
        self._generation += 1
        return self._generation

    def is_current(self, generation: int) -> bool:
        """True when `generation` is still the newest request in flight."""
        return generation == self._generation

