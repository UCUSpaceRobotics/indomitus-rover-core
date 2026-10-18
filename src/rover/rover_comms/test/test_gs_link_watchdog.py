"""Connected/disconnected rule for the ground-station link heartbeat, pinned.

GsLinkWatchdog has no ROS in it and takes its clock as an argument precisely
so these can be tested here, same split and same reason as test_link_state.py.
"""

from rover_comms.gs_link_watchdog import GsLinkWatchdog


class FakeClock:
    """Time only moves when a test says so."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def make_watchdog(clock=None, timeout=2.0):
    return GsLinkWatchdog(timeout=timeout, clock=clock or FakeClock())


# -- starting state ----------------------------------------------------

def test_starts_disconnected_before_any_message():
    # A rover that has never heard from link_status_node is in the same state
    # as one that stopped hearing from it - no first-message grace period.
    watchdog = make_watchdog()
    assert watchdog.connected() is False


# -- normal operation ----------------------------------------------------

def test_ok_state_is_connected():
    watchdog = make_watchdog()
    watchdog.on_message("OK")
    assert watchdog.connected() is True


def test_degraded_state_is_still_connected():
    # DEGRADED means the link is up but poor, not that the ground station is
    # unreachable - the tower lamp is about reachability, not link quality.
    watchdog = make_watchdog()
    watchdog.on_message("DEGRADED")
    assert watchdog.connected() is True


def test_down_state_is_not_connected_even_though_messages_are_arriving():
    # link_status_node keeps publishing at 2 Hz even when it has nothing to
    # say - "DOWN" is a message, not silence, and must not read as connected.
    watchdog = make_watchdog()
    watchdog.on_message("DOWN")
    assert watchdog.connected() is False


# -- silence ----------------------------------------------------------------

def test_timeout_marks_disconnected_when_messages_stop():
    clock = FakeClock()
    watchdog = make_watchdog(clock, timeout=2.0)
    watchdog.on_message("OK")
    assert watchdog.connected() is True

    clock.advance(1.9)
    assert watchdog.connected() is True    # still inside the window

    clock.advance(0.2)
    assert watchdog.connected() is False


def test_recovers_without_a_restart():
    clock = FakeClock()
    watchdog = make_watchdog(clock, timeout=2.0)
    watchdog.on_message("OK")
    clock.advance(3.0)
    assert watchdog.connected() is False

    watchdog.on_message("OK")
    assert watchdog.connected() is True


# -- the watchdog clock -------------------------------------------------

def test_watchdog_is_immune_to_a_backward_clock_step():
    # Same reason link_state.py's watchdog takes time.monotonic and not the
    # ROS clock: an NTP step or a paused /clock must not hold a stale
    # "connected" reading off indefinitely by making the measured age negative.
    clock = FakeClock()
    watchdog = make_watchdog(clock, timeout=2.0)
    watchdog.on_message("OK")
    clock.now -= 3600.0
    assert watchdog.connected() is True, "a jump must not trip it early"
    clock.now += 3600.0 + 3.0
    assert watchdog.connected() is False, "and must not hold it off either"


def test_defaults_to_a_monotonic_clock():
    import time
    watchdog = GsLinkWatchdog(timeout=2.0)
    assert watchdog._clock is time.monotonic
