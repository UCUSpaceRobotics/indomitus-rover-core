"""Watchdog timing and out-of-order service replies.

Both failures these guard against are ones a controller in your hands will not
reproduce on demand: a joystick that drops and comes back while nav holds the
rover, and two controller switches whose replies race each other.

No ROS import anywhere in here — teleop_state is deliberately standalone.
"""

from rover_teleop.teleop_state import JoyWatchdog


def live_watchdog(timeout=0.5, at=100.0, zero_burst=3):
    """A watchdog that has just seen a /joy message, i.e. not timed out."""
    watchdog = JoyWatchdog(timeout=timeout, zero_burst=zero_burst)
    watchdog.on_message(at)
    return watchdog


# ── freshness ────────────────────────────────────────────────────────────────

def test_fresh_input_neither_goes_stale_nor_publishes():
    watchdog = live_watchdog(timeout=0.5, at=100.0)

    tick = watchdog.tick(100.4, active=True)

    assert tick == (False, False)
    assert watchdog.timed_out is False


def test_a_watchdog_that_has_never_seen_a_message_is_stale():
    # last_message_time starts at 0.0, which means 'nothing has arrived' and
    # not 'a message at the epoch' — otherwise the very first tick after boot
    # would compare against 1970 and look stale for a reason nobody intended.
    watchdog = JoyWatchdog(timeout=0.5)

    assert watchdog.timed_out is True
    assert watchdog.tick(100.0, active=True).publish_zero is True


def test_timeout_publishes_a_burst_of_zeros_not_a_single_one():
    # One zero followed by silence is indistinguishable from a dropped message
    # to anything downstream. The stop has to be worth relying on.
    watchdog = live_watchdog(timeout=0.5, at=100.0, zero_burst=3)

    first = watchdog.tick(100.6, active=True)
    assert first == (True, True)

    # Stale is announced once; the rest of the burst keeps coming.
    assert watchdog.tick(100.7, active=True) == (False, True)
    assert watchdog.tick(100.8, active=True) == (False, True)


def test_the_burst_ends_and_the_topic_goes_quiet():
    # The reason the zeros are finite: cmd_vel_joy has the highest priority in
    # twist_mux, so a node that keeps publishing after the gamepad is unplugged
    # holds that priority forever and locks the ground station out of a rover
    # nobody is driving. Going quiet lets the mux time this input out and fall
    # through to whoever is still there.
    watchdog = live_watchdog(timeout=0.5, at=100.0, zero_burst=3)

    published = sum(
        1 for step in range(50)
        if watchdog.tick(100.6 + step * 0.05, active=True).publish_zero
    )

    assert published == 3


def test_a_reconnect_re_arms_the_burst():
    # However brief the gap, the next disconnect gets a full stop of its own.
    watchdog = live_watchdog(timeout=0.5, at=100.0, zero_burst=3)
    for step in range(10):
        watchdog.tick(100.6 + step * 0.05, active=True)

    watchdog.on_message(103.0)

    published = sum(
        1 for step in range(10)
        if watchdog.tick(103.6 + step * 0.05, active=True).publish_zero
    )
    assert published == 3


def test_a_zero_length_burst_publishes_nothing():
    # An operator who would rather hand the topic over instantly can ask for
    # it; the swerve controller's own cmd_vel_timeout_s still stops the rover.
    watchdog = live_watchdog(timeout=0.5, at=100.0, zero_burst=0)

    tick = watchdog.tick(100.6, active=True)

    assert tick.went_stale is True
    assert tick.publish_zero is False


# ── reconnect while the joystick is not the active command source ────────────

def test_timeout_while_inactive_publishes_nothing():
    # Yielding to nav: nav owns /cmd_vel, and a zero from here would fight it.
    watchdog = live_watchdog(timeout=0.5, at=100.0)

    tick = watchdog.tick(100.6, active=False)

    # Still detected — the light bar depends on it — but silent on /cmd_vel.
    assert tick.went_stale is True
    assert tick.publish_zero is False
    assert watchdog.timed_out is True

    for step in range(1, 10):
        assert watchdog.tick(100.6 + step * 0.1, active=False).publish_zero is False


def test_reconnect_while_inactive_still_reports_the_recovery_edge():
    # The whole point of detecting a timeout the joystick is not driving
    # through: a controller that dropped and came back has lost whatever
    # colour it was wearing, and only this edge tells us to repaint it.
    watchdog = live_watchdog(timeout=0.5, at=100.0)
    watchdog.tick(100.6, active=False)
    assert watchdog.timed_out is True

    # Device comes back, first Joy message of the new connection.
    assert watchdog.on_message(105.0) is True
    assert watchdog.timed_out is False


def test_recovery_edge_fires_once_per_disconnect():
    # Repainting on every message would be harmless but pointless; repainting
    # on none of them after the first is the bug. Assert the edge, not the level.
    watchdog = live_watchdog(timeout=0.5, at=100.0)
    watchdog.tick(100.6, active=False)

    assert watchdog.on_message(105.0) is True
    assert watchdog.on_message(105.1) is False
    assert watchdog.on_message(105.2) is False

    # Second disconnect gets its own edge.
    watchdog.tick(106.0, active=False)
    assert watchdog.on_message(107.0) is True


def test_full_drop_and_recovery_cycle_while_inactive_never_touches_cmd_vel():
    # The scenario end to end: nav is driving, the joystick is unplugged, and
    # is plugged back in. Detection and repaint happen throughout; /cmd_vel is
    # never written, because the joystick does not hold control.
    watchdog = live_watchdog(timeout=0.5, at=100.0)
    zeros = 0

    for step in range(20):  # 2 s at the 10 Hz timeout rate — unplugged
        if watchdog.tick(100.1 + step * 0.1, active=False).publish_zero:
            zeros += 1

    assert watchdog.on_message(103.0) is True

    for step in range(10):  # replugged, /joy flowing again
        now = 103.0 + step * 0.1
        watchdog.on_message(now)
        if watchdog.tick(now, active=False).publish_zero:
            zeros += 1

    assert zeros == 0
    assert watchdog.timed_out is False


def test_recovery_while_inactive_leaves_the_joystick_ready_to_drive():
    # Recovery is not conditional on being active, so taking control back
    # afterwards must not need another timeout/recovery round trip.
    watchdog = live_watchdog(timeout=0.5, at=100.0)
    watchdog.tick(100.6, active=False)
    watchdog.on_message(105.0)

    # Operator presses the active-toggle button; input is still flowing.
    assert watchdog.tick(105.2, active=True) == (False, False)
    assert watchdog.timed_out is False


def test_going_active_while_still_timed_out_starts_publishing_zeros():
    # The burst is only spent on zeros actually published, so yielding to nav
    # through a whole disconnect does not use it up. An operator taking control
    # back gets a full stop, not an exhausted counter.
    watchdog = live_watchdog(timeout=0.5, at=100.0, zero_burst=3)
    for step in range(20):
        assert watchdog.tick(100.6 + step * 0.05, active=False).publish_zero is False

    # Nothing came back, but the operator took control anyway.
    published = sum(
        1 for step in range(20)
        if watchdog.tick(101.6 + step * 0.05, active=True).publish_zero
    )
    assert published == 3
