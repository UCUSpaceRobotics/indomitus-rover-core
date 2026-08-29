"""Watchdog timing and out-of-order service replies.

Both failures these guard against are ones a controller in your hands will not
reproduce on demand: a joystick that drops and comes back while nav holds the
rover, and two controller switches whose replies race each other.

No ROS import anywhere in here — teleop_state is deliberately standalone.
"""

from rover_teleop.teleop_state import GenerationGuard, JoyWatchdog


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


# ── superseded service replies ───────────────────────────────────────────────

class SwitchControllerSim:
    """The node's switch_controller flow with the futures taken out.

    Mirrors _set_swerve_controller_state (claim a generation, send it along
    with the request) and _on_switch_controller_result (drop the reply unless
    its generation is still the newest). Replies are completed by hand so they
    can be landed in any order.
    """

    def __init__(self):
        self.guard = GenerationGuard()
        self.controller_active = False
        self.ignored = []

    def request(self, activate: bool):
        """Returns a callable that completes this request's reply."""
        generation = self.guard.start()

        def complete(ok: bool = True):
            if not self.guard.is_current(generation):
                self.ignored.append(activate)
                return False
            if ok:
                self.controller_active = activate
            return True

        return complete


def test_generations_are_unique_and_only_the_newest_is_current():
    guard = GenerationGuard()

    first = guard.start()
    second = guard.start()

    assert first != second
    assert guard.is_current(second) is True
    assert guard.is_current(first) is False


def test_stale_reply_landing_last_cannot_overwrite_the_newer_state():
    # Motors are cycled off and straight back on. The deactivate reply is slow
    # and lands after the activate reply. Letting it through leaves
    # _controller_active False while the controller is in fact active — the
    # light bar then reads orange with the rover drivable.
    sim = SwitchControllerSim()

    deactivate = sim.request(activate=False)   # request A
    activate = sim.request(activate=True)      # request B

    assert activate() is True
    assert sim.controller_active is True

    assert deactivate() is False
    assert sim.controller_active is True
    assert sim.ignored == [False]


def test_stale_reply_landing_first_is_ignored_and_the_newest_still_applies():
    # The opposite order, which is the one that proves the guard keys on the
    # generation and not on 'whoever answered last'. A's reply arrives while B
    # is still in flight: it is already superseded and must not be applied,
    # and B must still land afterwards.
    sim = SwitchControllerSim()

    activate = sim.request(activate=True)      # request A
    deactivate = sim.request(activate=False)   # request B

    assert activate() is False
    assert sim.controller_active is False
    assert sim.ignored == [True]

    assert deactivate() is True
    assert sim.controller_active is False


def test_the_newest_request_wins_however_many_are_in_flight():
    sim = SwitchControllerSim()

    replies = [sim.request(activate=(i % 2 == 0)) for i in range(5)]

    # Land them in a shuffled order; only the last request may take effect.
    for index in (2, 0, 4, 1, 3):
        replies[index]()

    assert sim.controller_active is True   # request 4 asked for activate=True
    assert sim.ignored == [True, True, False, False]


def test_the_newest_reply_still_applies_when_nothing_superseded_it():
    # The guard must not be so eager that the ordinary case stops working.
    sim = SwitchControllerSim()

    assert sim.request(activate=True)() is True
    assert sim.controller_active is True
    assert sim.ignored == []


def test_a_refused_switch_leaves_the_state_alone():
    # controller_manager answered, but said no. The reply is current, so it is
    # not ignored — it simply must not claim a state change that never happened.
    sim = SwitchControllerSim()
    sim.request(activate=True)(ok=True)

    assert sim.request(activate=False)(ok=False) is True
    assert sim.controller_active is True
    assert sim.ignored == []
