"""Safety behaviour of the LoRa link, pinned.

These are the cases that decide whether the rover moves, and until now they
were only ever checked by hand against real hardware. LinkState has no ROS in
it and takes its clock as an argument precisely so they can be tested here:
every one of these ran as a manual bench check before it was a test.

The clock is a plain counter, not time.monotonic, so "a second passed" is a
statement about the code rather than about how fast the machine running CI is.
"""

from rover_comms import link_state, lora_frame


class FakeClock:
    """Time only moves when a test says so."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def make_state(clock=None, **kwargs):
    settings = dict(failsafe_timeout=1.0, max_linear=0.5, max_angular=1.0,
                    limit_linear=0.3, limit_angular=0.6)
    settings.update(kwargs)
    return link_state.LinkState(clock=clock or FakeClock(), **settings)


def teleop(vx=0, vy=0, wz=0, flags=0):
    return lora_frame.pack_teleop(lora_frame.Teleop(vx, vy, wz, flags))


# -- starting state --------------------------------------------------------

def test_starts_failsafed_before_any_frame():
    # A rover that has never heard from the mast is in the same state as one
    # that stopped hearing from it. No first-timeout grace period.
    state = make_state()
    assert state.failsafe is True
    assert state.twist_components() == (0.0, 0.0, 0.0)
    assert state.status_flags() & lora_frame.STATUS_FAILSAFE


def test_timeout_cannot_trip_before_a_first_frame():
    clock = FakeClock()
    state = make_state(clock)
    clock.advance(3600.0)
    assert state.check_timeout() is None
    assert state.twist_components() == (0.0, 0.0, 0.0)


# -- normal operation ------------------------------------------------------

def test_first_frame_takes_the_link_up():
    state = make_state()
    assert state.on_teleop(teleop(vx=50)) is True     # reports the recovery
    assert state.failsafe is False
    assert state.on_teleop(teleop(vx=50)) is False    # already up


def test_commands_decode_at_the_wire_scale():
    state = make_state()
    state.on_teleop(teleop(vx=50, vy=-50, wz=25))
    vx, vy, wz = state.twist_components()
    assert (vx, vy) == (0.25, -0.25)   # 50% of 0.5, under the 0.3 cap
    assert wz == 0.25                  # 25% of 1.0, under the 0.6 cap


def test_top_speed_is_capped_not_rescaled():
    # The distinction this link got wrong once: everything below the cap must
    # arrive at the magnitude the operator asked for.
    state = make_state()
    state.on_teleop(teleop(vx=100, wz=100))
    assert state.twist_components() == (0.3, 0.0, 0.6)

    state.on_teleop(teleop(vx=20))
    assert abs(state.twist_components()[0] - 0.1) < 1e-12


def test_over_range_percentages_cannot_beat_the_cap():
    # unpack_teleop yields the full signed-byte range; the protocol says
    # -100..100. A sender that is not lora_gateway_node must not get 127%.
    state = make_state()
    for payload in (bytes((127, 127, 127, 0)), bytes((128, 128, 128, 0))):
        state.on_teleop(payload)
        vx, vy, wz = state.twist_components()
        assert abs(vx) <= 0.3 and abs(vy) <= 0.3 and abs(wz) <= 0.6


# -- link loss -------------------------------------------------------------

def test_timeout_zeroes_the_command():
    clock = FakeClock()
    state = make_state(clock)
    state.on_teleop(teleop(vx=100))
    assert state.twist_components()[0] == 0.3

    clock.advance(0.9)
    assert state.check_timeout() is None          # still inside the window
    assert state.twist_components()[0] == 0.3

    clock.advance(0.2)
    age = state.check_timeout()
    assert age is not None and age > 1.0
    assert state.failsafe is True
    assert state.twist_components() == (0.0, 0.0, 0.0)


def test_timeout_reports_only_once():
    clock = FakeClock()
    state = make_state(clock)
    state.on_teleop(teleop(vx=100))
    clock.advance(2.0)
    assert state.check_timeout() is not None
    assert state.check_timeout() is None


def test_serial_loss_zeroes_immediately():
    # The port is known dead, so there is nothing to wait for. Before this,
    # the last command stood for up to failsafe_timeout while the reopen was
    # attempted - a second of driving on a command nobody can retract.
    clock = FakeClock()
    state = make_state(clock)
    state.on_teleop(teleop(vx=100))
    assert state.twist_components()[0] == 0.3

    assert state.on_link_lost() is True
    assert state.failsafe is True
    assert state.twist_components() == (0.0, 0.0, 0.0)
    assert clock.now == 1000.0, "no clock advance: it stopped, it did not time out"


def test_serial_loss_is_idempotent():
    state = make_state()
    state.on_teleop(teleop(vx=100))
    assert state.on_link_lost() is True
    assert state.on_link_lost() is False


def test_recovers_without_a_restart():
    clock = FakeClock()
    state = make_state(clock)
    state.on_teleop(teleop(vx=100))
    clock.advance(2.0)
    state.check_timeout()
    assert state.failsafe is True

    assert state.on_teleop(teleop(vx=40)) is True
    assert state.failsafe is False
    assert abs(state.twist_components()[0] - 0.2) < 1e-12


def test_recovers_after_a_serial_loss_too():
    state = make_state()
    state.on_teleop(teleop(vx=100))
    state.on_link_lost()
    state.on_teleop(teleop(vx=40))
    assert state.failsafe is False
    assert abs(state.twist_components()[0] - 0.2) < 1e-12


# -- e-stop ----------------------------------------------------------------

def test_estop_is_applied_here_not_trusted_from_the_sender():
    # The sender is supposed to zero the velocities alongside the flag. This
    # end does not depend on it having done so.
    state = make_state()
    state.on_teleop(teleop(vx=100, vy=100, wz=100, flags=lora_frame.FLAG_ESTOP))
    assert state.twist_components() == (0.0, 0.0, 0.0)
    assert state.estop_active() is True
    assert state.status_flags() & lora_frame.STATUS_ESTOP


def test_estop_survives_a_timeout():
    # A link that went quiet is not the operator releasing the e-stop. It used
    # to be reported as one, on lora/rover_estop and in the STATUS reply.
    clock = FakeClock()
    state = make_state(clock)
    state.on_teleop(teleop(flags=lora_frame.FLAG_ESTOP))
    clock.advance(2.0)
    state.check_timeout()
    assert state.estop_active() is True
    assert state.status_flags() & lora_frame.STATUS_ESTOP
    assert state.status_flags() & lora_frame.STATUS_FAILSAFE


def test_estop_survives_a_serial_loss():
    state = make_state()
    state.on_teleop(teleop(flags=lora_frame.FLAG_ESTOP))
    state.on_link_lost()
    assert state.estop_active() is True


def test_estop_clears_only_when_a_frame_clears_it():
    state = make_state()
    state.on_teleop(teleop(flags=lora_frame.FLAG_ESTOP))
    assert state.estop_active() is True
    state.on_teleop(teleop(vx=50))
    assert state.estop_active() is False
    assert state.twist_components()[0] == 0.25


# -- the watchdog clock ----------------------------------------------------

def test_watchdog_is_immune_to_a_backward_clock_step():
    # The reason this takes time.monotonic and not the ROS clock. An NTP step
    # on a Jetson that syncs late, or a paused /clock, used to be able to hold
    # the failsafe off indefinitely by making the measured age negative.
    clock = FakeClock()
    state = make_state(clock)
    state.on_teleop(teleop(vx=100))
    clock.now -= 3600.0
    assert state.check_timeout() is None, "a jump must not trip it early"
    clock.now += 3600.0 + 2.0
    assert state.check_timeout() is not None, "and must not hold it off either"
    assert state.twist_components() == (0.0, 0.0, 0.0)


def test_defaults_to_a_monotonic_clock():
    import time
    state = link_state.LinkState(
        failsafe_timeout=1.0, max_linear=0.5, max_angular=1.0,
        limit_linear=0.3, limit_angular=0.6)
    assert state._clock is time.monotonic


# -- should_publish: burst then quiet ---------------------------------------

def test_live_commands_publish_unmetered():
    state = make_state(zero_burst=3)
    state.on_teleop(teleop(vx=100))
    for _ in range(10):
        assert state.should_publish() is True


def test_failsafed_link_gets_only_a_short_burst_then_goes_quiet():
    # A failsafed LinkState must not hold cmd_vel_lora open forever: LoRa is
    # twist_mux's lowest priority, so there is nothing beneath it for that to
    # protect, and a permanently-fresh zero misreads as "someone is driving"
    # to anything watching twist_mux's inputs.
    state = make_state(zero_burst=3)
    assert state.should_publish() is True
    assert state.should_publish() is True
    assert state.should_publish() is True
    assert state.should_publish() is False
    assert state.should_publish() is False


def test_zero_burst_of_zero_publishes_nothing():
    state = make_state(zero_burst=0)
    assert state.should_publish() is False


def test_a_reconnect_re_arms_the_burst():
    clock = FakeClock()
    state = make_state(clock, zero_burst=2)
    state.should_publish()
    state.should_publish()
    assert state.should_publish() is False   # burst spent

    state.on_teleop(teleop(vx=50))
    assert state.should_publish() is True    # live, unmetered

    clock.advance(2.0)
    state.check_timeout()
    assert state.should_publish() is True    # burst re-armed by the reconnect
    assert state.should_publish() is True
    assert state.should_publish() is False


def test_serial_loss_does_not_grant_a_fresh_burst_on_top_of_an_exhausted_one():
    # on_link_lost() is idempotent once already failsafed - it must not hand
    # back publishes that check_timeout() already spent.
    state = make_state(zero_burst=1)
    state.on_teleop(teleop(vx=50))
    state.on_link_lost()
    assert state.should_publish() is True    # the one burst publish
    assert state.should_publish() is False
    state.on_link_lost()                     # idempotent, no-op
    assert state.should_publish() is False
