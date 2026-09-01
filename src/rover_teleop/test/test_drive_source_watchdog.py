"""Which input wins the twist_mux race, pinned.

DriveSourceWatchdog has no ROS in it and takes its clock as an argument
precisely so these can be tested here - same split and same reason as
test_link_state.py / test_gs_link_watchdog.py. The priorities/timeouts below
mirror rover_bringup/config/twist_mux.yaml at the time this was written.
"""

from rover_teleop.drive_source_watchdog import DriveSourceWatchdog, Source


class FakeClock:
    """Time only moves when a test says so."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


SOURCES = (
    Source('cmd_vel_joy', priority=100, timeout=0.5, autonomous=False),
    Source('cmd_vel_gs', priority=50, timeout=0.5, autonomous=False),
    Source('cmd_vel_nav', priority=20, timeout=1.0, autonomous=True),
    Source('cmd_vel_ext', priority=10, timeout=0.5, autonomous=False),
    Source('cmd_vel_lora', priority=5, timeout=1.0, autonomous=False),
)


def make_watchdog(clock=None):
    return DriveSourceWatchdog(SOURCES, clock=clock or FakeClock())


# -- starting state ----------------------------------------------------

def test_starts_with_no_winner_before_any_message():
    watchdog = make_watchdog()
    assert watchdog.winner() is None
    assert watchdog.autonomous() is None


# -- priority ordering ----------------------------------------------------

def test_only_source_wins_by_default():
    watchdog = make_watchdog()
    watchdog.on_message('cmd_vel_nav')
    assert watchdog.winner() == 'cmd_vel_nav'
    assert watchdog.autonomous() is True


def test_higher_priority_source_wins_even_if_older():
    clock = FakeClock()
    watchdog = make_watchdog(clock)
    watchdog.on_message('cmd_vel_nav')
    clock.advance(0.2)
    watchdog.on_message('cmd_vel_joy')
    # Both are still fresh; joystick (100) outranks nav (20) regardless of
    # which one published more recently - this is twist_mux's rule, not a
    # most-recent-wins one.
    assert watchdog.winner() == 'cmd_vel_joy'
    assert watchdog.autonomous() is False


def test_lower_priority_source_wins_once_the_higher_one_goes_stale():
    # The case that motivates keeping every input, not just nav vs the
    # highest manual one: LoRa (5) outranks nothing except a stale nav (20),
    # but a stale nav is exactly when it should.
    clock = FakeClock()
    watchdog = make_watchdog(clock)
    watchdog.on_message('cmd_vel_nav')
    clock.advance(1.1)                     # nav's 1.0s timeout elapses
    watchdog.on_message('cmd_vel_lora')
    assert watchdog.winner() == 'cmd_vel_lora'
    assert watchdog.autonomous() is False


def test_nav_and_lora_fresh_together_nav_still_wins():
    # nav (20) outranks lora (5) even though lora is the more "recent enough
    # to matter" manual fallback - a naive "any manual source fresh -> green"
    # rule would get this wrong.
    clock = FakeClock()
    watchdog = make_watchdog(clock)
    watchdog.on_message('cmd_vel_lora')
    watchdog.on_message('cmd_vel_nav')
    assert watchdog.winner() == 'cmd_vel_nav'
    assert watchdog.autonomous() is True


# -- silence ----------------------------------------------------------------

def test_no_winner_once_the_only_source_goes_stale():
    clock = FakeClock()
    watchdog = make_watchdog(clock)
    watchdog.on_message('cmd_vel_joy')
    clock.advance(0.6)
    assert watchdog.winner() is None
    assert watchdog.autonomous() is None


def test_falls_through_to_the_next_fresh_source_on_timeout():
    clock = FakeClock()
    watchdog = make_watchdog(clock)
    watchdog.on_message('cmd_vel_joy')
    watchdog.on_message('cmd_vel_nav')
    clock.advance(0.6)                     # joystick's 0.5s timeout elapses
    assert watchdog.winner() == 'cmd_vel_nav'
    assert watchdog.autonomous() is True


def test_recovers_without_a_restart():
    clock = FakeClock()
    watchdog = make_watchdog(clock)
    watchdog.on_message('cmd_vel_joy')
    clock.advance(0.6)
    assert watchdog.winner() is None

    watchdog.on_message('cmd_vel_gs')
    assert watchdog.winner() == 'cmd_vel_gs'
    assert watchdog.autonomous() is False
