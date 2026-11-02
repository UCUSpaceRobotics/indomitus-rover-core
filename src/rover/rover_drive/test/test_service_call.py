from rover_drive.service_call import GenerationGuard


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
