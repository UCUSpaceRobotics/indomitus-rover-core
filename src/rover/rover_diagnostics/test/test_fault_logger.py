"""Replay suppression and log-file behaviour.

The failure these guard against is a forensic log that lies: the same fault
recorded twice after a reconnect reads as two faults, and a log that stops
rotating fills the disk on the rover it is supposed to be diagnosing.
"""

import json
import math
import types

import pytest

from rover_diagnostics.event_log import EventLog
from rover_diagnostics.fault_logger_node import EventDeduper


def make_event(sec=10, nanosec=500, component='chassis/drive_FL',
               event=0, fault=7, raw_code=0xB):
    """A stand-in for FaultEvent with just the fields dedupe keys on."""
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec)),
        component=component,
        event=event,
        fault=fault,
        raw_code=raw_code,
    )


# ── replay suppression ───────────────────────────────────────────────────────

def test_first_event_is_not_a_duplicate():
    assert EventDeduper().is_duplicate(make_event()) is False


def test_transient_local_replay_is_suppressed():
    # What a reconnect looks like: the publisher re-delivers its history, so
    # the same messages arrive a second time with their original stamps.
    d = EventDeduper()
    history = [make_event(sec=s) for s in range(5)]

    for event in history:
        assert d.is_duplicate(event) is False
    for event in history:
        assert d.is_duplicate(event) is True


def test_distinct_transitions_are_all_kept():
    d = EventDeduper()
    base = make_event()
    assert d.is_duplicate(base) is False
    # Same instant, different motor.
    assert d.is_duplicate(make_event(component='chassis/drive_FR')) is False
    # Same motor and instant, different transition.
    assert d.is_duplicate(make_event(event=2)) is False
    # Same everything but a later stamp: a genuine second occurrence.
    assert d.is_duplicate(make_event(nanosec=501)) is False


def test_unstamped_event_is_passed_through():
    # Nothing to key on. Logging a duplicate beats dropping a real fault.
    d = EventDeduper()
    assert d.is_duplicate(make_event(sec=0, nanosec=0)) is False
    assert d.is_duplicate(make_event(sec=0, nanosec=0)) is False


def test_history_is_bounded_and_evicts_oldest():
    d = EventDeduper(history=4)
    for sec in range(4):
        d.is_duplicate(make_event(sec=sec))

    # Still remembered.
    assert d.is_duplicate(make_event(sec=3)) is True

    # Pushing past the bound evicts the oldest, which can then reappear. That
    # is the accepted cost of a fixed-size memory: the bound only has to
    # outlast one replay burst.
    for sec in range(10, 15):
        d.is_duplicate(make_event(sec=sec))
    assert d.is_duplicate(make_event(sec=0)) is False


# ── log file ─────────────────────────────────────────────────────────────────

def test_each_session_gets_its_own_file(tmp_path):
    first = EventLog(directory=str(tmp_path))
    second = EventLog(directory=str(tmp_path))
    assert first.path != second.path
    first.close()
    second.close()


def test_non_finite_floats_become_null(tmp_path):
    # NaN is the "vendor does not report this" sentinel, and bare NaN is not
    # valid JSON -- it breaks every standard parser reading the log later.
    log = EventLog(directory=str(tmp_path))
    log.write({'event': 'FAULT_ENTER', 'freeze_frame': {'voltage': math.nan,
                                                        'current': math.inf,
                                                        'torque': 1.5}})
    log.close()

    with open(log.path, encoding='utf-8') as handle:
        record = json.loads(handle.readline())

    assert record['freeze_frame']['voltage'] is None
    assert record['freeze_frame']['current'] is None
    assert record['freeze_frame']['torque'] == 1.5


def test_log_rotates_and_is_size_capped(tmp_path):
    # A full disk is itself a rover failure mode, so the cap matters more than
    # keeping every line.
    log = EventLog(directory=str(tmp_path), max_bytes=200, backup_count=2)
    for i in range(200):
        log.write({'event': 'FAULT_ENTER', 'i': i, 'pad': 'x' * 40})
    log.close()

    written = sorted(tmp_path.iterdir())
    assert len(written) <= 3, 'backup_count=2 means base + 2 backups at most'
    for path in written:
        assert path.stat().st_size < 400


def test_rotation_keeps_the_most_recent_events(tmp_path):
    # Rotation happens after the write that crossed the threshold, so the
    # newest event can be in the live file or in .1 depending on where the
    # boundary fell. What must never happen is losing it.
    log = EventLog(directory=str(tmp_path), max_bytes=200, backup_count=1)
    for i in range(200):
        log.write({'i': i, 'pad': 'x' * 40})
    log.close()

    seen = set()
    for path in tmp_path.iterdir():
        with open(path, encoding='utf-8') as handle:
            seen.update(json.loads(line)['i'] for line in handle if line.strip())

    assert 199 in seen, 'the newest event must survive rotation'


@pytest.mark.parametrize('max_bytes', [0, -1])
def test_rotation_can_be_disabled(tmp_path, max_bytes):
    log = EventLog(directory=str(tmp_path), max_bytes=max_bytes)
    for i in range(50):
        log.write({'i': i})
    log.close()

    assert len(list(tmp_path.iterdir())) == 1
