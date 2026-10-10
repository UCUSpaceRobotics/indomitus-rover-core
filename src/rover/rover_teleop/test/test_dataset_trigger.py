"""Gamepad-driven dataset capture against stand-in camera servers.

Each fake server answers /health, /capture and /record the way
camera_mjpeg_server.py does, so this checks the part that is easy to get
wrong: one press reaching every camera, a missing camera not breaking the
rest, and the record toggle converging on one state instead of flipping each
camera on its own.

No ROS import anywhere in here — dataset_trigger is deliberately standalone.
"""

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

import pytest

from rover_teleop.dataset_trigger import DatasetTrigger, parse_recording


class FakeCamera:
    def __init__(self, recording=False):
        self.recording = recording
        self.captures = 0
        camera = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                url = urlparse(self.path)
                if url.path == '/health':
                    body = f'ok name=fake recording={int(camera.recording)} saved=0'
                elif url.path == '/capture':
                    camera.captures += 1
                    body = 'ok saved=/tmp/x.jpg'
                elif url.path == '/record':
                    camera.recording = parse_qs(url.query)['on'][0] == '1'
                    body = f'ok recording={int(camera.recording)}'
                else:
                    self.send_error(404)
                    return
                data = body.encode()
                self.send_response(200)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.server = HTTPServer(('127.0.0.1', 0), Handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def dead_port():
    """A port nothing listens on: bound, then released."""
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.fixture
def cameras():
    made = []

    def make(*recording):
        for r in recording:
            made.append(FakeCamera(r))
        return made

    yield make
    for cam in made:
        cam.close()


def trigger(ports):
    return DatasetTrigger('127.0.0.1', ports, timeout=1.0, log=lambda *a: None)


# ── /health parsing ──────────────────────────────────────────────────────────

def test_parse_recording_reads_the_field():
    assert parse_recording('ok name=a mode=idle recording=1 saved=3') is True
    assert parse_recording('ok name=a recording=0 saved=0') is False
    assert parse_recording('ok name=a mode=idle') is None   # an older server
    assert parse_recording(None) is None


# ── capture ──────────────────────────────────────────────────────────────────

def test_capture_reaches_every_camera_and_skips_a_missing_one(cameras):
    a, b = cameras(False, False)
    saved = trigger([a.port, dead_port(), b.port]).capture_now()
    assert sorted(saved) == sorted([a.port, b.port])
    assert (a.captures, b.captures) == (1, 1)


# ── record toggle ────────────────────────────────────────────────────────────

def test_toggle_turns_recording_on_when_nobody_records(cameras):
    a, b = cameras(False, False)
    target, done = trigger([a.port, b.port]).toggle_record_now()
    assert target is True
    assert len(done) == 2
    assert a.recording and b.recording


def test_toggle_turns_everything_off_when_any_camera_records(cameras):
    # Out of step (one camera restarted mid-run): converge on off, never flip
    # into the opposite mixed state.
    a, b = cameras(True, False)
    target, _ = trigger([a.port, b.port]).toggle_record_now()
    assert target is False
    assert not a.recording and not b.recording


def test_toggle_with_no_camera_running_does_nothing():
    target, done = trigger([dead_port()]).toggle_record_now()
    assert target is None and done == {}


# ── background press ─────────────────────────────────────────────────────────

def test_a_second_press_while_the_first_is_out_is_dropped(cameras):
    (a,) = cameras(False)
    t = trigger([a.port])
    release = threading.Event()
    t._capture_and_log = release.wait          # hold the first request open
    assert t.capture() is True
    assert t.capture() is False
    release.set()
    deadline = time.time() + 2
    while time.time() < deadline and not t._busy.acquire(blocking=False):
        time.sleep(0.01)
    t._busy.release()
