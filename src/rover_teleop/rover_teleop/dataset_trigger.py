#!/usr/bin/env python3
"""Dataset capture from the gamepad: one frame now, or one every second.

The cameras are not ROS nodes on this rover: each is a camera_mjpeg_server.py
(indomitus-ground-station/cameras) serving HTTP on its own port, and saving
frames is its job — /capture saves one, /record?on=1|0 starts or stops saving
one per interval, and /health reports `recording=` and `saved=`. This module
only asks. The container runs with host networking, so the servers are on
127.0.0.1.

The recording state lives in the servers, the same way drive and light state
live in their owners: the ground station UI can flip it too, so a button here
reads every camera's state back and sets an explicit target rather than
sending a blind toggle that two operators would drive out of step.

No ROS import anywhere in here — like joy_input, this is deliberately standalone.
"""

import threading
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor


def _get(host, port, path, timeout):
    """(ok, body) for GET http://host:port/path. A camera that is not running
    on this port reads as (False, None), not as an exception."""
    try:
        with urllib.request.urlopen(f'http://{host}:{port}{path}', timeout=timeout) as resp:
            return True, resp.read().decode('utf-8', 'replace').strip()
    except urllib.error.HTTPError as exc:
        return False, exc.read().decode('utf-8', 'replace').strip()
    except OSError:
        return False, None


def parse_recording(health):
    """True/False from a /health line, None when it says nothing about it."""
    for field in (health or '').split():
        if field.startswith('recording='):
            return field.split('=', 1)[1] == '1'
    return None


class DatasetTrigger:
    """Fans a capture or a record toggle out to every camera server.

    Runs on a background thread, because a closed camera takes about a second
    to open for a capture and /joy must not wait on that. One request in flight
    at a time, the same policy as GuardedCall: a second press while the first
    is still out is dropped and reported, not queued.
    """

    def __init__(self, host, ports, timeout=4.0, log=print):
        self.host = host
        self.ports = list(ports)
        self.timeout = timeout
        self._log = log
        self._busy = threading.Lock()

    # ── what a button press calls ───────────────────────────────────────────

    def capture(self) -> bool:
        return self._in_background(self._capture_and_log)

    def toggle_record(self) -> bool:
        return self._in_background(self._toggle_and_log)

    # ── synchronous core, also what the tests drive ─────────────────────────

    def capture_now(self):
        """{port: body} for every camera that saved a frame."""
        results = self._fan_out('/capture')
        return {port: body for port, (ok, body) in results.items() if ok}

    def toggle_record_now(self):
        """(target, {port: body}), or (None, {}) when no camera answered.

        Recording goes ON unless at least one camera is already recording, so a
        set that is out of step (one camera restarted mid-run) is first brought
        to all-off rather than flipped into the opposite mixed state.
        """
        states = {
            port: parse_recording(body)
            for port, (ok, body) in self._fan_out('/health').items() if ok
        }
        if not states:
            return None, {}
        target = not any(states.values())
        results = self._fan_out(f'/record?on={int(target)}', ports=list(states))
        return target, {port: body for port, (ok, body) in results.items() if ok}

    # ── internals ───────────────────────────────────────────────────────────

    def _fan_out(self, path, ports=None):
        ports = self.ports if ports is None else ports
        if not ports:
            return {}
        with ThreadPoolExecutor(max_workers=len(ports)) as pool:
            futures = {port: pool.submit(_get, self.host, port, path, self.timeout)
                       for port in ports}
            return {port: future.result() for port, future in futures.items()}

    def _in_background(self, work) -> bool:
        if not self._busy.acquire(blocking=False):
            return False

        def run():
            try:
                work()
            finally:
                self._busy.release()

        threading.Thread(target=run, daemon=True).start()
        return True

    def _capture_and_log(self):
        saved = self.capture_now()
        if saved:
            self._log('info', f'dataset: captured {len(saved)} camera(s) — '
                              + ', '.join(f':{port}' for port in sorted(saved)))
        else:
            self._log('warn', f'dataset: capture saved nothing — no camera answered '
                              f'on {self.host}:{self.ports}')

    def _toggle_and_log(self):
        target, done = self.toggle_record_now()
        if target is None:
            self._log('warn', f'dataset: no camera answered on {self.host}:{self.ports}')
        else:
            self._log('info', f'dataset: recording {"ON" if target else "OFF"} on '
                              f'{len(done)} camera(s) — '
                              + ', '.join(f':{port}' for port in sorted(done)))
