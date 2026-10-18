"""MJPEG over HTTP for the ground station, plus dataset capture. No ROS, no GStreamer.

Same video URLs as indomitus-ground-station/cameras/camera_mjpeg_server.py, so
the GS UI tiles work unchanged:

    /<name>?action=stream  multipart/x-mixed-replace MJPEG
    /?action=snapshot      the newest JPEG
    /health                one line of key=value, for scripts

Dataset capture (Dataset) is driven over ROS services by camera_node, not over
HTTP: see camera_node.py and dataset_node.py.

A FrameStore holds the newest JPEG of one pipeline branch and counts its users;
the node closes the branch's valve when the count drops to zero, so nothing is
encoded while nobody watches or records.
"""

import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

BOUNDARY = 'cameramjpegframe'


class FrameStore:
    """The newest frame of one branch, with a user count driving `on_active`."""

    def __init__(self, on_active=None):
        self._on_active = on_active or (lambda active: None)
        self._cond = threading.Condition()
        self._frame = None
        self._users = 0
        self.seq = 0

    @property
    def users(self):
        return self._users

    def acquire(self):
        with self._cond:
            self._users += 1
            first = self._users == 1
        if first:
            self._on_active(True)

    def release(self):
        with self._cond:
            self._users = max(0, self._users - 1)
            last = self._users == 0
        if last:
            self._on_active(False)

    def push(self, frame):
        with self._cond:
            self._frame = frame
            self.seq += 1
            self._cond.notify_all()

    def latest(self, since, timeout=5.0):
        """(frame, seq) once newer than `since`; the current one on timeout."""
        with self._cond:
            if self.seq <= since:
                self._cond.wait(timeout)
            return self._frame, self.seq

    def fresh(self, since, timeout):
        """First frame after sequence `since`, or None on timeout."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while self.seq <= since:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self._cond.wait(left)
            return self._frame


class Dataset:
    """Writes JPEGs from the save branch to <save_dir>/<date>/<name>/.

    Holds the branch open (as a user) while recording and for the moment a
    capture takes. Disk writes run on a worker thread, never in GStreamer's.
    """

    def __init__(self, name, frames, save_dir, interval, log, on_change=None):
        self.name = name
        self.frames = frames
        self.save_dir = os.path.expanduser(save_dir)
        self.interval = float(interval)
        self.recording = False
        self.saved = 0
        self._log = log
        self._on_change = on_change or (lambda: None)
        self._since = 0
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._queue = queue.Queue(maxsize=64)
        threading.Thread(target=self._writer, daemon=True).start()
        threading.Thread(target=self._recorder, daemon=True).start()

    def capture(self, timeout=3.0):
        """Save the next frame; its path, or None if none arrived."""
        since = self.frames.seq
        self.frames.acquire()
        try:
            jpeg = self.frames.fresh(since, timeout)
        finally:
            self.frames.release()
        return None if jpeg is None else self._enqueue(jpeg)

    def set_recording(self, on):
        with self._lock:
            if on == self.recording:
                return
            self.recording = on
            if on:
                self._since = self.frames.seq
                self.frames.acquire()
            else:
                self.frames.release()
        self._log('recording %s (every %gs to %s)'
                  % ('ON' if on else 'OFF', self.interval, self.save_dir))
        self._wake.set()
        self._on_change()

    def _recorder(self):
        while True:
            self._wake.wait(self.interval)
            self._wake.clear()
            if not self.recording:
                continue
            # A frame is only worth saving once.
            if self.frames.seq > self._since:
                jpeg, self._since = self.frames.latest(self._since, 0)
                self._enqueue(jpeg)

    def _enqueue(self, jpeg):
        # Named at capture time, so a backed-up queue cannot skew the names.
        now = time.time()
        local = time.localtime(now)
        path = os.path.join(
            self.save_dir, time.strftime('%Y-%m-%d', local), self.name,
            '%s_%s-%03d.jpg' % (self.name, time.strftime('%Y%m%d-%H%M%S', local),
                                int(now * 1000) % 1000))
        try:
            self._queue.put_nowait((path, jpeg))
        except queue.Full:
            self._log('dataset write queue full, dropping %s' % path)
            return None
        return path

    def _writer(self):
        while True:
            path, jpeg = self._queue.get()
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'wb') as f:
                    f.write(jpeg)
                self.saved += 1
            except OSError as exc:
                self._log('cannot save %s: %s' % (path, exc))
                continue
            self._on_change()


class Handler(BaseHTTPRequestHandler):
    """Reads `self.server.stream` (FrameStore) and `.health()`."""

    protocol_version = 'HTTP/1.0'

    def log_message(self, fmt, *args):
        pass        # one line per frame is not a log

    def do_GET(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        action = query.get('action', [''])[0]
        if parsed.path == '/health':
            self._text(200, self.server.health() + '\n')
        elif action == 'snapshot':
            self._snapshot()
        elif action == 'stream' or parsed.path in ('/', '/stream'):
            self._stream()
        else:
            self.send_error(404)

    def _text(self, code, text):
        body = text.encode()
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        # The GS UI reads these cross-origin.
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _snapshot(self):
        stream = self.server.stream
        # Open the branch for a moment if nobody is watching.
        since = stream.seq
        stream.acquire()
        try:
            jpeg = stream.fresh(since, 3.0)
        finally:
            stream.release()
        if jpeg is None:
            self.send_error(503, 'no frame')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(jpeg)

    def _stream(self):
        stream = self.server.stream
        # The connection itself is the "someone is watching" signal.
        stream.acquire()
        try:
            self.send_response(200)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=' + BOUNDARY)
            self.end_headers()
            seq = stream.seq
            while True:
                jpeg, new_seq = stream.latest(seq)
                if jpeg is None or new_seq == seq:
                    continue
                seq = new_seq
                self.wfile.write(b'--' + BOUNDARY.encode() + b'\r\n'
                                 b'Content-Type: image/jpeg\r\n'
                                 b'Content-Length: %d\r\n\r\n' % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass        # the operator closed a tile
        finally:
            stream.release()


def serve(bind, port, stream, health):
    """Start the HTTP server on a daemon thread; returns the server."""
    server = ThreadingHTTPServer((bind, port), Handler)
    server.daemon_threads = True
    server.stream = stream
    server.health = health
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
