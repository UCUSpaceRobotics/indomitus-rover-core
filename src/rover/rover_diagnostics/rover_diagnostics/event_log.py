"""
Append-only JSONL event log with size-based rotation.
"""

import datetime
import json
import math
import os


def _sanitize(value):
    """Replace non-finite floats with None."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v) for v in value]
    return value


class EventLog:
    """Writes one JSON object per line to a per-session, rotating file."""

    def __init__(self, directory, prefix='faults', max_bytes=5 * 1024 * 1024,
                 backup_count=3):
        self._dir = os.path.expanduser(directory)
        self._max_bytes = max_bytes
        self._backup_count = backup_count
        self._file = None

        os.makedirs(self._dir, exist_ok=True)

        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%d_%H-%M-%S-%f')[:-3]
        self._path = os.path.join(self._dir, '{}_{}.jsonl'.format(prefix, stamp))

        suffix = 1
        while os.path.exists(self._path):
            self._path = os.path.join(
                self._dir, '{}_{}-{}.jsonl'.format(prefix, stamp, suffix))
            suffix += 1

        self._open()

    def _open(self):
        self._file = open(self._path, 'a', encoding='utf-8')

    def _rotate_if_needed(self):
        if self._max_bytes <= 0:
            return
        if self._file.tell() < self._max_bytes:
            return

        self._file.close()
        # Shift .2 -> .3, .1 -> .2, base -> .1; drop whatever falls off the end.
        for index in range(self._backup_count, 0, -1):
            src = self._path if index == 1 else '{}.{}'.format(self._path, index - 1)
            dst = '{}.{}'.format(self._path, index)
            if os.path.exists(src):
                os.replace(src, dst)
        self._open()

    def write(self, event):
        """Append one event dict, then flush and fsync it to disk."""
        line = json.dumps(_sanitize(event), separators=(',', ':'), sort_keys=False)
        self._file.write(line + '\n')
        self._file.flush()
        os.fsync(self._file.fileno())
        self._rotate_if_needed()

    def close(self):
        if self._file is not None and not self._file.closed:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()

    @property
    def path(self):
        return self._path
