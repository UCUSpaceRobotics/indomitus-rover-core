"""Which capture modes a V4L2 camera offers, and snapping a request onto one.

Ported from indomitus-ground-station/cameras/camera_mjpeg_server.py
(enumerate_modes/pick_mode), with one change: modes are kept per pixel format.
The arm camera offers 1280x720 at 30 fps as MJPG but only at 10 as YUYV, so a
list merged across formats picks rates the chosen format does not have.
"""

import re
import subprocess


def parse_formats_ext(text):
    """{fourcc: {(w, h): [fps, ...]}} from `v4l2-ctl --list-formats-ext` output."""
    formats = {}
    fourcc = None
    size = None
    for line in text.splitlines():
        m = re.search(r"\[\d+\]: '(\w+)'", line)
        if m:
            fourcc = m.group(1)
            formats.setdefault(fourcc, {})
            size = None
            continue
        m = re.search(r'Size: Discrete (\d+)x(\d+)', line)
        if m and fourcc:
            size = (int(m.group(1)), int(m.group(2)))
            formats[fourcc].setdefault(size, [])
            continue
        m = re.search(r'\(([\d.]+) fps\)', line)
        if m and fourcc and size:
            formats[fourcc][size].append(float(m.group(1)))
    return formats


def enumerate_formats(device):
    """parse_formats_ext() of the device, or {} if v4l2-ctl cannot tell."""
    try:
        out = subprocess.check_output(
            ['v4l2-ctl', '-d', device, '--list-formats-ext'],
            stderr=subprocess.STDOUT, timeout=5).decode('utf-8', 'replace')
    except (OSError, subprocess.SubprocessError):
        return {}
    return parse_formats_ext(out)


def pick_mode(modes, want_w, want_h, want_fps):
    """Snap a request onto a mode the camera has. Returns (w, h, fps, note).

    `modes` is {(w, h): [fps, ...]} for one pixel format; empty means unknown,
    and the request is used as given.
    """
    if not modes:
        return want_w, want_h, want_fps, 'unverified (no format list)'

    notes = []
    if (want_w, want_h) in modes:
        size = (want_w, want_h)
    else:
        # Requested size is gone: fall back to the largest one on offer.
        size = max(modes, key=lambda s: s[0] * s[1])
        notes.append('%dx%d not offered' % (want_w, want_h))

    rates = sorted(modes[size])
    if not rates:
        return size[0], size[1], want_fps, '; '.join(notes + ['no rate list'])
    if want_fps in rates:
        fps = want_fps
    else:
        # Highest rate at or below the request, else the slowest on offer.
        # Never silently run the camera faster than was asked for.
        below = [r for r in rates if r <= want_fps]
        fps = below[-1] if below else rates[0]
        notes.append('%g fps not offered (have %s)'
                     % (want_fps, '/'.join('%g' % r for r in rates)))

    return size[0], size[1], fps, '; '.join(notes) or 'exact'
