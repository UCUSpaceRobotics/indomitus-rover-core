from rover_cameras.modes import parse_formats_ext, pick_mode

# Trimmed `v4l2-ctl --list-formats-ext` of the arm camera.
ARM_CAMERA = """\
ioctl: VIDIOC_ENUM_FMT
\tType: Video Capture

\t[0]: 'MJPG' (Motion-JPEG, compressed)
\t\tSize: Discrete 1280x720
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t[1]: 'YUYV' (YUYV 4:2:2)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 1280x720
\t\t\tInterval: Discrete 0.100s (10.000 fps)
"""


def test_formats_are_kept_apart():
    f = parse_formats_ext(ARM_CAMERA)
    assert f['MJPG'][(1280, 720)] == [30.0]
    assert f['YUYV'][(1280, 720)] == [10.0]


def test_rate_snaps_down_never_up():
    w, h, fps, note = pick_mode({(960, 600): [15.0, 30.0, 60.0]}, 960, 600, 20)
    assert fps == 15.0 and 'not offered' in note
    # Only faster rates: the slowest one.
    assert pick_mode({(960, 600): [15.0, 30.0]}, 960, 600, 10)[2] == 15.0


def test_missing_size_falls_back_to_largest():
    w, h, _, _ = pick_mode({(640, 480): [30.0], (1920, 1200): [15.0]}, 800, 600, 15)
    assert (w, h) == (1920, 1200)
