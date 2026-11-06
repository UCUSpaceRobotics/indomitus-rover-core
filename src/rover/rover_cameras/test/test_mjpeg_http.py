import threading

from rover_cameras.mjpeg_http import Dataset, FrameStore


def test_capture_writes_the_next_frame(tmp_path):
    store = FrameStore(lambda on: on and threading.Timer(0.05, store.push, [b'jpeg']).start())
    ds = Dataset('cam', store, str(tmp_path), 1.0, print)
    path = ds.capture(timeout=2.0)
    assert path is not None and '/cam/cam_' in path
    for _ in range(100):
        if ds.saved:
            break
        threading.Event().wait(0.02)
    assert open(path, 'rb').read() == b'jpeg'


def test_recording_asks_for_each_frame_it_saves(tmp_path):
    # A store that only delivers on demand, like the gated save branch.
    store = FrameStore(on_demand=lambda: threading.Timer(
        0.01, store.push, [b'\x89PNG frame']).start())
    ds = Dataset('cam', store, str(tmp_path), 0.05, print)
    ds.set_recording(True)
    for _ in range(100):
        if ds.saved >= 3:
            break
        threading.Event().wait(0.02)
    ds.set_recording(False)
    assert ds.saved >= 3
    assert all(p.suffix == '.png' for p in tmp_path.rglob('cam_*'))
