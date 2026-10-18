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
