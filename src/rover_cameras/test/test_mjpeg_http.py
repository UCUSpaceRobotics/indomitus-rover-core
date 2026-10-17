import threading

from rover_cameras.mjpeg_http import Dataset, FrameStore


def test_users_drive_the_valve():
    calls = []
    store = FrameStore(calls.append)
    store.acquire()
    store.acquire()
    store.release()
    assert calls == [True]
    store.release()
    assert calls == [True, False]


def test_fresh_waits_for_a_new_frame():
    store = FrameStore()
    store.push(b'old')
    since = store.seq
    threading.Timer(0.05, store.push, [b'new']).start()
    assert store.fresh(since, 2.0) == b'new'
    assert store.fresh(store.seq, 0.05) is None


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
