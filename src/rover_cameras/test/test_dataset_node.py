import pytest

dataset_node = pytest.importorskip('rover_cameras.dataset_node')


def test_capture_all_saved():
    ok, msg = dataset_node.summarize_capture({'a': (True, '/p/a.jpg'), 'b': (True, '/p/b.jpg')})
    assert ok and msg == 'saved 2: a=/p/a.jpg, b=/p/b.jpg'


def test_capture_skips_cameras_that_are_not_mounted():
    ok, msg = dataset_node.summarize_capture({
        'a': (True, '/p/a.jpg'), 'b': (False, 'not capturing'), 'c': (False, 'not running')})
    assert ok and msg == 'saved 1: a=/p/a.jpg; skipped: b, c'


def test_capture_fails_when_a_live_camera_fails_or_nothing_saved():
    ok, msg = dataset_node.summarize_capture({'a': (True, '/p/a.jpg'), 'b': (False, 'timeout')})
    assert not ok and 'failed: b (timeout)' in msg
    ok, _ = dataset_node.summarize_capture({'b': (False, 'not capturing')})
    assert not ok
    ok, _ = dataset_node.summarize_capture({})
    assert not ok


def test_record_summary():
    ok, msg = dataset_node.summarize_record(
        {'a': (True, 'recording ON'), 'b': (False, 'not running')}, True)
    assert ok and msg == 'recording ON (1 camera)'
    ok, msg = dataset_node.summarize_record({'a': (False, 'timeout')}, False)
    assert not ok and msg == 'recording OFF (0 cameras); failed: a (timeout)'


def test_aggregate():
    s = dataset_node.DatasetState
    agg = dataset_node.aggregate([s(recording=False, cameras_live=1, saved=3),
                                  s(recording=True, cameras_live=0, saved=4)])
    assert agg.recording and agg.cameras_live == 1 and agg.saved == 7
    assert not dataset_node.aggregate([]).recording
