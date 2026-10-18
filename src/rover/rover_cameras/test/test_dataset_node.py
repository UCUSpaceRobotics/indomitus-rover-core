import pytest

dataset_node = pytest.importorskip('rover_cameras.dataset_node')


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
