import pytest

from rover_cameras import pipeline


def test_raw_camera_encodes_and_mjpg_passes_through():
    raw = pipeline.build('/dev/x', 'YUYV', 960, 600, 15, quality=7)
    assert 'format=YUY2' in raw and 'jpegenc quality=7' in raw
    mjpg = pipeline.build('/dev/x', 'MJPG', 640, 480, 30)
    assert 'image/jpeg' in mjpg and 'jpegenc' not in mjpg and 'jpegdec' in mjpg


@pytest.mark.parametrize('fourcc', ['YUYV', 'MJPG'])
def test_gstreamer_accepts_it(fourcc):
    gi = pytest.importorskip('gi')
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
    pipe = Gst.parse_launch(pipeline.build(pipeline.TEST_DEVICE, fourcc, 320, 240, 15))
    for name in ('t', 'ros_queue', 'ros_sink', 'stream_valve', 'stream_sink',
                 'save_valve', 'save_sink'):
        assert pipe.get_by_name(name) is not None, name
