import pytest

from rover_cameras import pipeline


def test_raw_camera_encodes_and_mjpg_passes_through():
    raw = pipeline.build('/dev/x', 'YUYV', 960, 600, 15, quality=7)
    assert 'format=YUY2' in raw and 'jpegenc quality=7' in raw
    mjpg = pipeline.build('/dev/x', 'MJPG', 640, 480, 30)
    assert 'image/jpeg' in mjpg and 'jpegenc' not in mjpg and 'jpegdec' in mjpg


def test_ros_branch_size_and_encoding():
    desc = pipeline.build('/dev/x', 'YUYV', 960, 600, 15, ros_encoding='bgr8',
                          ros_width=480, ros_height=300)
    assert 'video/x-raw,format=BGR,width=480,height=300 ! appsink name=ros_sink' in desc


def test_framerate():
    assert pipeline.framerate(15) == '15/1'
    assert pipeline.framerate(7.5) == '15/2'


def test_rejects_unknown_format():
    with pytest.raises(ValueError):
        pipeline.build('/dev/x', 'H264', 960, 600, 15)


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
