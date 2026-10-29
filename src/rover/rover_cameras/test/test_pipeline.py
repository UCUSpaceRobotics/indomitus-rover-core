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


@pytest.mark.parametrize('view, crop', [('left', 'videocrop right=1280'),
                                        ('right', 'videocrop left=1280')])
def test_stereo_keeps_one_half(view, crop):
    desc = pipeline.build('/dev/x', 'YUYV', 2560, 720, 15, stereo=view)
    assert crop in desc


def test_stereo_both_is_the_joined_frame():
    assert 'videocrop' not in pipeline.build('/dev/x', 'YUYV', 2560, 720, 15)


def test_stereo_crops_before_turning():
    # The flip swaps the halves: `left` must be cut from the unturned frame.
    desc = pipeline.build('/dev/x', 'YUYV', 2560, 720, 15, stereo='left', rotate_180=True)
    assert desc.index('videocrop right=1280') < desc.index('videoflip')


def test_stereo_mjpg_decodes_once_and_reencodes():
    desc = pipeline.build('/dev/x', 'MJPG', 2560, 720, 15, stereo='right', rotate_180=True)
    source = desc.split('! tee')[0]
    assert source.count('jpegdec') == 1
    assert source.index('jpegdec') < source.index('videocrop')
    assert desc.count('jpegenc') == 2


def test_stereo_rejects_unknown_view():
    with pytest.raises(ValueError, match='stereo'):
        pipeline.build('/dev/x', 'YUYV', 2560, 720, 15, stereo='middle')


@pytest.mark.parametrize('view', ['left', 'right'])
def test_gstreamer_outputs_half_width(view):
    gi = pytest.importorskip('gi')
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
    pipe = Gst.parse_launch(pipeline.build(pipeline.TEST_DEVICE, 'YUYV', 640, 240, 15,
                                           stereo=view, rotate_180=True))
    sink = pipe.get_by_name('ros_sink')
    pipe.set_state(Gst.State.PLAYING)
    try:
        sample = sink.emit('try-pull-sample', 5 * Gst.SECOND)
        assert sample is not None
        caps = sample.get_caps().get_structure(0)
        assert (caps.get_value('width'), caps.get_value('height')) == (320, 240)
    finally:
        pipe.set_state(Gst.State.NULL)
