"""The GStreamer pipeline of one camera, as a gst-launch string.

    source ─ caps ─ tee ─┬─ queue ─ valve ─ [videoconvert ─ jpegenc] ─ appsink  stream
                         ├─ queue ─ valve ─ [videoconvert ─ jpegenc] ─ appsink  dataset
                         └─ queue ─ [jpegdec] ─ videoconvert ─ videoscale ─ appsink  ros

- A camera that already sends MJPG has its JPEGs passed through to the stream
  and dataset branches as they are (no encode), and decoded only for ROS.
- The valves keep the JPEG branches from encoding while nobody watches or
  records; the node opens them on demand. The ROS branch always runs.
- Every branch has a leaky queue, so a slow consumer drops its own frames
  instead of stalling the camera or the other branches.
- The ROS branch's rate is cut by a pad probe on its queue (in the node), not
  by videorate: videorate re-stamps buffers onto a perfect grid, and the
  capture time is what the image header has to carry.
- `jpeg_encoder` is a plain element name, so a hardware encoder (nvjpegenc)
  can be swapped in by config once the board has one.
- A side-by-side stereo camera (ZED 2i as UVC) can be cut to one sensor with
  `stereo`; the crop sits before the tee, so every branch sees the same half.
"""

from fractions import Fraction

# V4L2 FourCC -> GStreamer raw format. MJPG is handled separately.
RAW_FORMATS = {
    'YUYV': 'YUY2',
    'UYVY': 'UYVY',
    'GREY': 'GRAY8',
    'NV12': 'NV12',
}

# sensor_msgs/Image encoding -> GStreamer raw format of the ROS branch.
ROS_ENCODINGS = {
    'rgb8': 'RGB',
    'bgr8': 'BGR',
    'mono8': 'GRAY8',
}

# Which part of a side-by-side stereo frame to keep; 'both' = the whole frame.
STEREO_VIEWS = ('both', 'left', 'right')

TEST_DEVICE = 'videotestsrc'

BRANCH_QUEUE = 'queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0'
APPSINK_PROPS = 'emit-signals=true max-buffers=1 drop=true sync=false async=false'


def framerate(fps):
    """GStreamer framerate caps value for a (possibly fractional) fps."""
    f = Fraction(fps).limit_denominator(1001)
    return '%d/%d' % (f.numerator, f.denominator)


def _source(device, fourcc, width, height, fps):
    size = 'width=%d,height=%d,framerate=%s' % (width, height, framerate(fps))
    if device == TEST_DEVICE:
        # A stand-in for a camera, for testing without hardware. MJPG is
        # emulated with an encoder so the branches see what a real one sends.
        if fourcc == 'MJPG':
            return ('videotestsrc is-live=true pattern=ball ! video/x-raw,%s '
                    '! jpegenc ! image/jpeg' % size)
        return 'videotestsrc is-live=true pattern=ball ! video/x-raw,format=%s,%s' % (
            RAW_FORMATS[fourcc], size)
    if fourcc == 'MJPG':
        return 'v4l2src device=%s ! image/jpeg,%s' % (device, size)
    return 'v4l2src device=%s ! video/x-raw,format=%s,%s' % (device, RAW_FORMATS[fourcc], size)


def _stereo_crop(view, width):
    """videocrop keeping one sensor of a side-by-side frame `width` px wide."""
    # Cut the other sensor's half off; the left one is on the left.
    cut = 'right' if view == 'left' else 'left'
    return ' ! videocrop %s=%d' % (cut, width // 2)


def _jpeg_branch(name, fourcc, encoder, quality):
    encode = '' if fourcc == 'MJPG' else ' ! videoconvert ! %s quality=%d' % (encoder, quality)
    return ('t. ! %s ! valve name=%s_valve drop=true drop-mode=forward-sticky-events%s '
            '! appsink name=%s_sink %s' % (BRANCH_QUEUE, name, encode, name, APPSINK_PROPS))


def _ros_branch(fourcc, encoding, width, height):
    decode = ' ! jpegdec' if fourcc == 'MJPG' else ''
    caps = 'video/x-raw,format=%s' % ROS_ENCODINGS[encoding]
    if width and height:
        caps += ',width=%d,height=%d' % (width, height)
    return ('t. ! %s name=ros_queue%s ! videoconvert ! videoscale ! %s '
            '! appsink name=ros_sink %s' % (BRANCH_QUEUE, decode, caps, APPSINK_PROPS))


def build(device, fourcc, width, height, fps, *, quality=60, save_quality=95,
          jpeg_encoder='jpegenc', ros_encoding='rgb8', ros_width=0, ros_height=0,
          rotate_180=False, stereo='both'):
    """The gst-launch string for one camera; see the module docstring."""
    if fourcc != 'MJPG' and fourcc not in RAW_FORMATS:
        raise ValueError('unsupported format %r (have MJPG, %s)'
                         % (fourcc, ', '.join(RAW_FORMATS)))
    if ros_encoding not in ROS_ENCODINGS:
        raise ValueError('unsupported ros_encoding %r (have %s)'
                         % (ros_encoding, ', '.join(ROS_ENCODINGS)))
    if stereo not in STEREO_VIEWS:
        raise ValueError('unsupported stereo %r (have %s)'
                         % (stereo, ', '.join(STEREO_VIEWS)))
    source = _source(device, fourcc, width, height, fps)
    branch_fourcc = fourcc
    if stereo != 'both' or rotate_180:
        if fourcc == 'MJPG':
            source += ' ! jpegdec'
        branch_fourcc = 'RAW'
    if stereo != 'both':
        source += _stereo_crop(stereo, width)
    if rotate_180:
        source += ' ! videoflip method=rotate-180'
    return ' '.join([
        source, '! tee name=t',
        _jpeg_branch('stream', branch_fourcc, jpeg_encoder, quality),
        _jpeg_branch('save', branch_fourcc, jpeg_encoder, save_quality),
        _ros_branch(branch_fourcc, ros_encoding, ros_width, ros_height),
    ])
