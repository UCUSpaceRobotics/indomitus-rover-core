"""One camera: GStreamer capture, MJPEG over HTTP to the GS, Image topics on the rover.

    ~/image_raw      sensor_msgs/Image, stamped with the capture time
    ~/camera_info    sensor_msgs/CameraInfo (K = 0 when there is no calibration)
    ~/capture        std_srvs/Trigger: save the next frame; message = its path
    ~/record         std_srvs/SetBool: save one frame every record_interval s, or stop
    ~/dataset_state  indomitus_interfaces/DatasetState, latched, on every change

Dataset frames go to <save_dir>/<date>/<name>/ at save_quality. dataset_node
calls ~/capture and ~/record of every camera for "all cameras at once".

The video the operators watch never goes through DDS (images over DDS on the
Wi-Fi lagged badly): it is the HTTP MJPEG of mjpeg_http.py, same URLs as the
ground station's camera_mjpeg_server.py. The topics are for nodes on the rover
(perception), at `ros_fps` and `ros_width`x`ros_height`, not the capture rate.
See pipeline.py for the pipeline.

One process per camera on purpose: these cameras wedge, and a wedged one then
takes down only its own feed. The node survives it: a pipeline error, an
unplugged device or `stall_timeout` without frames tears the pipeline down and
rebuilds it `reopen_delay` later, waiting for the device to come back.
"""

import array
import os
import signal
import time

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import GLib, Gst, GstVideo  # noqa: E402

import rclpy  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

from indomitus_interfaces.msg import DatasetState  # noqa: E402

from rover_cameras import calibration, modes, pipeline  # noqa: E402
from rover_cameras.mjpeg_http import Dataset, FrameStore, serve  # noqa: E402


# Latched: a late subscriber (dataset_node, the GS) gets the current state.
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera')
        p = self.declare_parameter
        self.cam_name = p('name', self.get_name()).value
        self.device = p('device', '/dev/video0').value
        self.fourcc = p('format', 'YUYV').value.upper()
        self.want_w = p('width', 960).value
        self.want_h = p('height', 600).value
        self.want_fps = float(p('fps', 15.0).value)
        self.quality = p('quality', 60).value
        self.jpeg_encoder = p('jpeg_encoder', 'jpegenc').value
        self.ros_fps = float(p('ros_fps', 5.0).value)
        self.ros_w = p('ros_width', 0).value
        self.ros_h = p('ros_height', 0).value
        self.ros_encoding = p('ros_encoding', 'rgb8').value
        self.rotate_180 = bool(p('rotate_180', False).value)
        self.stereo = p('stereo', 'both').value.lower()
        self.frame_id = p('frame_id', 'camera_optical_frame').value
        self.camera_info_file = p('camera_info_file', '').value
        port = p('port', 0).value
        bind = p('bind', '0.0.0.0').value
        save_dir = p('save_dir', '~/dataset').value
        self.save_quality = p('save_quality', 95).value
        record_interval = p('record_interval', 1.0).value
        self.warmup_frames = p('warmup_frames', 3).value
        self.stall_timeout = p('stall_timeout', 5.0).value
        self.reopen_delay = p('reopen_delay', 2.0).value

        # Depth 1, reliable: a local subscriber of either reliability matches,
        # and nobody gets a backlog of stale frames.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.image_pub = self.create_publisher(Image, '~/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, '~/camera_info', qos)

        self.calib = None
        if self.camera_info_file:
            try:
                self.calib = calibration.load(self.camera_info_file)
            except (OSError, KeyError, TypeError, ValueError) as exc:
                self.get_logger().error('cannot read calibration %s: %s'
                                        % (self.camera_info_file, exc))
        if self.calib is None:
            self.get_logger().warn('no calibration: camera_info has K = 0')

        Gst.init(None)
        # Capture time = base time + PTS on this clock, so make it wall time,
        # the clock ROS stamps use.
        self.clock = Gst.SystemClock.obtain()
        self.clock.set_property('clock-type', Gst.ClockType.REALTIME)

        self.pipe = None
        self.mode = 'closed'
        self.frames = 0
        self.ros_frames = 0
        self.errors = 0
        self.started = time.time()
        self._retry_at = 0.0
        self._last_frame = 0.0
        self._warmup_left = 0
        self._last_ros_pts = None
        self._video_info = None
        self._info_msg = None
        self._waiting_logged = False

        self.stream = FrameStore(lambda on: self._set_valve('stream', on))
        self.save = FrameStore(lambda on: self._set_valve('save', on))
        self.state_pub = self.create_publisher(DatasetState, '~/dataset_state', STATE_QOS)
        self.dataset = Dataset(self.cam_name, self.save, save_dir, record_interval,
                               self.get_logger().info, on_change=self._publish_state)
        self._publish_state()
        # Own group: a capture waits up to 3 s for a frame and must not hold up
        # _supervise. Dataset is thread-safe (the HTTP threads used it before).
        services = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '~/capture', self._on_capture_srv, callback_group=services)
        self.create_service(SetBool, '~/record', self._on_record_srv, callback_group=services)

        self.http = None
        if port:
            self.http = serve(bind, port, self.stream, self.health)
            self.get_logger().info('serving http://%s:%d/%s?action=stream'
                                   % (bind, port, self.cam_name))

        self.create_timer(0.5, self._supervise)

    # ------------------------------------------------------------ pipeline --

    def _open(self):
        test = self.device == pipeline.TEST_DEVICE
        if not test and not os.path.exists(self.device):
            if not self._waiting_logged:
                self.get_logger().warn('%s not present, waiting for it' % self.device)
                self._waiting_logged = True
            self._retry_at = time.monotonic() + self.reopen_delay
            return
        self._waiting_logged = False

        if test:
            w, h, fps, note = self.want_w, self.want_h, self.want_fps, 'test source'
        else:
            offered = modes.enumerate_formats(self.device)
            if offered and self.fourcc not in offered:
                self.get_logger().error('%s offers no %s (has %s)' % (
                    self.device, self.fourcc, ', '.join(offered)))
            w, h, fps, note = modes.pick_mode(offered.get(self.fourcc, {}),
                                              self.want_w, self.want_h, self.want_fps)
        desc = pipeline.build(
            self.device, self.fourcc, w, h, fps, quality=self.quality,
            save_quality=self.save_quality, jpeg_encoder=self.jpeg_encoder,
            ros_encoding=self.ros_encoding, ros_width=self.ros_w, ros_height=self.ros_h,
            rotate_180=self.rotate_180, stereo=self.stereo)
        try:
            pipe = Gst.parse_launch(desc)
        except GLib.Error as exc:
            self.get_logger().error('bad pipeline (%s): %s' % (exc.message, desc))
            self._retry_at = time.monotonic() + self.reopen_delay
            return

        pipe.use_clock(self.clock)
        pipe.get_by_name('t').get_static_pad('sink').add_probe(
            Gst.PadProbeType.BUFFER, self._on_capture)
        pipe.get_by_name('ros_queue').get_static_pad('sink').add_probe(
            Gst.PadProbeType.BUFFER, self._ros_rate_limit)
        pipe.get_by_name('ros_sink').connect('new-sample', self._on_ros_sample)
        for branch, store in (('stream', self.stream), ('save', self.save)):
            pipe.get_by_name(branch + '_sink').connect(
                'new-sample', self._on_jpeg_sample, store)
            pipe.get_by_name(branch + '_valve').set_property('drop', store.users == 0)

        self._warmup_left = self.warmup_frames
        self._last_ros_pts = None
        self._video_info = None
        self._last_frame = time.monotonic()     # stall grace starts now
        self.pipe = pipe
        self.mode = '%s %dx%d@%g' % (self.fourcc, w, h, fps)
        if self.stereo != 'both':
            self.mode += ' ' + self.stereo
        if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            msg = pipe.get_bus().pop_filtered(Gst.MessageType.ERROR)
            self._close('cannot start: %s' % (msg.parse_error()[0].message if msg else '?'))
            return
        self.get_logger().info('capturing %s from %s (%s)' % (self.mode, self.device, note))
        self._publish_state()

    def _close(self, reason):
        self.get_logger().warn('closing %s: %s' % (self.device, reason))
        self.errors += 1
        pipe, self.pipe = self.pipe, None
        if pipe is not None:
            pipe.set_state(Gst.State.NULL)
        self.mode = 'closed'
        self._retry_at = time.monotonic() + self.reopen_delay
        self._publish_state()

    def _supervise(self):
        if self.pipe is None:
            if time.monotonic() >= self._retry_at:
                self._open()
            return
        bus = self.pipe.get_bus()
        while True:
            msg = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS
                                   | Gst.MessageType.WARNING)
            if msg is None:
                break
            if msg.type == Gst.MessageType.WARNING:
                warn, _ = msg.parse_warning()
                self.get_logger().warn('gstreamer: %s' % warn.message)
            elif msg.type == Gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                self._close('%s (%s)' % (err.message, debug))
                return
            else:
                self._close('end of stream')
                return
        idle = time.monotonic() - self._last_frame
        if idle > self.stall_timeout:
            self._close('no frames for %.0f s' % idle)

    def _set_valve(self, branch, on):
        pipe = self.pipe
        if pipe is not None:
            pipe.get_by_name(branch + '_valve').set_property('drop', not on)

    # -------------------------------------------------- streaming threads --

    def _on_capture(self, pad, info):
        """Every captured frame, before the tee: warm-up drop and liveness."""
        # Arducams on SuperSpeed hand back an all-zero first frame on every
        # other open; drop a few after each open.
        if self._warmup_left > 0:
            self._warmup_left -= 1
            return Gst.PadProbeReturn.DROP
        self._last_frame = time.monotonic()
        self.frames += 1
        return Gst.PadProbeReturn.OK

    def _ros_rate_limit(self, pad, info):
        if self.ros_fps <= 0:
            return Gst.PadProbeReturn.OK
        pts = info.get_buffer().pts
        if pts == Gst.CLOCK_TIME_NONE:
            return Gst.PadProbeReturn.OK
        period = 1e9 / self.ros_fps
        # 10 % slack, so a 15 fps camera gives exactly every 3rd frame at 5 fps.
        if self._last_ros_pts is not None and pts - self._last_ros_pts < 0.9 * period:
            return Gst.PadProbeReturn.DROP
        self._last_ros_pts = pts
        return Gst.PadProbeReturn.OK

    def _on_jpeg_sample(self, sink, store):
        sample = sink.emit('pull-sample')
        if sample is not None:
            buf = sample.get_buffer()
            store.push(buf.extract_dup(0, buf.get_size()))
        return Gst.FlowReturn.OK

    def _on_ros_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        if self._video_info is None:
            # camera_info first: if it raises, the next frame retries.
            vi = GstVideo.VideoInfo.new_from_caps(sample.get_caps())
            self._info_msg = self._camera_info(vi.width, vi.height)
            self._video_info = vi
        vi = self._video_info

        if buf.pts != Gst.CLOCK_TIME_NONE:
            stamp_ns = sink.get_base_time() + buf.pts
            stamp = Time(nanoseconds=stamp_ns).to_msg()
        else:
            stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = vi.height
        msg.width = vi.width
        msg.encoding = self.ros_encoding
        msg.step = vi.stride[0]
        # One copy out of the GstBuffer; array('B') skips rclpy's per-byte check.
        msg.data = array.array('B', buf.extract_dup(0, msg.step * vi.height))
        self._info_msg.header = msg.header
        self.image_pub.publish(msg)
        self.info_pub.publish(self._info_msg)
        self.ros_frames += 1
        return Gst.FlowReturn.OK

    def _camera_info(self, width, height):
        fields = (calibration.scaled(self.calib, width, height) if self.calib
                  else calibration.uncalibrated(width, height))
        if self.calib and abs(width / height - self.calib['width'] / self.calib['height']) > 0.01:
            self.get_logger().error('published %dx%d has another aspect than the '
                                    'calibration (%dx%d): camera_info is wrong'
                                    % (width, height, self.calib['width'], self.calib['height']))
        info = CameraInfo()
        info.width = fields['width']
        info.height = fields['height']
        info.distortion_model = fields['distortion_model']
        info.d = fields['d']
        info.k = fields['k']
        info.r = fields['r']
        info.p = fields['p']
        return info

    # ------------------------------------------------------------- dataset --

    def _on_capture_srv(self, request, response):
        if self.pipe is None:
            response.success, response.message = False, 'not capturing'
            return response
        path = self.dataset.capture()
        response.success = path is not None
        response.message = path if path is not None else 'no frame'
        return response

    def _on_record_srv(self, request, response):
        self.dataset.set_recording(request.data)
        response.success = True
        response.message = 'recording %s' % ('ON' if self.dataset.recording else 'OFF')
        return response

    def _publish_state(self):
        # Also called from Dataset's writer thread, which can outlive shutdown.
        if not rclpy.ok():
            return
        msg = DatasetState()
        msg.recording = self.dataset.recording
        msg.cameras_live = int(self.pipe is not None)
        msg.saved = self.dataset.saved
        self.state_pub.publish(msg)

    # ---------------------------------------------------------------- misc --

    def health(self):
        up = time.time() - self.started
        # ros_frames before frames: start-cameras.sh reads `.*frames=`, greedy,
        # so the last `frames=` on the line must be the capture count.
        return ('ok name=%s mode=%s state=%s viewers=%d ros_frames=%d frames=%d errors=%d '
                'uptime=%.0fs fps=%.1f recording=%d saved=%d'
                % (self.cam_name, self.mode.replace(' ', '_'),
                   'capturing' if self.pipe is not None else 'waiting',
                   self.stream.users, self.ros_frames, self.frames, self.errors, up,
                   self.frames / up if up else 0, self.dataset.recording, self.dataset.saved))

    def shutdown(self):
        if self.http is not None:
            self.http.shutdown()
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)
            self.pipe = None


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Own signal handling: rclpy's would invalidate the context at once, while
    # GStreamer threads are still publishing. Stop the pipeline first.
    # SIGINT explicitly too: a process started in the background of a
    # non-interactive shell inherits SIGINT ignored, and Python then installs
    # no handler of its own.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node = CameraNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
