"""Stand-in for probe_detector_node: projects known probe positions into each camera.

For testing the triangulation (and anything downstream) without a model. For
each camera with a calibrated camera_info, every probe in front of it and inside
the image is published as a detection at its projected pixel, plus Gaussian
`pixel_noise`, on the same topic the detector uses.

    probes:        [x1, y1, z1, x2, y2, z2, ...] in probes_frame
    probes_frame:  default base_footprint
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray

from rover_probe_detection import geometry
from rover_probe_detection.detector import Detection
from rover_probe_detection.ros_conversions import detection_to_msg, transform_to_rt


class FakeDetectionsNode(Node):

    def __init__(self):
        super().__init__('fake_probe_detections')
        p = self.declare_parameter
        self.cameras = p('cameras', ['arducam_mast', 'arducam_fr_bottom', 'arm_camera']).value
        info_topic = p('camera_info_topic', 'cameras/{camera}/camera_info').value
        det_topic = p('detections_topic', 'cameras/{camera}/probe_detections').value
        flat = p('probes', [1.5, 0.0, 0.0]).value
        self.frame = p('probes_frame', 'base_footprint').value
        self.noise = p('pixel_noise', 0.5).value
        self.box = p('box_size', 30.0).value
        rate = p('rate', 5.0).value
        if len(flat) % 3:
            raise ValueError('probes must be x, y, z triples')
        self.probes = np.array(flat, dtype=float).reshape(-1, 3)
        self.rng = np.random.default_rng()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.infos = {}
        self.pubs = {}
        for cam in self.cameras:
            self.create_subscription(
                CameraInfo, info_topic.format(camera=cam),
                lambda msg, cam=cam: self.infos.__setitem__(cam, msg), qos_profile_sensor_data)
            self.pubs[cam] = self.create_publisher(
                Detection2DArray, det_topic.format(camera=cam), 10)
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        for cam in self.cameras:
            info = self.infos.get(cam)
            if info is None or info.k[0] == 0.0:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(info.header.frame_id, self.frame, Time())
            except TransformException:
                continue
            rot, t = transform_to_rt(tf)
            header = Header(frame_id=info.header.frame_id, stamp=tf.header.stamp)
            if Time.from_msg(tf.header.stamp).nanoseconds == 0:       # static transforms
                header.stamp = self.get_clock().now().to_msg()
            out = Detection2DArray(header=header)
            for probe in self.probes:
                uv = geometry.project(rot @ probe + t, info.k, info.d, info.distortion_model)
                if uv is None:
                    continue
                u, v = np.array(uv) + self.rng.normal(0.0, self.noise, 2)
                if not (0 <= u < info.width and 0 <= v < info.height):
                    continue
                h = self.box / 2.0
                out.detections.append(detection_to_msg(
                    Detection(u - h, v - h, u + h, v + h, score=0.9), header))
            self.pubs[cam].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetectionsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
