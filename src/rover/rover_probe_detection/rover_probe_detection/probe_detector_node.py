"""Runs the probe detector on each camera's image; publishes 2D detections.

    in   cameras/<camera>/image_raw           sensor_msgs/Image (rover_cameras)
    out  cameras/<camera>/probe_detections    vision_msgs/Detection2DArray
         (header = the image's: capture stamp, optical frame_id)

The detector backend is not implemented yet (see detector.py for the plan);
with backend 'none' this node subscribes to nothing and only says so.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from rover_probe_detection.detector import make_detector
from rover_probe_detection.ros_conversions import detection_to_msg, image_to_rgb


class ProbeDetectorNode(Node):

    def __init__(self):
        super().__init__('probe_detector')
        cameras = self.declare_parameter('cameras', ['arducam_mast']).value
        image_topic = self.declare_parameter('image_topic', 'cameras/{camera}/image_raw').value
        det_topic = self.declare_parameter(
            'detections_topic', 'cameras/{camera}/probe_detections').value
        backend = self.declare_parameter('backend', 'none').value
        model_path = self.declare_parameter('model_path', '').value
        self.min_score = self.declare_parameter('min_score', 0.3).value

        self.detector = make_detector(backend, model_path=model_path)
        if self.detector is None:
            self.get_logger().warn(
                "detector backend 'none': no model yet, publishing nothing "
                '(see rover_probe_detection/detector.py for the plan)')
            return

        self._pubs = {}
        for cam in cameras:
            self._pubs[cam] = self.create_publisher(
                Detection2DArray, det_topic.format(camera=cam), 10)
            # Best effort, depth 1: always the newest frame, never a backlog.
            self.create_subscription(
                Image, image_topic.format(camera=cam),
                lambda msg, cam=cam: self._on_image(cam, msg), qos_profile_sensor_data)

    def _on_image(self, camera, msg):
        out = Detection2DArray()
        out.header = msg.header
        for det in self.detector.detect(image_to_rgb(msg)):
            if det.score >= self.min_score:
                out.detections.append(detection_to_msg(det, msg.header))
        self._pubs[camera].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ProbeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
