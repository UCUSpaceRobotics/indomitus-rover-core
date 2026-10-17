"""Probe positions in 3D from the 2D detections of several cameras.

    in   cameras/<camera>/probe_detections   vision_msgs/Detection2DArray
         cameras/<camera>/camera_info        sensor_msgs/CameraInfo (K must be calibrated)
         /tf                                 camera optical frames, at each detection's stamp
    out  probes          vision_msgs/Detection3DArray in `fixed_frame`
         probes/markers  visualization_msgs/MarkerArray (RViz)

Every `rate` Hz, each camera's newest detections (if younger than `max_age`)
become rays in `fixed_frame`, using TF at the detection's own capture stamp, so
cameras that captured at slightly different moments still agree. Rays of 2+
cameras meeting within `max_ray_gap` are one probe, placed by least squares
(geometry.triangulate). With `single_view: ground`, a probe only one camera
sees is placed where its ray meets the ground (base_footprint's z = 0 plane,
raised by `point_height`, the height of the triangulated image point).

Each detection's point is its bbox centre for now (keypoints: see detector.py).
Output per probe: bbox.center and results[0].pose = position, score = mean
detection score, covariance = position variance (rms ray distance for
triangulated probes, `single_view_sigma` for ground-plane ones).

`fixed_frame` defaults to base_footprint, which is only right while the rover
holds still within `max_age`; use odom once odom -> base_footprint is published
(rover_localization's EKF).
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from rover_probe_detection import geometry
from rover_probe_detection.ros_conversions import transform_to_rt


class ProbeTriangulatorNode(Node):

    def __init__(self):
        super().__init__('probe_triangulator')
        p = self.declare_parameter
        self.cameras = p('cameras', ['arducam_mast', 'arducam_fr_bottom', 'arm_camera']).value
        det_topic = p('detections_topic', 'cameras/{camera}/probe_detections').value
        info_topic = p('camera_info_topic', 'cameras/{camera}/camera_info').value
        self.fixed_frame = p('fixed_frame', 'base_footprint').value
        self.ground_frame = p('ground_frame', 'base_footprint').value
        self.class_id = p('class_id', 'probe').value
        self.min_score = p('min_score', 0.3).value
        self.max_age = p('max_age', 0.5).value
        self.max_ray_gap = p('max_ray_gap', 0.10).value
        self.min_angle = math.radians(p('min_ray_angle_deg', 2.0).value)
        self.max_range = p('max_range', 8.0).value
        self.single_view = p('single_view', 'ground').value
        self.point_height = p('point_height', 0.0).value
        self.single_view_sigma = p('single_view_sigma', 0.3).value
        self.min_sigma = p('min_sigma', 0.02).value
        rate = p('rate', 5.0).value
        if self.single_view not in ('ground', 'none'):
            raise ValueError("single_view must be 'ground' or 'none'")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.infos = {}
        self.detections = {}
        self._warned = set()
        for cam in self.cameras:
            self.create_subscription(
                CameraInfo, info_topic.format(camera=cam),
                lambda msg, cam=cam: self.infos.__setitem__(cam, msg), qos_profile_sensor_data)
            self.create_subscription(
                Detection2DArray, det_topic.format(camera=cam),
                lambda msg, cam=cam: self.detections.__setitem__(cam, msg), 10)
        self.probes_pub = self.create_publisher(Detection3DArray, 'probes', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'probes/markers', 10)
        self.create_timer(1.0 / rate, self._tick)

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    def _lookup(self, frame, stamp):
        """(R, t) of `frame` in fixed_frame at `stamp`, or None if TF has not got it yet."""
        try:
            return transform_to_rt(self.tf_buffer.lookup_transform(
                self.fixed_frame, frame, Time.from_msg(stamp)))
        except TransformException:
            return None

    def _rays(self):
        """(rays, ground plane (point, normal) per ray, newest stamp)."""
        now = self.get_clock().now()
        rays, planes, newest = [], [], None
        for cam in self.cameras:
            msg = self.detections.get(cam)
            if msg is None or not msg.detections:
                continue
            if (now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9 > self.max_age:
                continue
            info = self.infos.get(cam)
            if info is None or info.k[0] == 0.0:
                self._warn_once(('calib', cam), f'{cam}: no calibrated camera_info, ignoring '
                                                'its detections')
                continue
            if not geometry.supported_model(info.distortion_model, info.d):
                self._warn_once(('model', cam), f'{cam}: distortion model '
                                f'{info.distortion_model!r} not supported, ignored')
            cam_tf = self._lookup(msg.header.frame_id, msg.header.stamp)
            ground_tf = self._lookup(self.ground_frame, msg.header.stamp)
            if cam_tf is None or ground_tf is None:
                continue                    # TF not there yet: retried next tick
            rot, origin = cam_tf
            g_rot, g_origin = ground_tf
            normal = g_rot[:, 2]
            plane = (g_origin + self.point_height * normal, normal)
            for det in msg.detections:
                if not det.results:
                    continue
                hyp = det.results[0].hypothesis
                if hyp.class_id != self.class_id or hyp.score < self.min_score:
                    continue
                d = geometry.pixel_to_ray(det.bbox.center.position.x, det.bbox.center.position.y,
                                          info.k, info.d, info.distortion_model)
                rays.append(geometry.Ray(cam, origin, rot @ d, hyp.score))
                planes.append(plane)
            if newest is None or Time.from_msg(msg.header.stamp) > Time.from_msg(newest):
                newest = msg.header.stamp
        return rays, planes, newest

    def _tick(self):
        rays, planes, stamp = self._rays()
        probes = []         # (point, sigma, score, cameras, triangulated)
        groups, singles = geometry.associate(rays, self.max_ray_gap, self.min_angle,
                                             self.max_range)
        for group in groups:
            members = [rays[i] for i in group]
            point, rms = geometry.triangulate(members)
            if point is None:
                continue
            probes.append((point, max(rms, self.min_sigma),
                           float(np.mean([r.score for r in members])),
                           [r.camera for r in members], True))
        if self.single_view == 'ground':
            for i in singles:
                point = geometry.intersect_plane(rays[i], *planes[i], max_range=self.max_range)
                if point is not None:
                    probes.append((point, self.single_view_sigma, rays[i].score,
                                   [rays[i].camera], False))

        out = Detection3DArray()
        out.header.frame_id = self.fixed_frame
        out.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        for point, sigma, score, cams, _ in probes:
            det = Detection3D()
            det.header = out.header
            det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = (
                float(v) for v in point)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.class_id
            hyp.hypothesis.score = score
            hyp.pose.pose.position = det.bbox.center.position
            for k in (0, 7, 14):
                hyp.pose.covariance[k] = sigma * sigma
            det.results.append(hyp)
            out.detections.append(det)
        self.probes_pub.publish(out)
        self.markers_pub.publish(self._markers(out.header, probes))

    def _markers(self, header, probes):
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (point, sigma, _, cams, triangulated) in enumerate(probes):
            m = Marker()
            m.header = header
            m.ns = 'probes'
            m.id = i
            m.type = Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in point)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = max(0.05, 2.0 * sigma)
            # green: triangulated from 2+ cameras; yellow: one camera + ground plane
            m.color.r, m.color.g, m.color.b, m.color.a = (
                (0.1, 0.9, 0.1, 0.6) if triangulated else (0.9, 0.8, 0.1, 0.6))
            arr.markers.append(m)
            t = Marker()
            t.header = header
            t.ns = 'probe_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y = float(point[0]), float(point[1])
            t.pose.position.z = float(point[2]) + 0.15
            t.pose.orientation.w = 1.0
            t.scale.z = 0.06
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = '+'.join(cams)
            arr.markers.append(t)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = ProbeTriangulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
