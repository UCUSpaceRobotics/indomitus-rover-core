"""Fuses aruco_opencv's per-marker panel-tag detections into one panel pose.

Subscribes to an ``aruco_opencv_msgs/msg/ArucoDetection`` topic (the
``aruco_tracker`` node from ``rover_aruco``/``aruco_opencv``, pointed at
whichever camera is relevant — the arm's wrist camera for this use case),
and publishes two separate signals at two separate confidence bars:

- ``panel_visible`` (``std_msgs/msg/Bool``, published True) the moment
  ANY single known marker is seen — cheap, "is a panel roughly here"
  notification for e.g. prompting a teleop operator, not safe to act on
  by itself (a single marker's pose, especially depth/roll-about-its-own
  -normal, is noticeably less reliable than a multi-marker fit).
- ``panel_pose`` (``geometry_msgs/msg/PoseStamped``), the fused pose,
  gated behind ``min_markers_to_publish`` (default 2 — see that
  parameter's own comment for why 1 isn't enough in practice) — the
  actually actionable signal, consumed by panel_align_node before
  planning any motion.

The GridBoard feature built into aruco_opencv (``board_descriptions.yaml``)
was considered and rejected: it only supports a regular MxN grid with a
single uniform spacing value, but this panel's three markers are an
asymmetric layout (3 of a rectangle's 4 corners, 0.27m horizontal vs
0.38m vertical spacing) that a GridBoard cannot represent. See
``panel_geometry.py`` for the actual (hand-rolled) fusion method.
"""

import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from panel_perception.panel_geometry import (
    KNOWN_MARKER_LOCAL_POSITIONS,
    MarkerDetection,
    fuse_panel_pose,
)

DEFAULT_ARUCO_DETECTIONS_TOPIC = '/aruco_detections'
# Absolute on purpose, matching keyboard_servo_node.py's and
# panel_align_node.py's own DEFAULT_PANEL_POSE_TOPIC — this node isn't
# launched under a namespace by anything in the repo today, so a relative
# default happened to resolve to the same absolute topic either way, but
# would silently stop matching the moment this node is later added to a
# launch file under any namespace (e.g. the panel_state_publisher's own
# namespace="panel" pattern elsewhere in this repo).
DEFAULT_PANEL_POSE_TOPIC = '/panel_pose'
DEFAULT_PANEL_VISIBLE_TOPIC = '/panel_visible'
DEFAULT_PANEL_TF_FRAME = 'panel'
# Disagreement thresholds: with only 1 marker visible these are moot (no
# comparison possible); with 2-3, exceeding either means the candidate
# poses don't agree with each other enough to trust — most likely a bad
# individual detection (motion blur, partial occlusion, a genuinely wrong
# marker-ID mapping) — so the fused pose is logged and dropped rather
# than published for panel_align_node to act on. Loosened from an
# initial 0.03/0.15 — confirmed live those were tighter than realistic
# monocular ArUco noise: two markers at a good, roughly frontal angle
# (top_left/top_right, 0.27m apart) disagreed by ~0.05m/~0.11rad on
# their own, before a third, more obliquely-viewed marker was even
# considered. This still catches genuine outliers (a badly-angled or
# occluded marker showed 0.30-0.40m disagreement against the other two
# in the same test), just no longer rejects normal-quality detections.
DEFAULT_MAX_POSITION_DISAGREEMENT = 0.08  # meters
DEFAULT_MAX_ORIENTATION_DISAGREEMENT = 0.3  # radians (~17 deg)


class PanelPoseFuser(Node):
    """Turns raw per-marker ArUco detections into one panel pose estimate."""

    def __init__(self):
        super().__init__('panel_pose_fuser')

        self.declare_parameter('aruco_detections_topic', DEFAULT_ARUCO_DETECTIONS_TOPIC)
        self.declare_parameter('panel_pose_topic', DEFAULT_PANEL_POSE_TOPIC)
        self.declare_parameter('panel_visible_topic', DEFAULT_PANEL_VISIBLE_TOPIC)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_position_disagreement', DEFAULT_MAX_POSITION_DISAGREEMENT)
        self.declare_parameter('max_orientation_disagreement', DEFAULT_MAX_ORIENTATION_DISAGREEMENT)
        # A pose fused from a single marker is geometrically valid but
        # MUCH noisier (esp. depth/roll-about-marker-normal) than a
        # multi-marker fit — confirmed live: lowering this to 1 for
        # testing produced a "panel" 1.08m from the camera (arm's real
        # reach is ~0.9m), i.e. a single marker's monocular pose estimate
        # was off badly enough to compute a physically unreachable
        # target. Back to requiring 2+; panel_align_node has no way to
        # know how many markers a given PoseStamped was built from (plain
        # geometry_msgs type, no count field) — so this gate is enforced
        # here, by simply not publishing panel_pose at all below this
        # count, rather than inventing a custom message type to carry a
        # count downstream.
        self.declare_parameter('min_markers_to_publish', 2)

        detections_topic = self.get_parameter('aruco_detections_topic').value
        pose_topic = self.get_parameter('panel_pose_topic').value
        visible_topic = self.get_parameter('panel_visible_topic').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._max_pos_disagreement = self.get_parameter('max_position_disagreement').value
        self._max_orient_disagreement = self.get_parameter('max_orientation_disagreement').value
        self._min_markers_to_publish = self.get_parameter('min_markers_to_publish').value

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._visible_pub = self.create_publisher(Bool, visible_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(ArucoDetection, detections_topic, self._on_detection, 10)

        self.get_logger().info(
            f'Watching for panel markers {sorted(KNOWN_MARKER_LOCAL_POSITIONS)} '
            f'on "{detections_topic}" — "{visible_topic}" fires on any single marker, '
            f'"{pose_topic}" needs {self._min_markers_to_publish}+.'
        )

    def _on_detection(self, msg: ArucoDetection) -> None:
        detections = [
            MarkerDetection(
                marker_id=m.marker_id,
                position=(m.pose.position.x, m.pose.position.y, m.pose.position.z),
                orientation_xyzw=(
                    m.pose.orientation.x, m.pose.orientation.y,
                    m.pose.orientation.z, m.pose.orientation.w,
                ),
            )
            for m in msg.markers
        ]

        fused = fuse_panel_pose(detections)
        if fused is None:
            return

        self._visible_pub.publish(Bool(data=True))

        if len(fused.marker_ids_used) < self._min_markers_to_publish:
            self.get_logger().debug(
                f'Only {len(fused.marker_ids_used)} panel marker(s) visible '
                f'(need {self._min_markers_to_publish}) — not publishing.'
            )
            return

        if (fused.max_position_disagreement > self._max_pos_disagreement
                or fused.max_orientation_disagreement > self._max_orient_disagreement):
            self.get_logger().warn(
                'Panel marker candidates disagree too much to trust '
                f'(markers={fused.marker_ids_used}, '
                f'pos_disagreement={fused.max_position_disagreement:.4f}m, '
                f'orient_disagreement={fused.max_orientation_disagreement:.4f}rad) '
                '— not publishing this frame.'
            )
            return

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = fused.position
        (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
         pose_msg.pose.orientation.z, pose_msg.pose.orientation.w) = fused.orientation_xyzw
        self._pose_pub.publish(pose_msg)

        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.child_frame_id = DEFAULT_PANEL_TF_FRAME
            tf_msg.transform.translation.x, tf_msg.transform.translation.y, \
                tf_msg.transform.translation.z = fused.position
            (tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
             tf_msg.transform.rotation.z, tf_msg.transform.rotation.w) = fused.orientation_xyzw
            self._tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = PanelPoseFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
