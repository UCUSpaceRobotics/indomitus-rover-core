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
  gated behind ``min_markers_to_publish`` (default 3 — see that
  parameter's own comment for why 1-2 isn't enough in practice) — the
  actually actionable signal, consumed by panel_align_node before
  planning any motion.

The GridBoard feature built into aruco_opencv (``board_descriptions.yaml``)
was considered and rejected: it only supports a regular MxN grid with a
single uniform spacing value, but this panel's three markers are an
asymmetric layout (3 of a rectangle's 4 corners, 0.27m horizontal vs
0.38m vertical spacing) that a GridBoard cannot represent. See
``panel_geometry.py`` for the actual (hand-rolled) fusion method.
"""

import math
import os
import time

import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from panel_perception.panel_geometry import (
    BOTTOM_LEFT_LOCAL_POSITION,
    TOP_LEFT_LOCAL_POSITION,
    TOP_RIGHT_LOCAL_POSITION,
    MarkerDetection,
    fuse_panel_pose,
)

DEFAULT_ARUCO_DETECTIONS_TOPIC = '/aruco_detections'
# Absolute on purpose, matching servo_controller.py's and
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
# Tightened on explicit request to 5 deg. NOTE this is still slightly
# below the real noise floor measured live with all 3 markers visible
# (~0.09-0.14rad / 5.6-8deg for genuinely good frames — 7deg was
# confirmed working, see git history). Expect panel_pose to reject a
# good chunk of frames and update less often than at 7deg — if 'p'
# starts reporting "no panel pose received yet" too often, raise this
# back toward 7-8deg.
DEFAULT_MAX_ORIENTATION_DISAGREEMENT = math.radians(5)
# Final fallback marker-ID assignment (15 unused) — matches
# panel_macro.xacro's own matching defaults, for consistency when
# neither an explicit --ros-args override nor the sim auto-layout file
# below is present (e.g. a real panel started without configuring
# marker_id_top_left/top_right/bottom_left yet).
FALLBACK_MARKER_ID_TOP_LEFT = 11
FALLBACK_MARKER_ID_TOP_RIGHT = 13
FALLBACK_MARKER_ID_BOTTOM_LEFT = 14
# Written fresh by arm_sim/launch/arm_gazebo.launch.py on every sim
# launch (see its own matching logic) with that run's randomized
# marker_id_top_left/top_right/bottom_left — lets `ros2 run
# panel_perception panel_pose_fuser_node` just work in sim with no
# --ros-args needed, despite the two nodes being launched separately
# (this repo's usual sim workflow) with no other way to share that
# choice. Irrelevant on real hardware (nothing ever writes this file
# there) — an explicit --ros-args override always wins over it either
# way, see _resolve_marker_id.
#
# Scoped by ROS_DOMAIN_ID, not one bare global path — two sim instances
# on the same host with different domain IDs would otherwise clobber
# each other's layout file. _load_sim_marker_layout also ignores this
# file if it's older than SIM_MARKER_LAYOUT_MAX_AGE_SEC, as a backstop
# against a crashed sim run's leftover file (arm_gazebo.launch.py
# deletes it on a clean exit, but can't on a crash) being picked up by a
# later, unrelated run.
PANEL_MARKER_LAYOUT_SIM_FILE = f'/tmp/panel_marker_layout_domain{os.environ.get("ROS_DOMAIN_ID", "0")}.yaml'
SIM_MARKER_LAYOUT_MAX_AGE_SEC = 600.0
# Sentinel: declare_parameter's default can't be "unset", so this
# stands in for "no explicit --ros-args override was given" — see
# _resolve_marker_id.
_UNSET_MARKER_ID = -1


def _resolve_marker_id(explicit_value: int, sim_layout: dict, role: str, fallback: int) -> int:
    """explicit --ros-args value, else the sim auto-layout file, else fallback."""
    if explicit_value != _UNSET_MARKER_ID:
        return explicit_value
    if role in sim_layout:
        return sim_layout[role]
    return fallback


def _load_sim_marker_layout(logger) -> dict:
    if not os.path.isfile(PANEL_MARKER_LAYOUT_SIM_FILE):
        return {}
    age_sec = time.time() - os.path.getmtime(PANEL_MARKER_LAYOUT_SIM_FILE)
    if age_sec > SIM_MARKER_LAYOUT_MAX_AGE_SEC:
        logger.warn(
            f'{PANEL_MARKER_LAYOUT_SIM_FILE} is {age_sec:.0f}s old (max '
            f'{SIM_MARKER_LAYOUT_MAX_AGE_SEC:.0f}s) — likely a leftover from a crashed '
            'sim run, ignoring it. Restart arm_gazebo.launch.py to write a fresh one.'
        )
        return {}
    try:
        with open(PANEL_MARKER_LAYOUT_SIM_FILE) as f:
            layout = yaml.safe_load(f) or {}
        logger.info(f'Loaded sim marker layout from {PANEL_MARKER_LAYOUT_SIM_FILE}: {layout}')
        return layout
    except (OSError, yaml.YAMLError) as exc:
        logger.warn(f'Could not read {PANEL_MARKER_LAYOUT_SIM_FILE}: {exc}')
        return {}


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
        # target. Raised from 2 to 3 on explicit request: panel_geometry's
        # fuse_panel_pose only uses its accurate, layout-based (Kabsch)
        # orientation fit — immune to individual markers' own noisy
        # solvePnP orientation, see that function's docstring — when ALL
        # 3 known markers are visible; with only 1-2 it silently falls
        # back to averaging each marker's own noisier orientation
        # estimate. Requiring 3 here means panel_align_node only ever
        # acts on the accurate fit. panel_align_node has no way to know
        # how many markers a given PoseStamped was built from (plain
        # geometry_msgs type, no count field) — so this gate is enforced
        # here, by simply not publishing panel_pose at all below this
        # count, rather than inventing a custom message type to carry a
        # count downstream.
        self.declare_parameter('min_markers_to_publish', 3)
        # Which ArUco ID is mounted at which of the panel's 3 fixed
        # physical mounts — per competition rules any 3 of IDs
        # {11,13,14,15} may be used, in any of the 3 positions, so this
        # is a runtime choice, not a constant. Sentinel default (-1)
        # means "not explicitly overridden" — see _resolve_marker_id for
        # what wins: an explicit --ros-args value, then the sim
        # auto-layout file (PANEL_MARKER_LAYOUT_SIM_FILE), then the
        # hardcoded fallback.
        self.declare_parameter('marker_id_top_left', _UNSET_MARKER_ID)
        self.declare_parameter('marker_id_top_right', _UNSET_MARKER_ID)
        self.declare_parameter('marker_id_bottom_left', _UNSET_MARKER_ID)

        detections_topic = self.get_parameter('aruco_detections_topic').value
        pose_topic = self.get_parameter('panel_pose_topic').value
        visible_topic = self.get_parameter('panel_visible_topic').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._max_pos_disagreement = self.get_parameter('max_position_disagreement').value
        self._max_orient_disagreement = self.get_parameter('max_orientation_disagreement').value
        self._min_markers_to_publish = self.get_parameter('min_markers_to_publish').value

        sim_layout = _load_sim_marker_layout(self.get_logger())
        marker_id_top_left = _resolve_marker_id(
            self.get_parameter('marker_id_top_left').value, sim_layout,
            'top_left', FALLBACK_MARKER_ID_TOP_LEFT)
        marker_id_top_right = _resolve_marker_id(
            self.get_parameter('marker_id_top_right').value, sim_layout,
            'top_right', FALLBACK_MARKER_ID_TOP_RIGHT)
        marker_id_bottom_left = _resolve_marker_id(
            self.get_parameter('marker_id_bottom_left').value, sim_layout,
            'bottom_left', FALLBACK_MARKER_ID_BOTTOM_LEFT)
        self._marker_local_positions = {
            marker_id_top_left: TOP_LEFT_LOCAL_POSITION,
            marker_id_top_right: TOP_RIGHT_LOCAL_POSITION,
            marker_id_bottom_left: BOTTOM_LEFT_LOCAL_POSITION,
        }
        if len(self._marker_local_positions) != 3:
            raise ValueError(
                'marker_id_top_left/top_right/bottom_left must resolve to 3 DISTINCT IDs, got '
                f'{marker_id_top_left}, {marker_id_top_right}, {marker_id_bottom_left}'
            )

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._visible_pub = self.create_publisher(Bool, visible_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(ArucoDetection, detections_topic, self._on_detection, 10)

        self.get_logger().info(
            # Full role mapping, not just the ID set — this is the only
            # record of which layout was actually in play for a given
            # run (sim's choice is randomized per-launch), findable
            # later in ~/.ros/log/ if something needs tracing back.
            f'Watching for panel markers: top_left={marker_id_top_left} '
            f'top_right={marker_id_top_right} bottom_left={marker_id_bottom_left} '
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

        fused = fuse_panel_pose(detections, self._marker_local_positions)
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
