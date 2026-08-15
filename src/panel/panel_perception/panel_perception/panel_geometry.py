"""Pure math: fuse per-marker ArUco detections into a single panel pose.

No ROS imports here on purpose — this module is unit-testable without a
running ROS graph (see ``test/test_panel_geometry.py``).

The panel's three markers each share the panel's own orientation (their
mount joints in ``panel_macro.xacro`` all use ``rpy="0 0 0"``), so a
marker's pose alone already fully determines a candidate panel pose —
unlike a position-only rigid-transform fit (e.g. Kabsch/Umeyama), this
needs no minimum marker count to be well-determined, and works the same
way whether 1, 2, or 3 markers are currently visible.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

# Marker mount positions in the panel's own local frame (panel_base_link),
# copied from src/panel/panel_description/urdf/panel_macro.xacro's
# panel_marker joints (`origin xyz="${x} -0.003 ${z}"`, `rpy="0 0 0"`).
# Duplicated here rather than parsed from the xacro at runtime — same
# tradeoff already accepted for DEFAULT_GRIPPER_STROKE in
# keyboard_servo_node.py. Keep in sync by hand if the panel layout changes.
KNOWN_MARKER_LOCAL_POSITIONS: dict[int, tuple[float, float, float]] = {
    20: (-0.135, -0.003, 0.415),  # top_left
    21: (0.135, -0.003, 0.415),   # top_right
    22: (-0.135, -0.003, 0.035),  # bottom_left
}


@dataclass(frozen=True)
class MarkerDetection:
    """One marker's pose (position + quaternion xyzw), in the camera frame."""

    marker_id: int
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class FusedPanelPose:
    """Fused panel pose, in the same frame the input detections were in."""

    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    marker_ids_used: tuple[int, ...]
    max_position_disagreement: float
    max_orientation_disagreement: float


def _candidate_panel_pose(detection: MarkerDetection) -> tuple[np.ndarray, Rotation]:
    """One marker's detection alone -> a full candidate panel pose.

    The marker and panel share the same orientation, so
    R_camera_panel == R_camera_marker, and the panel's origin is just the
    marker's origin shifted by the marker's own local offset from the
    panel origin, rotated into the camera frame:
        t_camera_panel = t_camera_marker - R_camera_marker @ local_offset
    """
    local_offset = np.array(KNOWN_MARKER_LOCAL_POSITIONS[detection.marker_id])
    r_camera_marker = Rotation.from_quat(detection.orientation_xyzw)
    t_camera_marker = np.array(detection.position)
    t_camera_panel = t_camera_marker - r_camera_marker.apply(local_offset)
    return t_camera_panel, r_camera_marker  # orientation is shared with the marker


def fuse_panel_pose(detections: list[MarkerDetection]) -> FusedPanelPose | None:
    """Fuse 1-3 marker detections into a single panel pose candidate.

    Returns ``None`` if no detection has a recognized (known) marker ID.
    Unknown marker IDs (not in ``KNOWN_MARKER_LOCAL_POSITIONS``) are
    silently ignored, not an error — the camera may see other, unrelated
    ArUco tags in the same frame.

    Disagreement fields let the caller decide whether the fused result is
    trustworthy enough to act on (e.g. drive the arm) — this function
    itself never refuses to return a fused pose, it just reports how much
    the individual candidates disagreed.
    """
    known = [d for d in detections if d.marker_id in KNOWN_MARKER_LOCAL_POSITIONS]
    if not known:
        return None

    candidates = [_candidate_panel_pose(d) for d in known]
    positions = np.stack([t for t, _ in candidates])
    rotations = Rotation.concatenate([r for _, r in candidates])

    fused_position = positions.mean(axis=0)
    # Markley's method (mean of quaternions via the dominant eigenvector of
    # their outer-product sum) — scipy's Rotation.mean() implements this.
    fused_rotation = rotations.mean()

    max_pos_disagreement = 0.0
    max_orient_disagreement = 0.0
    if len(known) > 1:
        max_pos_disagreement = float(
            np.max(np.linalg.norm(positions - fused_position, axis=1))
        )
        relative = fused_rotation.inv() * rotations
        max_orient_disagreement = float(np.max(relative.magnitude()))

    return FusedPanelPose(
        position=tuple(fused_position.tolist()),
        orientation_xyzw=tuple(fused_rotation.as_quat().tolist()),
        marker_ids_used=tuple(d.marker_id for d in known),
        max_position_disagreement=max_pos_disagreement,
        max_orientation_disagreement=max_orient_disagreement,
    )
