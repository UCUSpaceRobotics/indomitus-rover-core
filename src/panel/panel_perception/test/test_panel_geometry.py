"""Pure-math tests for panel_geometry.py — no ROS graph needed."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from panel_perception.panel_geometry import (
    KNOWN_MARKER_LOCAL_POSITIONS,
    MarkerDetection,
    fuse_panel_pose,
)


def _detections_for_panel_pose(position, orientation_xyzw):
    """Build detections for all 3 known markers as if the panel really
    were at the given pose (in some arbitrary "camera" frame)."""
    r = Rotation.from_quat(orientation_xyzw)
    position = np.array(position)
    dets = []
    for marker_id, local_pos in KNOWN_MARKER_LOCAL_POSITIONS.items():
        cam_pos = position + r.apply(np.array(local_pos))
        dets.append(MarkerDetection(marker_id, tuple(cam_pos.tolist()), orientation_xyzw))
    return dets


def test_returns_none_with_no_known_markers():
    assert fuse_panel_pose([]) is None
    assert fuse_panel_pose([MarkerDetection(999, (0, 0, 0), (0, 0, 0, 1))]) is None


def test_identity_pose_recovered_exactly():
    dets = _detections_for_panel_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    fused = fuse_panel_pose(dets)
    assert fused is not None
    np.testing.assert_allclose(fused.position, (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(fused.orientation_xyzw, (0.0, 0.0, 0.0, 1.0), atol=1e-9)
    assert fused.max_position_disagreement == pytest.approx(0.0, abs=1e-9)
    assert fused.max_orientation_disagreement == pytest.approx(0.0, abs=1e-9)
    assert set(fused.marker_ids_used) == {20, 21, 22}


def test_offset_and_rotated_pose_recovered():
    position = (1.0, 2.0, 0.5)
    orientation = tuple(Rotation.from_euler('z', 30, degrees=True).as_quat().tolist())
    dets = _detections_for_panel_pose(position, orientation)
    fused = fuse_panel_pose(dets)
    np.testing.assert_allclose(fused.position, position, atol=1e-9)
    # Quaternion sign ambiguity (q and -q represent the same rotation).
    same = np.allclose(fused.orientation_xyzw, orientation, atol=1e-9)
    flipped = np.allclose(fused.orientation_xyzw, -np.array(orientation), atol=1e-9)
    assert same or flipped


def test_single_marker_is_sufficient():
    dets = _detections_for_panel_pose((3.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    fused = fuse_panel_pose([dets[0]])
    assert fused is not None
    assert len(fused.marker_ids_used) == 1
    np.testing.assert_allclose(fused.position, (3.0, 0.0, 0.0), atol=1e-9)
    # No second candidate to disagree with.
    assert fused.max_position_disagreement == 0.0
    assert fused.max_orientation_disagreement == 0.0


def test_unknown_marker_ids_are_ignored():
    known = _detections_for_panel_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    unknown = MarkerDetection(999, (5.0, 5.0, 5.0), (0, 0, 0, 1))
    fused = fuse_panel_pose(known + [unknown])
    assert set(fused.marker_ids_used) == {20, 21, 22}


def test_disagreeing_candidates_report_nonzero_disagreement():
    dets = _detections_for_panel_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    # Perturb one marker's detected position as if it were a bad reading.
    bad = dets[0]
    dets[0] = MarkerDetection(
        bad.marker_id,
        (bad.position[0] + 0.05, bad.position[1], bad.position[2]),
        bad.orientation_xyzw,
    )
    fused = fuse_panel_pose(dets)
    assert fused.max_position_disagreement > 0.01
