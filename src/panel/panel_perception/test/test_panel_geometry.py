"""Pure-math tests for panel_geometry.py — no ROS graph needed."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from panel_perception.panel_geometry import (
    BOTTOM_LEFT_LOCAL_POSITION,
    KNOWN_MARKER_LOCAL_POSITIONS,
    TOP_LEFT_LOCAL_POSITION,
    TOP_RIGHT_LOCAL_POSITION,
    MarkerDetection,
    fuse_panel_pose,
)


def _detections_for_panel_pose(position, orientation_xyzw, marker_local_positions=KNOWN_MARKER_LOCAL_POSITIONS):
    """Build detections for all 3 configured markers as if the panel
    really were at the given pose (in some arbitrary "camera" frame).

    Simulates realistic ArUco output: the marker's reported orientation is
    in ArUco's own convention, not the panel's, so it must be the true
    panel orientation composed with the inverse of
    MARKER_TO_PANEL_ORIENTATION_CORRECTION (see panel_geometry.py's
    module docstring) — otherwise these tests would just re-encode the
    exact bug that correction fixes.
    """
    r_true_panel = Rotation.from_quat(orientation_xyzw)
    r_raw_marker = r_true_panel * Rotation.from_euler('x', 90, degrees=True)
    raw_quat = tuple(r_raw_marker.as_quat().tolist())
    position = np.array(position)
    dets = []
    for marker_id, local_pos in marker_local_positions.items():
        cam_pos = position + r_true_panel.apply(np.array(local_pos))
        dets.append(MarkerDetection(marker_id, tuple(cam_pos.tolist()), raw_quat))
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
    assert set(fused.marker_ids_used) == set(KNOWN_MARKER_LOCAL_POSITIONS)


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
    assert set(fused.marker_ids_used) == set(KNOWN_MARKER_LOCAL_POSITIONS)


def test_layout_fit_ignores_per_marker_orientation_noise():
    """With all 3 markers visible, injected per-marker orientation noise
    should barely move the fused orientation — it comes from the marker
    LAYOUT (Kabsch fit), not from averaging each marker's own (noisy)
    solvePnP orientation. See _fit_orientation_from_marker_layout's
    docstring for why this matters for panel_align_node's perpendicularity.
    """
    position = (1.0, 2.0, 0.5)
    true_orientation = Rotation.from_euler('z', 30, degrees=True)
    dets = _detections_for_panel_pose(position, tuple(true_orientation.as_quat().tolist()))

    # Perturb each marker's own reported orientation (not its position) by
    # a plausible few-degree amount, as real solvePnP noise would.
    rng = np.random.default_rng(0)
    noisy = []
    for d in dets:
        noise = Rotation.from_euler('xyz', rng.uniform(-6, 6, size=3), degrees=True)
        r_noisy = Rotation.from_quat(d.orientation_xyzw) * noise
        noisy.append(MarkerDetection(d.marker_id, d.position, tuple(r_noisy.as_quat().tolist())))

    fused = fuse_panel_pose(noisy)
    error_deg = (Rotation.from_quat(fused.orientation_xyzw).inv() * true_orientation).magnitude()
    error_deg = np.degrees(error_deg)
    assert error_deg < 1.0, f'layout fit should reject per-marker orientation noise, got {error_deg:.2f}deg off'


def test_layout_fit_works_with_arbitrary_marker_ids_and_role_assignment():
    """Per competition rules, any 3 of IDs {11,13,14,15} may be mounted at
    any of the panel's 3 physical positions — the fit must not depend on
    the specific IDs 20/21/22 or on which particular ID ends up at which
    role, only on the actual (x,z) layout it's told about.
    """
    custom_mapping = {
        14: BOTTOM_LEFT_LOCAL_POSITION,  # 14 at the bottom-left mount
        11: TOP_RIGHT_LOCAL_POSITION,    # 11 at the top-right mount
        15: TOP_LEFT_LOCAL_POSITION,     # 15 at the top-left mount
    }
    position = (0.3, -0.2, 1.1)
    true_orientation = Rotation.from_euler('xyz', (5, 15, -40), degrees=True)
    dets = _detections_for_panel_pose(
        position, tuple(true_orientation.as_quat().tolist()), custom_mapping)
    fused = fuse_panel_pose(dets, custom_mapping)
    assert fused is not None
    assert set(fused.marker_ids_used) == {11, 14, 15}

    top_edge = np.array(dets[[d.marker_id for d in dets].index(11)].position) - \
        np.array(dets[[d.marker_id for d in dets].index(15)].position)
    x_axis = Rotation.from_quat(fused.orientation_xyzw).apply((1.0, 0.0, 0.0))
    np.testing.assert_allclose(x_axis, top_edge / np.linalg.norm(top_edge), atol=1e-9)


def test_layout_fit_x_axis_exactly_matches_top_edge():
    """The whole point of building the frame from the marker edges
    directly (rather than a generic least-squares fit): the panel's real
    top edge (top_left->top_right, in camera-frame) must land EXACTLY
    along the fused orientation's local X axis, so panel_align_node's
    target roll makes that edge come out perfectly horizontal in the
    final camera view — not just "close" in some averaged sense.
    """
    position = (0.3, -0.2, 1.1)
    true_orientation = Rotation.from_euler('xyz', (5, 15, -40), degrees=True)
    dets = _detections_for_panel_pose(position, tuple(true_orientation.as_quat().tolist()))
    fused = fuse_panel_pose(dets)

    ids_by_local_position = {v: k for k, v in KNOWN_MARKER_LOCAL_POSITIONS.items()}
    top_left_id = ids_by_local_position[TOP_LEFT_LOCAL_POSITION]
    top_right_id = ids_by_local_position[TOP_RIGHT_LOCAL_POSITION]
    by_id = {d.marker_id: np.array(d.position) for d in dets}
    top_edge = by_id[top_right_id] - by_id[top_left_id]
    top_edge_hat = top_edge / np.linalg.norm(top_edge)

    x_axis = Rotation.from_quat(fused.orientation_xyzw).apply((1.0, 0.0, 0.0))
    np.testing.assert_allclose(x_axis, top_edge_hat, atol=1e-9)


def test_layout_fit_not_used_with_fewer_than_three_markers():
    """2 markers can never geometrically pin down orientation alone (any
    2 points are trivially collinear) — falls back to averaging, so
    injected orientation noise DOES move the result (sanity check that
    the fallback path is actually exercised, not accidentally bypassed).
    """
    position = (0.0, 0.0, 0.0)
    true_orientation = Rotation.from_euler('z', 0, degrees=True)
    dets = _detections_for_panel_pose(position, tuple(true_orientation.as_quat().tolist()))[:2]
    noise = Rotation.from_euler('xyz', (5, 5, 5), degrees=True)
    d0 = dets[0]
    r_noisy = Rotation.from_quat(d0.orientation_xyzw) * noise
    dets[0] = MarkerDetection(d0.marker_id, d0.position, tuple(r_noisy.as_quat().tolist()))

    fused = fuse_panel_pose(dets)
    error_deg = np.degrees((Rotation.from_quat(fused.orientation_xyzw).inv() * true_orientation).magnitude())
    assert error_deg > 0.5


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
