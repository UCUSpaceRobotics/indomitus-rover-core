"""Pure math: fuse per-marker ArUco detections into a single panel pose.

No ROS imports here on purpose — this module is unit-testable without a
running ROS graph (see ``test/test_panel_geometry.py``).

The panel's three markers each share the panel's own orientation (their
mount joints in ``panel_macro.xacro`` all use ``rpy="0 0 0"``), so a
marker's pose alone already fully determines a candidate panel pose —
unlike a position-only rigid-transform fit (e.g. Kabsch/Umeyama), this
needs no minimum marker count to be well-determined, and works the same
way whether 1, 2, or 3 markers are currently visible.

That "shares the panel's own orientation" statement is true of the URDF
link frame (rpy="0 0 0" relative to ``panel_base_link``), but ArUco's own
reported marker orientation uses a *different*, fixed convention
(marker's own local X=right, Y=up-in-plane, Z=out-of-plane toward the
camera — the standard OpenCV/ArUco tag convention), which is not the same
basis as the panel's own X=width, Y=depth/normal, Z=up. Confirmed live
(sim, all 3 markers): fusing without correcting for this produced a
~0.38m position disagreement between markers, tracking exactly with the
markers' differing local Z offsets; composing each detection's
orientation with ``Rx(-90 deg)`` (``MARKER_TO_PANEL_ORIENTATION_CORRECTION``
below) before fusing dropped that to ~0.02m (sensor noise). See
``_candidate_panel_pose``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

# The panel's 3 physical marker mounts, in the panel's own local frame
# (panel_base_link) — copied from src/panel/panel_description/urdf/
# panel_macro.xacro's panel_marker joints (`origin xyz="${x} -0.003
# ${z}"`, `rpy="0 0 0"`). These positions are fixed (the panel is a
# manufactured object with 3 fixed mounting points); WHICH ArUco ID sits
# at which mount is not — per competition rules any 3 of IDs
# {11,13,14,15} may be used, in any of these 3 positions, so that
# mapping is a runtime choice (see panel_pose_fuser_node.py's
# marker_id_top_left/top_right/bottom_left ROS parameters), not a
# constant here. Duplicated from the xacro rather than parsed from it at
# runtime — same tradeoff already accepted for DEFAULT_GRIPPER_STROKE in
# keyboard_servo_node.py. Keep in sync by hand if the panel's physical
# mount positions ever change (not if just the ID assignment changes).
TOP_LEFT_LOCAL_POSITION = (-0.135, -0.003, 0.415)
TOP_RIGHT_LOCAL_POSITION = (0.135, -0.003, 0.415)
BOTTOM_LEFT_LOCAL_POSITION = (-0.135, -0.003, 0.035)

# One valid arrangement of the competition's ID set (15 unused) — used
# as fuse_panel_pose's default so existing callers/tests keep working
# unchanged; matches panel_macro.xacro's/panel_pose_fuser_node.py's own
# matching defaults. Real/competition use should pass an explicit
# marker_local_positions mapping built from whatever IDs that panel
# actually uses instead of relying on this default.
KNOWN_MARKER_LOCAL_POSITIONS: dict[int, tuple[float, float, float]] = {
    11: TOP_LEFT_LOCAL_POSITION,
    13: TOP_RIGHT_LOCAL_POSITION,
    14: BOTTOM_LEFT_LOCAL_POSITION,
}

# ArUco's marker-local convention (X=right, Y=up-in-plane, Z=out-of-plane
# toward camera) expressed in the panel's own local convention (X=right,
# Y=depth/normal, Z=up): ArUco's X -> panel's X, ArUco's Y -> panel's Z,
# ArUco's Z -> panel's -Y. Composing a detection's orientation with this
# (on the right) converts "R_camera_arucoMarker" into "R_camera_panel" —
# see module docstring for how this was found and confirmed.
MARKER_TO_PANEL_ORIENTATION_CORRECTION = Rotation.from_euler('x', -90, degrees=True)


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


def _candidate_panel_pose(
    detection: MarkerDetection,
    marker_local_positions: dict[int, tuple[float, float, float]],
) -> tuple[np.ndarray, Rotation]:
    """One marker's detection alone -> a full candidate panel pose.

    The marker link and panel share the same URDF orientation, but ArUco's
    reported orientation is in ArUco's own convention, not the panel's —
    apply MARKER_TO_PANEL_ORIENTATION_CORRECTION first to get a genuine
    R_camera_panel. From there, the panel's origin is just the marker's
    origin shifted by the marker's own local offset from the panel
    origin, rotated into the camera frame:
        t_camera_panel = t_camera_marker - R_camera_panel @ local_offset
    """
    local_offset = np.array(marker_local_positions[detection.marker_id])
    r_camera_marker = Rotation.from_quat(detection.orientation_xyzw) * MARKER_TO_PANEL_ORIENTATION_CORRECTION
    t_camera_marker = np.array(detection.position)
    t_camera_panel = t_camera_marker - r_camera_marker.apply(local_offset)
    return t_camera_panel, r_camera_marker  # now genuinely R_camera_panel


def _fit_orientation_from_marker_layout(
    known: list[MarkerDetection],
    marker_local_positions: dict[int, tuple[float, float, float]],
) -> Rotation | None:
    """Orientation from the markers' physical LAYOUT alone — built
    directly from the real edges between marker positions (the panel's
    top edge = the panel's own local +X/"right"; its left edge = local
    +Z/"up") — ignoring each marker's own individually-estimated
    orientation entirely.

    Why this is more accurate: a single marker's ArUco orientation comes
    from solvePnP on just that one ~5cm tag, which is inherently noisy
    for the panel-normal ("broadside tilt") axis — confirmed live, ~6deg
    disagreement between two well-viewed markers 0.27m apart (see
    panel_pose_fuser_node.py's DEFAULT_MAX_ORIENTATION_DISAGREEMENT
    comment). The physical baseline BETWEEN markers (up to 0.38m) is a far
    longer lever arm for the same angular error, so a position-only fit
    across that baseline is far less sensitive to per-marker noise.

    Which of the 3 VISIBLE markers is "top_left"/"top_right"/
    "bottom_left" is worked out from ``marker_local_positions`` itself
    (lowest local Z -> bottom; of the other two, lower/higher local X ->
    left/right) rather than assumed from fixed IDs — per competition
    rules any 3 of IDs {11,13,14,15} may be mounted in any of the panel's
    3 physical positions, so the ID<->role mapping is a runtime
    configuration, not a constant (see panel_pose_fuser_node.py's
    marker_id_top_left/top_right/bottom_left parameters, which build
    that mapping).

    Building the frame straight from these two named edge vectors (rather
    than a generic least-squares fit across however many markers happen
    to be visible) is deliberate: it's what actually guarantees the
    panel's own real top edge comes out exactly along the resulting
    local X axis — i.e. panel_align_node's target camera roll ends up
    matching the panel's own edges exactly, not just "close" in a
    least-squares sense. Requires all 3 of the panel's configured
    markers to be visible (needs BOTH edges to fully pin down the frame
    — with only 2 markers there's a single edge, one axis, and the roll
    about it is inherently undetermined by position alone, no matter how
    that pair is chosen). Returns None otherwise, so the caller falls
    back to the (noisier, but always available) per-marker orientation
    average.
    """
    required_ids = set(marker_local_positions)
    if len(required_ids) != 3:
        return None  # the panel has exactly 3 mounts — a misconfiguration otherwise
    by_id_cam = {d.marker_id: np.array(d.position) for d in known if d.marker_id in required_ids}
    if len(by_id_cam) < 3:
        return None

    bottom_id = min(required_ids, key=lambda i: marker_local_positions[i][2])
    left_id, right_id = sorted(required_ids - {bottom_id}, key=lambda i: marker_local_positions[i][0])

    right_raw = by_id_cam[right_id] - by_id_cam[left_id]     # top-left -> top-right: local +X
    up_raw = by_id_cam[left_id] - by_id_cam[bottom_id]       # bottom-left -> top-left: local +Z
    right_norm = np.linalg.norm(right_raw)
    up_norm = np.linalg.norm(up_raw)
    if right_norm < 1e-6 or up_norm < 1e-6:
        return None  # degenerate detection (duplicate/coincident positions)

    x_axis = right_raw / right_norm
    # Orthogonalize the measured "up" edge against X (real marker
    # measurements at these baselines won't be perfectly perpendicular),
    # keeping X as the primary reference since panel_align_node's roll
    # requirement cares about the top edge landing exactly horizontal.
    z_raw = up_raw - np.dot(up_raw, x_axis) * x_axis
    z_norm = np.linalg.norm(z_raw)
    if z_norm < 1e-6:  # up_raw was parallel to right_raw — degenerate layout
        return None
    z_axis = z_raw / z_norm
    y_axis = np.cross(z_axis, x_axis)  # right-handed (X=right,Y=depth/normal,Z=up): Z x X = Y

    return Rotation.from_matrix(np.column_stack([x_axis, y_axis, z_axis]))


def fuse_panel_pose(
    detections: list[MarkerDetection],
    marker_local_positions: dict[int, tuple[float, float, float]] = KNOWN_MARKER_LOCAL_POSITIONS,
) -> FusedPanelPose | None:
    """Fuse 1-3 marker detections into a single panel pose candidate.

    ``marker_local_positions`` maps each ArUco ID actually mounted on
    THIS panel to which of its 3 physical mounts it's at — defaults to
    this repo's own dev/sim assignment (IDs 20/21/22); pass an explicit
    mapping (e.g. built from panel_pose_fuser_node.py's
    marker_id_top_left/top_right/bottom_left parameters) for a
    real/competition panel using a different ID assignment.

    Returns ``None`` if no detection has a recognized (known) marker ID.
    Unknown marker IDs (not in ``marker_local_positions``) are silently
    ignored, not an error — the camera may see other, unrelated ArUco
    tags in the same frame.

    Disagreement fields let the caller decide whether the fused result is
    trustworthy enough to act on (e.g. drive the arm) — this function
    itself never refuses to return a fused pose, it just reports how much
    the individual candidates disagreed. These are always computed from
    the per-marker orientation candidates below, even when the fused
    orientation itself came from the (more accurate) layout-based fit —
    they measure raw sensor agreement, not fused-result error.
    """
    known = [d for d in detections if d.marker_id in marker_local_positions]
    if not known:
        return None

    candidates = [_candidate_panel_pose(d, marker_local_positions) for d in known]
    positions = np.stack([t for t, _ in candidates])
    rotations = Rotation.concatenate([r for _, r in candidates])

    fused_position = positions.mean(axis=0)
    # Prefer the layout-based fit when possible — see
    # _fit_orientation_from_marker_layout's docstring for why it's more
    # accurate. Falls back to Markley's method (mean of quaternions via
    # the dominant eigenvector of their outer-product sum, what
    # scipy's Rotation.mean() implements) when fewer than 3 markers are
    # visible and a layout-only fit isn't possible.
    layout_rotation = _fit_orientation_from_marker_layout(known, marker_local_positions)
    fused_rotation = layout_rotation if layout_rotation is not None else rotations.mean()

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
