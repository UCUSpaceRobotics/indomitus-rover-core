"""Pure math for panel_align_node: standoff distance + target tip_link pose.

No ROS imports on purpose — unit-testable without a running ROS graph
(see ``test/test_camera_target_math.py``). Frame/transform composition
follows one consistent convention throughout this module and
``panel_align_node.py``:

    A transform ``T_A_B = (position, orientation_xyzw)`` means "B's pose,
    expressed in frame A". Composing ``T_A_B`` with ``T_B_C`` (B's frame
    with C's pose in it) gives ``T_A_C``, via ``compose_transforms``.

This is the exact trap flagged during planning: composing a *fixed rigid
offset* (camera<->tip_link, both fixed children of arm_end_effector_link)
onto a *freshly computed target pose* is plain transform chaining, not a
``tf2_geometry_msgs.do_transform_pose``-style re-expression of a
stationary point — see ``compute_target_tip_pose`` below for exactly
where this applies.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

Transform = tuple[tuple[float, float, float], tuple[float, float, float, float]]
"""(position xyz, orientation xyzw) — see module docstring for the T_A_B convention."""


def compose_transforms(t_a_b: Transform, t_b_c: Transform) -> Transform:
    """T_A_B composed with T_B_C -> T_A_C (B's-pose-in-A, C's-pose-in-B -> C's-pose-in-A)."""
    pos_a_b, quat_a_b = t_a_b
    pos_b_c, quat_b_c = t_b_c
    r_a_b = Rotation.from_quat(quat_a_b)
    r_b_c = Rotation.from_quat(quat_b_c)

    r_a_c = r_a_b * r_b_c
    pos_a_c = np.array(pos_a_b) + r_a_b.apply(np.array(pos_b_c))
    return tuple(pos_a_c.tolist()), tuple(r_a_c.as_quat().tolist())


@dataclass(frozen=True)
class StandoffResult:
    distance: float
    within_bounds: bool
    reason: str  # empty if within_bounds


def compute_standoff_distance(
    fx: float, fy: float, image_width: int, image_height: int,
    panel_width: float, panel_height: float,
    margin_multiplier: float = 1.15,
    min_floor: float = 0.15,
    max_reach: float = 0.75,
) -> StandoffResult:
    """FOV-fit standoff distance: how far back the camera must be for the
    whole panel to fit in frame on both axes, from *live* CameraInfo
    intrinsics (not any static/sim-only FOV constant).

    ``margin_multiplier`` only pads the FOV-fit result a little (the panel
    occupies ~1/margin_multiplier of the frame on its binding axis) — it
    is not a safety margin. Actual safety is real MoveGroup collision
    checking against the panel's CollisionObject (see panel_align_node);
    ``min_floor``/``max_reach`` here are just sanity bounds on the
    computed distance, not substitutes for that.
    """
    hfov = 2 * np.arctan(image_width / (2 * fx))
    vfov = 2 * np.arctan(image_height / (2 * fy))

    d_h = (panel_width / 2) / np.tan(hfov / 2)
    d_v = (panel_height / 2) / np.tan(vfov / 2)
    distance = float(margin_multiplier * max(d_h, d_v))

    if distance < min_floor:
        return StandoffResult(
            distance, False,
            f'computed standoff {distance:.3f}m below min_floor {min_floor:.3f}m '
            '(likely a bad/degenerate panel detection)',
        )
    if distance > max_reach:
        return StandoffResult(
            distance, False,
            f'computed standoff {distance:.3f}m exceeds max_reach {max_reach:.3f}m '
            '(panel too far to frame it within a safe reach)',
        )
    return StandoffResult(distance, True, '')


def compute_target_tip_pose(
    panel_pose_in_camera: Transform,
    camera_to_tip: Transform,
    standoff: float,
    world_up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> Transform:
    """Target tip_link pose, in the camera's current frame.

    Args:
        panel_pose_in_camera: T_C_P — the fused panel detection, i.e. the
            panel's pose as seen right now from the current camera frame.
        camera_to_tip: T_C_T — tip_link's pose in the camera's frame. This
            is a *constant* (both links are fixed-joint children of
            arm_end_effector_link), so the caller should look it up via
            TF **once at startup** and cache it, not re-query it per call:
            ``tf_buffer.lookup_transform(target_frame='arm_camera_optical_frame',
            source_frame=tip_link, time=Time())``.
        standoff: desired distance (m) from the panel to the target
            camera position, along the panel's own outward normal.
        world_up: vertical direction, in the same frame as
            ``panel_pose_in_camera`` — camera roll is levelled against
            this rather than the panel's own (possibly tilted) local +Z,
            so a panel that isn't perfectly plumb doesn't tilt the camera
            to match it. Defaults to (0,0,1), correct whenever the caller
            resolved the detection into ``arm_mount_link`` first (as
            ``panel_align_node.py`` always does).

    Returns:
        T_C_Ttarget — the target tip_link pose, in whatever frame "C"
        (``panel_pose_in_camera``'s first frame) was expressed in. Despite
        the name, that doesn't have to literally be the camera's own
        current frame — ``panel_align_node.align_to_panel()`` resolves the
        detection into the fixed ``arm_mount_link`` frame *before* calling
        this, and hands the result to MoveIt with
        ``header.frame_id='arm_mount_link'`` accordingly. Do the same:
        feeding a robot-link-attached frame_id like
        ``arm_camera_optical_frame`` straight to a MoveIt goal constraint,
        on the theory that MoveGroup resolves it against the live robot
        state, was tried and confirmed live to fail — OMPL's goal-tree
        sampling failed 100% of the time (all threads, full
        allowed_planning_time) for a target independently confirmed
        reachable via a direct ``/compute_ik`` call. See
        ``panel_align_node.py``'s own ``PLANNING_FRAME`` comment for the
        full story.

    Derivation: the panel's front face is at local -Y (arm approaches
    from -Y, per panel_macro.xacro), so outward normal = -Y_panel and the
    target's forward axis is +Y_panel. Image-down (ROS optical
    convention, +Y down) is Gram-Schmidt'd against world_up instead of
    the panel's own +Z — reduces to the old fixed Rx(-90 deg) result
    exactly whenever the panel's own +Z already equals world_up.
    """
    panel_pos, panel_quat = panel_pose_in_camera
    r_panel = Rotation.from_quat(panel_quat)

    forward = r_panel.apply([0.0, 1.0, 0.0])
    cameratarget_pos = np.array(panel_pos) + r_panel.apply([0.0, -standoff, 0.0])

    right = np.cross(forward, world_up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-6:
        # forward parallel to world_up (camera would look straight up/
        # down) — fall back to the panel's own local X so this never
        # divides by zero.
        right = r_panel.apply([1.0, 0.0, 0.0])
        right = right - np.dot(right, forward) * forward
        right_norm = np.linalg.norm(right)
    right = right / right_norm
    down = np.cross(forward, right)

    cameratarget_quat = Rotation.from_matrix(np.column_stack([right, down, forward])).as_quat()
    t_camera_cameratarget: Transform = (
        tuple(cameratarget_pos.tolist()), tuple(cameratarget_quat.tolist()))
    return compose_transforms(t_camera_cameratarget, camera_to_tip)
