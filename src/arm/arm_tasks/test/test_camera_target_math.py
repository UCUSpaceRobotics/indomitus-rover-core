"""Pure-math tests for camera_target_math.py — no ROS graph needed."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from arm_tasks.camera_target_math import (
    compose_transforms,
    compute_standoff_distance,
    compute_target_tip_pose,
)


def test_compose_identity_is_noop():
    t_a_b = ((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    pos, quat = compose_transforms(t_a_b, identity)
    np.testing.assert_allclose(pos, t_a_b[0], atol=1e-9)
    np.testing.assert_allclose(quat, t_a_b[1], atol=1e-9)


def test_compose_pure_translation_chain():
    t_a_b = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    t_b_c = ((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    pos, quat = compose_transforms(t_a_b, t_b_c)
    np.testing.assert_allclose(pos, (1.0, 2.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(quat, (0.0, 0.0, 0.0, 1.0), atol=1e-9)


def test_compose_rotation_applied_to_second_translation():
    # A 90 deg yaw of B means B's local +Y (t_b_c) becomes A's -X.
    r = Rotation.from_euler('z', 90, degrees=True)
    t_a_b = ((0.0, 0.0, 0.0), tuple(r.as_quat().tolist()))
    t_b_c = ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    pos, _ = compose_transforms(t_a_b, t_b_c)
    np.testing.assert_allclose(pos, (-1.0, 0.0, 0.0), atol=1e-9)


class TestStandoffDistance:
    def test_wider_panel_needs_more_distance(self):
        narrow = compute_standoff_distance(
            fx=500, fy=500, image_width=640, image_height=480,
            panel_width=0.2, panel_height=0.2,
        )
        wide = compute_standoff_distance(
            fx=500, fy=500, image_width=640, image_height=480,
            panel_width=1.0, panel_height=0.2,
        )
        assert wide.distance > narrow.distance

    def test_below_floor_is_rejected(self):
        result = compute_standoff_distance(
            fx=5000, fy=5000, image_width=640, image_height=480,
            panel_width=0.01, panel_height=0.01,
            min_floor=0.15,
        )
        assert not result.within_bounds
        assert 'min_floor' in result.reason

    def test_above_max_reach_is_rejected(self):
        # A large fx means a narrow (telephoto) FOV, which needs a large
        # standoff to fit the same panel width/height in frame.
        result = compute_standoff_distance(
            fx=5000, fy=5000, image_width=640, image_height=480,
            panel_width=0.33, panel_height=0.45,
            max_reach=0.75,
        )
        assert not result.within_bounds
        assert 'max_reach' in result.reason

    def test_sim_camera_numbers_are_within_bounds(self):
        # arm_camera's sim intrinsics: horizontal_fov=1.047rad, 640x480 ->
        # fx = (W/2) / tan(hfov/2).
        fx = (640 / 2) / np.tan(1.047 / 2)
        fy = fx * (480 / 640)  # square pixels assumption
        result = compute_standoff_distance(
            fx=fx, fy=fy, image_width=640, image_height=480,
            panel_width=0.33, panel_height=0.45,
        )
        assert result.within_bounds
        assert 0.15 < result.distance < 0.75


class TestTargetTipPose:
    def test_identity_camera_to_tip_and_panel_orientation(self):
        panel_pose_in_camera = ((1.0, 2.0, 0.5), (0.0, 0.0, 0.0, 1.0))
        camera_to_tip = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        position, orientation = compute_target_tip_pose(
            panel_pose_in_camera, camera_to_tip, standoff=0.5)
        # Outward normal is -Y (panel front face convention), so the
        # target camera sits at panel_position + (0, -standoff, 0) when
        # both panel and camera_to_tip are identity-oriented.
        np.testing.assert_allclose(position, (1.0, 1.5, 0.5), atol=1e-9)
        r = Rotation.from_quat(orientation)
        np.testing.assert_allclose(r.as_euler('xyz', degrees=True), (-90.0, 0.0, 0.0), atol=1e-6)

    def test_camera_to_tip_offset_is_applied(self):
        panel_pose_in_camera = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        # tip sits 0.2m further along the (target) camera's own +Z from camera.
        camera_to_tip = ((0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
        position, _ = compute_target_tip_pose(
            panel_pose_in_camera, camera_to_tip, standoff=1.0)
        # Target camera position alone would be (0,-1,0); Rx(-90) maps
        # local +Z to world +Y, so the 0.2m tip offset adds (0, 0.2, 0).
        np.testing.assert_allclose(position, (0.0, -0.8, 0.0), atol=1e-9)

    def test_further_standoff_moves_target_further_along_normal(self):
        panel_pose_in_camera = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        camera_to_tip = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        near, _ = compute_target_tip_pose(panel_pose_in_camera, camera_to_tip, standoff=0.3)
        far, _ = compute_target_tip_pose(panel_pose_in_camera, camera_to_tip, standoff=0.6)
        assert far[1] < near[1]  # further out means more negative Y
