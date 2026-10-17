import math

import numpy as np
import pytest

from rover_probe_detection import geometry
from rover_probe_detection.geometry import Ray

K = [600.0, 0.0, 480.0, 0.0, 600.0, 300.0, 0.0, 0.0, 1.0]
D = [-0.25, 0.08, 0.001, -0.002, 0.0]
W, H = 960, 600

# optical axes in a REP-103 body frame (x fwd, y left, z up): z_opt = x, x_opt = -y, y_opt = -z
BODY_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def camera(name, origin, pitch_deg, yaw_deg=0.0):
    """(name, origin, R world<-optical) for a camera looking forward, pitched down."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return name, np.array(origin, dtype=float), rz @ ry @ BODY_FROM_OPTICAL


# Roughly the rover's: mast, front-bottom, arm camera.
CAMERAS = [
    camera('mast', [-0.31, 0.02, 1.30], 25),
    camera('fr_bottom', [0.15, 0.0, 0.32], 15),
    camera('arm', [0.6, 0.3, 0.7], 40, yaw_deg=-15),
]


def observe(cam, point, noise=0.0, rng=None):
    """Ray of `cam` to a world point via the pixel, as the node computes it; None if unseen."""
    name, origin, rot = cam
    uv = geometry.project(rot.T @ (point - origin), K, D)
    if uv is None:
        return None
    u, v = uv
    if rng is not None:
        u, v = u + rng.normal(0, noise), v + rng.normal(0, noise)
    if not (0 <= u < W and 0 <= v < H):
        return None
    return Ray(name, origin, rot @ geometry.pixel_to_ray(u, v, K, D))


def test_project_and_back_through_distortion():
    p = np.array([0.4, -0.3, 2.0])
    u, v = geometry.project(p, K, D)
    ray = geometry.pixel_to_ray(u, v, K, D)
    assert np.allclose(ray, p / np.linalg.norm(p), atol=1e-9)


def test_quaternion_matrix_is_a_rotation():
    q = np.array([0.1, -0.3, 0.2, 0.9])
    q /= np.linalg.norm(q)
    r = geometry.quaternion_matrix(*q)
    assert np.allclose(r @ r.T, np.eye(3)) and math.isclose(np.linalg.det(r), 1.0)


def test_triangulate_three_cameras_exact():
    probe = np.array([1.8, 0.2, 0.1])
    rays = [observe(c, probe) for c in CAMERAS]
    assert all(rays)
    point, rms = geometry.triangulate(rays)
    assert np.allclose(point, probe, atol=1e-6) and rms < 1e-6


def test_triangulate_with_pixel_noise():
    rng = np.random.default_rng(0)
    probe = np.array([2.0, -0.3, 0.1])
    errors = []
    for _ in range(200):
        rays = [observe(c, probe, 1.0, rng) for c in CAMERAS]
        point, _ = geometry.triangulate(rays)
        errors.append(np.linalg.norm(point - probe))
    assert np.median(errors) < 0.03       # 1 px noise at ~2 m: a few cm


def test_parallel_rays_are_undetermined():
    d = np.array([1.0, 0.0, 0.0])
    rays = [Ray('a', np.zeros(3), d), Ray('b', np.array([0.0, 0.0, 1.0]), d)]
    assert geometry.triangulate(rays)[0] is None


def test_associate_three_probes():
    probes = [np.array(p) for p in ([1.6, 0.4, 0.1], [2.2, -0.2, 0.1], [1.4, -0.6, 0.1])]
    rays, truth = [], []
    for i, probe in enumerate(probes):
        for cam in CAMERAS:
            r = observe(cam, probe)
            if r is not None:
                rays.append(r)
                truth.append(i)
    groups, singles = geometry.associate(rays, max_gap=0.05, min_angle=math.radians(2),
                                         max_range=8.0)
    assert not singles
    assert sorted(len(g) for g in groups) == sorted(truth.count(i) for i in range(3))
    for g in groups:
        assert len({truth[k] for k in g}) == 1                  # one probe per group
        assert len({rays[k].camera for k in g}) == len(g)       # one ray per camera
        point, _ = geometry.triangulate([rays[k] for k in g])
        assert np.allclose(point, probes[truth[g[0]]], atol=1e-6)


def test_single_view_goes_to_ground_plane():
    probe = np.array([1.5, 0.3, 0.0])
    ray = observe(CAMERAS[0], probe)
    groups, singles = geometry.associate([ray], 0.05, 0.03, 8.0)
    assert groups == [] and singles == [0]
    hit = geometry.intersect_plane(ray, np.zeros(3), np.array([0.0, 0.0, 1.0]))
    assert np.allclose(hit, probe, atol=1e-9)


def test_plane_behind_or_parallel_is_none():
    up = Ray('a', np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]))
    flat = Ray('a', np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
    n = np.array([0.0, 0.0, 1.0])
    assert geometry.intersect_plane(up, np.zeros(3), n) is None
    assert geometry.intersect_plane(flat, np.zeros(3), n) is None


def test_rays_meeting_behind_cameras_are_not_paired():
    # Two cameras looking away from each other: their lines cross behind both.
    a = Ray('a', np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0, 0.0]))
    b = Ray('b', np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    groups, singles = geometry.associate([a, b], 0.1, 0.03, 8.0)
    assert groups == [] and singles == [0, 1]


@pytest.mark.parametrize('model', ['plumb_bob', 'rational_polynomial'])
def test_supported_models(model):
    assert geometry.supported_model(model, D)
    assert not geometry.supported_model('equidistant', D)
    assert geometry.supported_model('equidistant', [0.0] * 4)
