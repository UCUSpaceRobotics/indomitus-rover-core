"""Camera geometry for probe localisation. numpy only, no ROS.

A detection is a pixel; with the camera's intrinsics (CameraInfo K, D) it is a
ray from the camera's optical centre, and with TF that ray is expressed in a
common frame. Rays from several cameras that see the same probe meet (nearly)
at the probe: `associate` groups rays by probe, `triangulate` finds the point
closest to all rays of a group. A probe only one camera sees can still be
placed by intersecting its ray with the ground plane (`intersect_plane`).

Frames: REP-103 optical (z into the scene, x right, y down), as CameraInfo and
rover_cameras' image_raw use. Images are unrectified, so K and D are used, not P.
"""

from dataclasses import dataclass
import math

import numpy as np

# Distortion models whose first coefficients are k1 k2 p1 p2 k3 (rational adds k4 k5 k6).
BROWN_CONRADY = ('plumb_bob', 'rational_polynomial')


@dataclass
class Ray:
    """One detection as a ray in the common frame."""
    camera: str
    origin: np.ndarray       # camera optical centre
    direction: np.ndarray    # unit vector
    score: float = 1.0


def _coeffs(d):
    c = list(d) + [0.0] * 8
    return c[:8]


def _radial(r2, c, model):
    k1, k2, _, _, k3, k4, k5, k6 = c
    num = 1.0 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    if model == 'rational_polynomial':
        return num / (1.0 + k4 * r2 + k5 * r2 ** 2 + k6 * r2 ** 3)
    return num


def _tangential(x, y, r2, c):
    p1, p2 = c[2], c[3]
    return (2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
            p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y)


def supported_model(model, d):
    """False for a distortion model this module cannot undo (it would be ignored)."""
    return not any(d) or model in BROWN_CONRADY


def distort(x, y, d, model):
    """Normalised undistorted (x, y) -> normalised distorted."""
    if not any(d) or model not in BROWN_CONRADY:
        return x, y
    c = _coeffs(d)
    r2 = x * x + y * y
    radial = _radial(r2, c, model)
    dx, dy = _tangential(x, y, r2, c)
    return x * radial + dx, y * radial + dy


def undistort(x, y, d, model, iterations=20):
    """Normalised distorted (x, y) -> undistorted, by fixed-point iteration."""
    if not any(d) or model not in BROWN_CONRADY:
        return x, y
    c = _coeffs(d)
    x0, y0 = x, y
    for _ in range(iterations):
        r2 = x * x + y * y
        dx, dy = _tangential(x, y, r2, c)
        radial = _radial(r2, c, model)
        x, y = (x0 - dx) / radial, (y0 - dy) / radial
    return x, y


def pixel_to_ray(u, v, k, d=(), model='plumb_bob'):
    """Unit direction (optical frame) of the ray through pixel (u, v)."""
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    x, y = undistort((u - cx) / fx, (v - cy) / fy, d, model)
    ray = np.array([x, y, 1.0])
    return ray / np.linalg.norm(ray)


def project(point, k, d=(), model='plumb_bob'):
    """Pixel (u, v) of an optical-frame point, or None behind the camera."""
    if point[2] <= 1e-9:
        return None
    x, y = distort(point[0] / point[2], point[1] / point[2], d, model)
    return k[0] * x + k[2], k[4] * y + k[5]


def quaternion_matrix(x, y, z, w):
    """3x3 rotation of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def triangulate(rays):
    """(point, rms distance to the rays) closest to all rays, or (None, inf).

    Least squares over the perpendicular distances: sum (I - d d^T)(p - o) = 0.
    None when the rays are (nearly) parallel and the point is undetermined.
    """
    a = np.zeros((3, 3))
    b = np.zeros(3)
    for r in rays:
        m = np.eye(3) - np.outer(r.direction, r.direction)
        a += m
        b += m @ r.origin
    if np.linalg.eigvalsh(a)[0] < 1e-6:
        return None, math.inf
    p = np.linalg.solve(a, b)
    rms = math.sqrt(sum(distance_to_ray(p, r) ** 2 for r in rays) / len(rays))
    return p, rms


def distance_to_ray(p, ray):
    v = p - ray.origin
    return float(np.linalg.norm(v - (v @ ray.direction) * ray.direction))


def depth_along(p, ray):
    """Signed distance of p's foot along the ray (negative: behind the camera)."""
    return float((p - ray.origin) @ ray.direction)


def closest_approach(r1, r2):
    """(gap, t1, t2): shortest distance between two lines and where it is on each."""
    w = r1.origin - r2.origin
    b = r1.direction @ r2.direction
    d = r1.direction @ w
    e = r2.direction @ w
    denom = 1.0 - b * b
    if denom < 1e-12:                       # parallel
        return float(np.linalg.norm(w - d * r1.direction)), math.inf, math.inf
    t1 = (b * e - d) / denom
    t2 = (e - b * d) / denom
    gap = np.linalg.norm((r1.origin + t1 * r1.direction) - (r2.origin + t2 * r2.direction))
    return float(gap), float(t1), float(t2)


def angle_between(r1, r2):
    return math.acos(max(-1.0, min(1.0, float(r1.direction @ r2.direction))))


def associate(rays, max_gap, min_angle, max_range):
    """Group rays that see the same probe.

    Returns (groups, singles): groups are lists of ray indices, at most one per
    camera, of 2+ cameras whose rays pass within `max_gap` of one point in front
    of all of them; singles are the indices left over. Greedy: the closest pair
    of rays from two cameras founds a group, rays of further cameras join it if
    they pass within `max_gap` of its point.
    """
    pairs = []
    for i in range(len(rays)):
        for j in range(i + 1, len(rays)):
            ri, rj = rays[i], rays[j]
            if ri.camera == rj.camera or angle_between(ri, rj) < min_angle:
                continue
            gap, ti, tj = closest_approach(ri, rj)
            if gap <= max_gap and 0.0 < ti <= max_range and 0.0 < tj <= max_range:
                pairs.append((gap, i, j))
    pairs.sort()

    used = set()
    groups = []
    for _, i, j in pairs:
        if i in used or j in used:
            continue
        group = [i, j]
        used.update(group)
        point, _ = triangulate([rays[k] for k in group])
        cameras = {rays[i].camera, rays[j].camera}
        candidates = sorted(
            (distance_to_ray(point, rays[k]), k) for k in range(len(rays))
            if k not in used and rays[k].camera not in cameras
            and 0.0 < depth_along(point, rays[k]) <= max_range)
        for dist, k in candidates:
            if dist > max_gap or rays[k].camera in cameras:
                continue
            group.append(k)
            used.add(k)
            cameras.add(rays[k].camera)
            point, _ = triangulate([rays[m] for m in group])
        groups.append(group)
    singles = [k for k in range(len(rays)) if k not in used]
    return groups, singles


def intersect_plane(ray, plane_point, plane_normal, max_range=math.inf):
    """Where the ray hits the plane in front of the camera, or None."""
    denom = float(ray.direction @ plane_normal)
    if abs(denom) < 1e-9:
        return None
    t = float((plane_point - ray.origin) @ plane_normal) / denom
    if t <= 0.0 or t > max_range:
        return None
    return ray.origin + t * ray.direction
