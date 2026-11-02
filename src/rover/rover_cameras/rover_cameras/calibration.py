"""Camera calibration files, in the YAML format `camera_calibration` writes.

The ROS branch may publish at a smaller size than the camera captures (and
than it was calibrated at), so K and P are scaled to the published image.
Pure Python, no ROS: the node turns the dict into a CameraInfo.
"""

import yaml


def load(path):
    """The calibration at `path` as a dict of CameraInfo fields."""
    with open(path) as f:
        c = yaml.safe_load(f)
    return {
        'width': int(c['image_width']),
        'height': int(c['image_height']),
        'distortion_model': c.get('distortion_model', 'plumb_bob'),
        'd': [float(x) for x in c['distortion_coefficients']['data']],
        'k': [float(x) for x in c['camera_matrix']['data']],
        'r': [float(x) for x in c.get('rectification_matrix', {}).get(
            'data', [1, 0, 0, 0, 1, 0, 0, 0, 1])],
        'p': [float(x) for x in c['projection_matrix']['data']],
    }


def uncalibrated(width, height):
    """CameraInfo fields for a camera with no calibration: K = 0 says so."""
    return {
        'width': width, 'height': height, 'distortion_model': '',
        # floats only: Humble's CameraInfo rejects int elements.
        'd': [], 'k': [0.0] * 9, 'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'p': [0.0] * 12,
    }


def scaled(info, width, height):
    """`info` for an image resized to width x height (same aspect assumed).

    fx, cx and fy, cy scale with the image; distortion coefficients are
    normalised and do not change.
    """
    if (width, height) == (info['width'], info['height']):
        return dict(info)
    sx = width / info['width']
    sy = height / info['height']
    k = list(info['k'])
    p = list(info['p'])
    k[0] *= sx
    k[2] *= sx
    k[4] *= sy
    k[5] *= sy
    p[0] *= sx
    p[2] *= sx
    p[3] *= sx      # Tx = -fx * baseline
    p[5] *= sy
    p[6] *= sy
    return dict(info, width=width, height=height, k=k, p=p)
