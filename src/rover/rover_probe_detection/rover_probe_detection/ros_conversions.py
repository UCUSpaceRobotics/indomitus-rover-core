"""Small helpers between ROS messages and numpy / detector types."""

import numpy as np
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from rover_probe_detection.geometry import quaternion_matrix

CHANNELS = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}


def image_to_rgb(msg):
    """HxWx3 uint8 RGB view of a sensor_msgs/Image (rgb8, bgr8 or mono8)."""
    if msg.encoding not in CHANNELS:
        raise ValueError(f'unsupported encoding {msg.encoding!r}')
    n = CHANNELS[msg.encoding]
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * n].reshape(msg.height, msg.width, n)
    if msg.encoding == 'bgr8':
        return img[:, :, ::-1]
    if n == 1:
        return np.repeat(img, 3, axis=2)
    return img


def detection_to_msg(det, header):
    """vision_msgs/Detection2D; bbox centre is the point the triangulator uses."""
    msg = Detection2D()
    msg.header = header
    cx, cy = det.center
    msg.bbox.center.position.x = cx
    msg.bbox.center.position.y = cy
    msg.bbox.size_x = det.x_max - det.x_min
    msg.bbox.size_y = det.y_max - det.y_min
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = det.class_id
    hyp.hypothesis.score = float(det.score)
    msg.results.append(hyp)
    return msg


def transform_to_rt(transform):
    """(R 3x3, t 3) of a geometry_msgs/TransformStamped."""
    q = transform.transform.rotation
    t = transform.transform.translation
    return quaternion_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
