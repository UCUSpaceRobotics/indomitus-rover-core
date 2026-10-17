"""Probe detector backends: an RGB image in, probe detections (pixels) out. No ROS.

NOT IMPLEMENTED YET — the model comes from the course project
(github.com/yaryna-zabchuk/ai-course-erc-sample-pose). Plan:

1. Model contract (agree with the course project, keep in its README):
   - file: ONNX export of YOLOv8-pose (`model.export(format='onnx')`), published
     as a GitHub Release asset with a version; never committed here;
   - input: imgsz (e.g. 640), letterboxed, RGB, float 0..1, NCHW;
   - classes: ['probe']; keypoints: 2, index 0 = top of the cylinder, 1 = bottom;
   - output (1, 4 + n_classes + 3 * n_kpts, N): cx cy w h, class scores, (x, y, conf) per keypoint.
2. `OnnxPoseDetector(Detector)` here: onnxruntime session (CPU now, CUDA/TensorRT
   execution provider on the Jetson once the image has them), letterbox, decode,
   NMS, map boxes and keypoints back to the input image's pixels.
3. Check it against the course project's own `infer.py` on the same frames
   (test/ with a few dataset images and their expected detections).
4. On the Jetson: build a TensorRT engine from the ONNX once, on the device, and
   cache it (an engine is tied to the GPU and TensorRT version).
5. Triangulate keypoints, not only the box centre: top and bottom separately
   give the probe's 3D axis (its tilt), which the arm needs for grasping.

Until then `make_detector('none')` returns None and probe_detector_node idles;
`fake_detections_node` stands in for testing the triangulation.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class Detection:
    """One probe in one image, in that image's pixels."""
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    score: float
    class_id: str = 'probe'
    # (u, v, confidence) per keypoint; for the pose model [top, bottom].
    keypoints: list = field(default_factory=list)

    @property
    def center(self):
        return (self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0


class Detector(ABC):

    @abstractmethod
    def detect(self, image_rgb):
        """Detections in an HxWx3 uint8 RGB image (numpy)."""


BACKENDS = ('none',)


def make_detector(backend, **kwargs):
    """The detector for `backend`, or None for 'none' (not implemented yet)."""
    if backend == 'none':
        return None
    raise ValueError(f'unknown detector backend {backend!r} (have {", ".join(BACKENDS)})')
