#!/usr/bin/env python3
# Copyright (c) 2026 UCU Space Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""Compute camera calibration parameters from saved chessboard images."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


DEFAULT_IMAGE_DIR = "media/calibration"
DEFAULT_OUTPUT = "camera_calibration.npz"
SUPPORTED_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png"}


def positive_int(value):
    """Parse an integer constrained to a usable chessboard dimension."""
    parsed = int(value)
    if parsed < 3:
        raise argparse.ArgumentTypeError("must be at least 3")
    return parsed


def positive_float(value):
    """Parse a positive floating-point measurement."""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and greater than 0")
    return parsed


def parse_args():
    """Parse command-line calibration settings."""
    parser = argparse.ArgumentParser(
        description="Calibrate a camera from saved chessboard images."
    )
    parser.add_argument("--image-dir", default=DEFAULT_IMAGE_DIR)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--cols",
        type=positive_int,
        required=True,
        help="Number of inner chessboard corners per row.",
    )
    parser.add_argument(
        "--rows",
        type=positive_int,
        required=True,
        help="Number of inner chessboard corners per column.",
    )
    parser.add_argument(
        "--square-size-m",
        type=positive_float,
        required=True,
        help="Physical chessboard square size in meters.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display detected corners while processing images.",
    )
    return parser.parse_args()


def image_paths(image_dir):
    """Return supported image files immediately inside a directory."""
    return sorted(
        path
        for path in image_dir.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS
    )


def build_object_points(cols, rows, square_size_m):
    """Build planar chessboard points in metric target coordinates."""
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return points * square_size_m


def main():
    """Detect chessboards, calibrate the camera, and save the result."""
    args = parse_args()
    image_dir = Path(args.image_dir).expanduser()
    if not image_dir.is_dir():
        raise SystemExit(f"Calibration image directory not found: {image_dir}")

    paths = image_paths(image_dir)
    if not paths:
        raise SystemExit(f"No calibration images found in: {image_dir}")

    pattern_size = (args.cols, args.rows)
    object_template = build_object_points(
        args.cols, args.rows, args.square_size_m
    )
    object_points = []
    image_points = []
    image_size = None

    print(f"[CALIBRATE] Reading {len(paths)} images from {image_dir}")

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"[CALIBRATE] skipped unreadable image: {path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_image_size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if not found:
            print(f"[CALIBRATE] no chessboard corners: {path}")
            continue

        if image_size is None:
            image_size = current_image_size
        elif current_image_size != image_size:
            raise SystemExit(
                "Calibration images must have one resolution; "
                f"expected {image_size[0]}x{image_size[1]}, but {path} is "
                f"{current_image_size[0]}x{current_image_size[1]}."
            )

        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            ),
        )
        object_points.append(object_template.copy())
        image_points.append(refined)
        print(f"[CALIBRATE] accepted: {path}")

        if args.show:
            preview = image.copy()
            cv2.drawChessboardCorners(preview, pattern_size, refined, found)
            cv2.imshow("Calibration corners", preview)
            if cv2.waitKey(250) & 0xFF in (ord("q"), 27):
                break

    if args.show:
        cv2.destroyAllWindows()

    if len(object_points) < 5:
        raise SystemExit(
            "Need at least 5 valid chessboard images for calibration; "
            f"found {len(object_points)}."
        )

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as output_file:
        np.savez(
            output_file,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=np.asarray(image_size, dtype=np.int32),
            rms=np.asarray(rms),
            rvecs=np.asarray(rvecs),
            tvecs=np.asarray(tvecs),
            cols=np.asarray(args.cols),
            rows=np.asarray(args.rows),
            square_size_m=np.asarray(args.square_size_m),
        )

    print(f"[CALIBRATE] valid images: {len(object_points)}")
    print(f"[CALIBRATE] image size: {image_size[0]}x{image_size[1]}")
    print(f"[CALIBRATE] RMS reprojection error: {rms:.6f}")
    print(f"[CALIBRATE] saved: {output}")


if __name__ == "__main__":
    main()
