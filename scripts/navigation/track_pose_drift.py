#!/usr/bin/env python3
"""Track drift of an estimated pose relative to its starting value.

For real-hardware field tests where there's no Gazebo ground truth: saves
the pose the first time the transform becomes available, then every
--interval seconds prints how far the current pose has moved from that
saved baseline. Drive out and back to the same physical spot - whatever
"drift from start" reads at that moment is the real localization error.

See scripts/navigation/README.md for background and companion script.

Requirements: rclpy + tf2_ros, from the sourced ROS 2 install (source
/opt/ros/<distro>/setup.bash).

Usage:
    python3 track_pose_drift.py
    python3 track_pose_drift.py --target-frame odom --interval 5
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class DriftTracker(Node):

    def __init__(self, target_frame, source_frame, interval_s):
        super().__init__('drift_tracker')
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.interval_s = interval_s
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.baseline = None
        self.start_time = None
        self._next_report = 0.0
        self._warned_waiting = False
        self.create_timer(1.0, self._tick)

    def _lookup(self):
        t = self.buffer.lookup_transform(self.target_frame, self.source_frame, Time())
        tr = t.transform.translation
        rot = t.transform.rotation
        yaw = math.degrees(yaw_from_quat(rot.x, rot.y, rot.z, rot.w))
        return tr.x, tr.y, yaw

    def _tick(self):
        now = time.monotonic()
        try:
            x, y, yaw = self._lookup()
        except tf2_ros.TransformException as ex:
            if not self._warned_waiting:
                self.get_logger().warn(
                    f'Waiting for {self.target_frame} -> {self.source_frame}: {ex}')
                self._warned_waiting = True
            return

        if self.baseline is None:
            self.baseline = (x, y, yaw)
            self.start_time = now
            self._next_report = now + self.interval_s
            self.get_logger().info(
                f'Baseline saved: x={x:.3f} y={y:.3f} yaw={yaw:.2f} deg '
                f'-- drive out and back, watch the drift below')
            return

        if now < self._next_report:
            return
        self._next_report = now + self.interval_s

        bx, by, byaw = self.baseline
        drift = math.hypot(x - bx, y - by)
        yaw_drift = (yaw - byaw + 180) % 360 - 180
        elapsed = now - self.start_time
        self.get_logger().info(
            f'[t={elapsed:6.1f}s] pos=({x:.3f}, {y:.3f}) yaw={yaw:7.2f} deg  |  '
            f'drift from start: {drift:.3f} m, yaw {yaw_drift:+.2f} deg')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--target-frame', default='map',
                         help='tf frame to treat as the fixed/reference frame, e.g. map or odom')
    parser.add_argument('--source-frame', default='base_footprint',
                         help='tf frame whose pose is tracked')
    parser.add_argument('--interval', type=float, default=10.0,
                         help='Seconds between drift reports')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DriftTracker(args.target_frame, args.source_frame, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
