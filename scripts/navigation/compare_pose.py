#!/usr/bin/env python3
"""Compare RViz's estimated rover pose against Gazebo ground truth.

Runs `ros2 run tf2_ros tf2_echo <target> <source>` and
`ign topic -e -t /world/<world>/pose/info -n 1` concurrently, parses one
reading from each, and reports the position/yaw error between them.

See scripts/navigation/README.md for background and companion script.

Requirements: Python 3 standard library only - no pip install needed.
Needs a sourced ROS 2 environment (`ros2`/`tf2_ros` on PATH) and Gazebo's
`ign` (or `gz`, on newer Gazebo) CLI on PATH.

Usage:
    python3 compare_pose.py
    python3 compare_pose.py --target-frame odom
    python3 compare_pose.py --world nav2_test_world
"""

import argparse
import math
import os
import re
import signal
import subprocess
import time


def get_synchronized_poses(target_frame, source_frame, world, model, tf_duration, gz_duration):
    """Start tf2_echo, wait for it to settle, snapshot Gazebo, then stop tf2_echo."""
    tf_cmd = ['ros2', 'run', 'tf2_ros', 'tf2_echo', target_frame, source_frame]
    tf_proc = subprocess.Popen(
        tf_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)

    # Give tf2_echo time to discover the transform and stream at least one
    # reading before we read anything back.
    time.sleep(tf_duration)

    gz_cmd = ['ign', 'topic', '-e', '-t', f'/world/{world}/pose/info', '-n', '1']
    try:
        gz_result = subprocess.run(gz_cmd, capture_output=True, text=True, timeout=gz_duration)
        gz_output = gz_result.stdout
    except subprocess.TimeoutExpired:
        raise RuntimeError(f'Gazebo topic snapshot timed out after {gz_duration}s')

    # `ros2 run` spawns the real node as a child of its own wrapper process,
    # so killing just tf_proc would leave that grandchild holding the stdout
    # pipe open forever. start_new_session above put the whole tree in its
    # own process group so it can be torn down as a unit.
    pgid = os.getpgid(tf_proc.pid)
    os.killpg(pgid, signal.SIGINT)  # cleanly shuts down the ROS 2 node
    try:
        tf_output, _ = tf_proc.communicate(timeout=2.0)
    except subprocess.TimeoutExpired:
        os.killpg(pgid, signal.SIGKILL)
        tf_output, _ = tf_proc.communicate()

    return tf_output, gz_output


def parse_tf2_echo(output):
    translations = re.findall(r'Translation:\s*\[([^\]]+)\]', output)
    # Matches both "Quaternion (xyzw) [...]" and older/plain "Quaternion [...]".
    quats = re.findall(r'Quaternion.*?\[([^\]]+)\]', output)
    if not translations or not quats:
        raise RuntimeError(f'Could not parse tf2_echo output:\n{output}')

    x, y, z = (float(v) for v in translations[-1].split(','))
    qx, qy, qz, qw = (float(v) for v in quats[-1].split(','))
    return (x, y, z), (qx, qy, qz, qw)


def parse_gz_pose(output, model_name):
    blocks = re.split(r'\n(?=name: )', output)
    for block in blocks:
        if f'name: "{model_name}"' not in block:
            continue
        pos_match = re.search(
            r'position\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)',
            block)
        ori_match = re.search(
            r'orientation\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*'
            r'z:\s*([-\d.eE]+)\s*w:\s*([-\d.eE]+)',
            block)
        if pos_match and ori_match:
            x, y, z = (float(v) for v in pos_match.groups())
            qx, qy, qz, qw = (float(v) for v in ori_match.groups())
            return (x, y, z), (qx, qy, qz, qw)
    raise RuntimeError(f"Could not find model '{model_name}' in gz pose output:\n{output}")


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--target-frame', default='map',
                         help='tf2_echo target frame, e.g. map or odom')
    parser.add_argument('--source-frame', default='base_footprint',
                         help='tf2_echo source frame')
    parser.add_argument('--world', default='mars_yard',
                         help='Gazebo world name, e.g. mars_yard or nav2_test_world')
    parser.add_argument('--model', default='indomitus_rover',
                         help='Gazebo model name to match in /world/<world>/pose/info')
    parser.add_argument('--tf-duration', type=float, default=3.0,
                         help='Seconds to let tf2_echo stream before reading the last sample')
    parser.add_argument('--gz-duration', type=float, default=5.0,
                         help='Max seconds to wait for the Gazebo pose snapshot')
    args, _ = parser.parse_known_args()

    print(f'Reading {args.target_frame} -> {args.source_frame} from tf2_echo...')
    print(f"Reading ground truth pose of '{args.model}' from Gazebo...")

    tf_output, gz_output = get_synchronized_poses(
        args.target_frame, args.source_frame, args.world, args.model,
        args.tf_duration, args.gz_duration)

    (tx, ty, _tz), (tqx, tqy, tqz, tqw) = parse_tf2_echo(tf_output)
    (gx, gy, _gz), (gqx, gqy, gqz, gqw) = parse_gz_pose(gz_output, args.model)

    tf_yaw = math.degrees(yaw_from_quat(tqx, tqy, tqz, tqw))
    gz_yaw = math.degrees(yaw_from_quat(gqx, gqy, gqz, gqw))

    pos_error = math.hypot(tx - gx, ty - gy)
    yaw_error = (tf_yaw - gz_yaw + 180) % 360 - 180  # wrap to [-180, 180]

    print()
    print(f"{'':22} {'x':>10} {'y':>10} {'yaw (deg)':>12}")
    print(f"{'RViz (' + args.target_frame + ')':22} {tx:10.3f} {ty:10.3f} {tf_yaw:12.2f}")
    print(f"{'Gazebo ground truth':22} {gx:10.3f} {gy:10.3f} {gz_yaw:12.2f}")
    print()
    print(f'Position error: {pos_error:.3f} m')
    print(f'Yaw error:      {yaw_error:+.2f} deg (abs: {abs(yaw_error):.2f} deg)')


if __name__ == '__main__':
    main()
