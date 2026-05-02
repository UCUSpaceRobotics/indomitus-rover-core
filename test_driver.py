#!/usr/bin/env python3
"""
Test script for damiao_driver.
Publishes WheelTargets and prints CAN frames + diagnostics to stdout.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import struct
import threading
import time

from can_msgs.msg import Frame
from indomitus_msgs.msg import WheelTargets
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray


def decode_can_frame(frame: Frame) -> str:
    cid = frame.id
    data = bytes(frame.data[:frame.dlc])

    # Identify frame type
    if frame.dlc == 4 and (cid & 0xFF00) == 0x0200:
        esc_id = cid & 0xFF
        vel = struct.unpack('<f', data)[0]
        return f"  VELOCITY  esc_id={esc_id}  vel={vel:+.3f} rad/s (motor shaft)"

    if frame.dlc == 8 and (cid & 0xFF00) == 0x0100:
        esc_id = cid & 0xFF
        pos, vmax = struct.unpack('<ff', data)
        return f"  POS-VEL   esc_id={esc_id}  pos={pos:+.3f} rad  vmax={vmax:.1f} rad/s"

    if frame.dlc == 8 and cid <= 8:
        code = data[7]
        name = {0xFC: "ENABLE", 0xFD: "DISABLE", 0xFE: "SET_ZERO"}.get(code, f"0x{code:02X}")
        return f"  SPECIAL   esc_id={cid}  cmd={name}"

    return f"  RAW  id=0x{cid:03X}  dlc={frame.dlc}  data={data.hex()}"


class TestNode(Node):
    def __init__(self):
        super().__init__('damiao_test')
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self.can_sub = self.create_subscription(
            Frame, '/to_can_bus', self.on_can, qos)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, qos)
        self.diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.on_diagnostics, qos)
        self.wt_pub = self.create_publisher(WheelTargets, '/wheel_targets', qos)

        self.can_frames = []
        self.js_received = False
        self.diag_received = False

    def on_can(self, msg: Frame):
        self.can_frames.append(msg)

    def on_joint_states(self, msg: JointState):
        if not self.js_received:
            print("\n[/joint_states] first message:")
            for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
                print(f"  {name:30s}  pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  torque={eff:+.4f} Nm")
            self.js_received = True

    def on_diagnostics(self, msg: DiagnosticArray):
        if not self.diag_received:
            print("\n[/diagnostics] first message:")
            for s in msg.status:
                level_name = {0: "OK", 1: "WARN", 2: "ERROR"}.get(s.level, "?")
                print(f"  [{level_name}] {s.name}: {s.message}")
            self.diag_received = True


def main():
    rclpy.init()
    node = TestNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    print("=" * 60)
    print("damiao_driver test")
    print("=" * 60)

    # Spin briefly to receive enable frames from the driver
    print("\n--- Waiting 1s to capture startup enable frames ---")
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    if node.can_frames:
        print(f"\n[/to_can_bus] {len(node.can_frames)} enable frames received at startup:")
        for f in node.can_frames:
            print(decode_can_frame(f))
    else:
        print("\n[/to_can_bus] No startup frames (driver may have already started)")

    # Publish a WheelTargets and capture the resulting CAN frames
    node.can_frames.clear()
    wt = WheelTargets()
    wt.steer_angles = [0.1, 0.1, -0.1, -0.1]       # rad
    wt.drive_velocities = [5.0, -5.0, 5.0, -5.0]   # rad/s
    print("\n--- Publishing WheelTargets ---")
    print(f"  steer_angles     = {list(wt.steer_angles)}")
    print(f"  drive_velocities = {list(wt.drive_velocities)}")
    node.wt_pub.publish(wt)

    time.sleep(0.5)

    print(f"\n[/to_can_bus] {len(node.can_frames)} CAN frames produced:")
    for f in node.can_frames:
        print(decode_can_frame(f))

    # Simulate feedback from motor 1 (steer FL) to trigger joint_states
    print("\n--- Injecting simulated CAN feedback for steer FL (esc_id=1) ---")
    fb_pub = node.create_publisher(Frame, '/from_can_bus',
                                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    time.sleep(0.2)

    # Build a valid Damiao feedback frame: err=0x1 (enabled), esc_id=1
    # pos=0.5 rad, vel=5.0 rad/s, tor=1.0 Nm (motor shaft, pmax=12.5, vmax=50, tmax=10)
    def to_uint(x, xmin, xmax, bits):
        x = max(xmin, min(xmax, x))
        return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))

    p_int = to_uint(0.5, -12.5, 12.5, 16)   # 0.5 rad
    v_int = to_uint(5.0, -50.0,  50.0, 12)  # 5.0 rad/s
    t_int = to_uint(1.0, -10.0,  10.0, 12)  # 1.0 Nm

    d = bytearray(8)
    d[0] = (0x1 << 4) | 0x1   # err=Enabled(0x1), esc_id=1
    d[1] = (p_int >> 8) & 0xFF
    d[2] =  p_int & 0xFF
    d[3] =  v_int >> 4
    d[4] = ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    d[5] =  t_int & 0xFF
    d[6] = 45   # t_mos = 45°C
    d[7] = 38   # t_rotor = 38°C

    fb_frame = Frame()
    fb_frame.id  = 0x000   # MST_ID
    fb_frame.dlc = 8
    fb_frame.data = list(d) + [0] * (8 - len(d))
    fb_pub.publish(fb_frame)

    time.sleep(0.5)

    # Wait for 1 Hz diagnostics
    print("\n--- Waiting for /diagnostics (1 Hz timer) ---")
    time.sleep(1.5)

    print("\n" + "=" * 60)
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
