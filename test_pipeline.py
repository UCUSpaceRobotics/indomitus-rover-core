#!/usr/bin/env python3
"""
chassis_driver pipeline test — interactive.

Requires chassis_driver_node + can_hw_bridge_node running:
    bash /work/run_rover.bash

Controls:
    e       Enable all motors  (publishes init frames to /to_can_bus)
    d       Disable all motors (zero → wait 1.5s → disable frames)
    1       Test: straight forward 0.5 m/s
    2       Test: spin in place left
    3       Test: turn left (Ackermann)
    4       Test: all wheels max steer angle, no drive
    s       Stop drive (publish cmd_vel zeros)
    f       Print latest /chassis/motor_states feedback
    q       Quit

Publishes to  : /cmd_vel, /to_can_bus
Subscribes to : /wheel_targets, /to_can_bus, /chassis/motor_states, /joint_states
"""
import math
import struct
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from indomitus_msgs.msg import WheelTargets, ChassisStatus

# ── Config ────────────────────────────────────────────────────────────────────

STEER_IDS = [11, 13, 15, 17]   # Steadywin, [FL, FR, RL, RR]
DRIVE_IDS = [10, 12, 14, 16]   # Damiao,    [FL, FR, RL, RR]

WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

# ── Frame builders (mirrors chassis_driver protocol headers) ──────────────────

def _frame(can_id: int, data: bytes) -> Frame:
    f = Frame()
    f.id  = can_id
    f.dlc = len(data)
    for i, b in enumerate(data):
        f.data[i] = b
    return f

def _rad_to_counts(angle_rad: float) -> int:
    return int(angle_rad * 16384 / (2 * math.pi))

# Steadywin
def sw_clear_fault(esc_id: int) -> Frame:
    return _frame(esc_id, bytes([0xAF]))

def sw_abs_position(esc_id: int, angle_rad: float) -> Frame:
    counts = _rad_to_counts(angle_rad)
    return _frame(esc_id, bytes([0xC2]) + struct.pack('<i', counts))

def sw_disable(esc_id: int) -> Frame:
    return _frame(esc_id, bytes([0xCF]))

def sw_status_query(esc_id: int) -> Frame:
    return _frame(esc_id, bytes([0xAE]))

# Damiao
def dm_set_mode(esc_id: int, mode: int) -> Frame:
    f = _frame(0x7FF, bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF, 0x55, 0x0A, mode, 0, 0, 0]))
    f.is_extended = True  # ros2_socketcan Humble rejects 0x7FF without this flag
    return f

def dm_enable(esc_id: int) -> Frame:
    return _frame(esc_id, bytes([0xFF] * 7 + [0xFC]))

def dm_disable(esc_id: int) -> Frame:
    return _frame(esc_id, bytes([0xFF] * 7 + [0xFD]))

def dm_velocity(esc_id: int, vel_rad_s: float) -> Frame:
    return _frame(0x200 + esc_id, struct.pack('<f', vel_rad_s))

# ── Frame decoder (for /to_can_bus display) ───────────────────────────────────

def decode_frame(f: Frame) -> str:
    cid  = f.id
    data = bytes(f.data[:f.dlc])

    # Damiao velocity
    if f.dlc == 4 and 0x200 < cid <= 0x210:
        vel, = struct.unpack('<f', data)
        return f"DAMIAO  VELOCITY  motor={cid-0x200:2d}  vel={vel:+.3f} rad/s"

    # Damiao setMode
    if cid == 0x7FF and f.dlc == 8 and data[2] == 0x55:
        esc = data[0] | (data[1] << 8)
        return f"DAMIAO  SET_MODE  motor={esc:2d}  mode={data[4]}"

    # Damiao enable / disable
    if f.dlc == 8 and data[7] in (0xFC, 0xFD):
        cmd = "ENABLE" if data[7] == 0xFC else "DISABLE"
        return f"DAMIAO  {cmd}   motor={cid:2d}"

    # Steadywin abs position (0xC2)
    if f.dlc == 5 and data[0] == 0xC2:
        counts, = struct.unpack('<i', data[1:5])
        angle = counts * 2 * math.pi / 16384
        return f"STEADYW ABS_POS   motor={cid:2d}  counts={counts:6d}  angle={math.degrees(angle):+.2f}°"

    # Steadywin 1-byte commands
    if f.dlc == 1:
        names = {0xAF: "CLEAR_FAULT", 0xCF: "DISABLE", 0xAE: "STATUS_QUERY"}
        name = names.get(data[0], f"CMD_0x{data[0]:02X}")
        return f"STEADYW {name}  motor={cid:2d}"

    return f"RAW  id=0x{cid:03X}  dlc={f.dlc}  data={data.hex().upper()}"

# ── ROS2 node ─────────────────────────────────────────────────────────────────

class PipelineNode(Node):
    def __init__(self):
        super().__init__('pipeline_test')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_vel_pub  = self.create_publisher(Twist,         '/cmd_vel',      qos)
        self.can_pub      = self.create_publisher(Frame,         '/to_can_bus',   qos)

        self.create_subscription(WheelTargets, '/wheel_targets',
                                 self._on_wheel_targets, qos)
        self.create_subscription(Frame, '/to_can_bus',
                                 self._on_can_frame, qos)
        self.create_subscription(ChassisStatus, '/chassis/motor_states',
                                 self._on_chassis_status, qos)

        self.last_wheel_targets: WheelTargets | None = None
        self.can_frames: list[Frame] = []
        self.last_chassis_status: ChassisStatus | None = None
        self._lock = threading.Lock()

    # subscriptions
    def _on_wheel_targets(self, msg: WheelTargets):
        with self._lock:
            self.last_wheel_targets = msg

    def _on_can_frame(self, msg: Frame):
        with self._lock:
            self.can_frames.append(msg)
            if len(self.can_frames) > 200:
                self.can_frames = self.can_frames[-200:]

    def _on_chassis_status(self, msg: ChassisStatus):
        with self._lock:
            self.last_chassis_status = msg

    # publishers
    def publish_twist(self, vx: float, wz: float):
        tw = Twist()
        tw.linear.x  = vx
        tw.angular.z = wz
        self.cmd_vel_pub.publish(tw)

    def publish_enable(self):
        """Replicate chassis_driver sendEnableFrames() via /to_can_bus."""
        print("\n[ENABLE] Sending enable sequence...")
        for esc_id in STEER_IDS:
            self.can_pub.publish(sw_clear_fault(esc_id))
        for esc_id in STEER_IDS:
            self.can_pub.publish(sw_abs_position(esc_id, 0.0))
        for esc_id in DRIVE_IDS:
            self.can_pub.publish(dm_set_mode(esc_id, 3))
        time.sleep(0.05)
        for esc_id in DRIVE_IDS:
            self.can_pub.publish(dm_enable(esc_id))
        print("[ENABLE] Done")

    def publish_disable(self):
        """Replicate chassis_driver sendDisableFrames() via /to_can_bus."""
        print("\n[DISABLE] Zeroing commands...")
        for esc_id in STEER_IDS:
            self.can_pub.publish(sw_abs_position(esc_id, 0.0))
        for esc_id in DRIVE_IDS:
            self.can_pub.publish(dm_velocity(esc_id, 0.0))
        print("[DISABLE] Waiting 1.5s to settle...")
        time.sleep(1.5)
        for esc_id in STEER_IDS:
            self.can_pub.publish(sw_disable(esc_id))
        for esc_id in DRIVE_IDS:
            self.can_pub.publish(dm_disable(esc_id))
        print("[DISABLE] All motors disabled")

    def collect_can_frames(self, duration: float) -> list[Frame]:
        """Collect /to_can_bus frames for `duration` seconds."""
        with self._lock:
            self.can_frames.clear()
        time.sleep(duration)
        with self._lock:
            return list(self.can_frames)

# ── Display helpers ───────────────────────────────────────────────────────────

def print_chassis_status(status: ChassisStatus | None):
    if status is None:
        print("  (no /chassis/motor_states received yet)")
        return
    print(f"\n  /chassis/motor_states  [{len(status.motors)} motors]")
    for m in status.motors:
        enabled = "ON " if m.enabled else "OFF"
        fault   = f" FAULT=0x{m.fault_code:02X}" if m.fault_code else ""
        kin     = f"pos={m.position:+.3f}rad vel={m.velocity:+.3f}rad/s" if m.kinematic_valid else "no-kin"
        health  = (f"V={m.voltage:.1f}V I={m.current:.3f}A T={m.temperature:.0f}°C mode={m.mode}"
                   if m.health_valid else "no-health")
        print(f"    [{enabled}] {m.motor_type:9s} esc={m.esc_id:2d} {m.joint_name:25s}"
              f"  {kin}  {health}{fault}")

def print_can_frames(frames: list[Frame]):
    # Deduplicate: keep last per (id, dlc, data[0])
    seen: dict[tuple, Frame] = {}
    for f in frames:
        key = (f.id, f.dlc, f.data[0] if f.dlc > 0 else -1)
        seen[key] = f
    print(f"\n  /to_can_bus  ({len(seen)} unique frames):")
    for f in sorted(seen.values(), key=lambda x: x.id):
        print(f"    {decode_frame(f)}")

def print_wheel_targets(wt: WheelTargets | None):
    if wt is None:
        print("  /wheel_targets: not received")
        return
    print("  /wheel_targets:")
    labels = ['FL', 'FR', 'RL', 'RR']
    angles = [wt.fl_angle, wt.fr_angle, wt.rl_angle, wt.rr_angle]
    speeds = [wt.fl_speed, wt.fr_speed, wt.rl_speed, wt.rr_speed]
    for i in range(4):
        print(f"    {labels[i]}  angle={math.degrees(angles[i]):+7.2f}°  speed={speeds[i]:+7.3f} rad/s")

# ── Test scenarios ────────────────────────────────────────────────────────────

def run_scenario(node: PipelineNode, vx: float, wz: float, label: str):
    print(f"\n{'='*60}")
    print(f"SCENARIO: {label}  (vx={vx:.2f} m/s  wz={wz:.2f} rad/s)")
    print(f"{'='*60}")

    node.publish_twist(vx, wz)
    frames = node.collect_can_frames(0.5)

    with node._lock:
        wt = node.last_wheel_targets
    print_wheel_targets(wt)
    print_can_frames(frames)

# ── Keyboard input ────────────────────────────────────────────────────────────

def get_key() -> str:
    """Read a single key from stdin without echoing."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

HELP = """
╔══════════════════════════════════════════╗
║      chassis_driver pipeline test        ║
╠══════════════════════════════════════════╣
║  e   Enable all motors                   ║
║  d   Disable all motors                  ║
║  ─── scenarios ─────────────────────────║
║  1   Straight forward  0.5 m/s           ║
║  2   Spin in place     1.0 rad/s         ║
║  3   Turn left         0.5 m/s + 0.5ω   ║
║  4   Max steer angle   no drive          ║
║  s   Stop (cmd_vel = 0)                  ║
║  ─── info ──────────────────────────────║
║  f   Print motor feedback                ║
║  q   Quit                                ║
╚══════════════════════════════════════════╝
"""

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = PipelineNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("Waiting 1s for nodes to connect...")
    time.sleep(1.0)
    print(HELP)

    try:
        while True:
            key = get_key()

            if key == 'e':
                node.publish_enable()

            elif key == 'd':
                node.publish_disable()

            elif key == '1':
                run_scenario(node, vx=0.5, wz=0.0, label="STRAIGHT FORWARD 0.5 m/s")

            elif key == '2':
                run_scenario(node, vx=0.0, wz=1.0, label="SPIN IN PLACE 1.0 rad/s")

            elif key == '3':
                run_scenario(node, vx=0.5, wz=0.5, label="TURN LEFT (Ackermann)")

            elif key == '4':
                # Max steer angle with no drive — publishes directly to /to_can_bus
                print("\n[TEST 4] Max steer angle, no drive")
                max_angle = math.radians(45.0)
                for esc_id in STEER_IDS:
                    node.can_pub.publish(sw_abs_position(esc_id, max_angle))
                for esc_id in DRIVE_IDS:
                    node.can_pub.publish(dm_velocity(esc_id, 0.0))
                frames = node.collect_can_frames(0.3)
                print_can_frames(frames)

            elif key == 's':
                print("\n[STOP] Publishing cmd_vel = 0")
                node.publish_twist(0.0, 0.0)

            elif key == 'f':
                with node._lock:
                    status = node.last_chassis_status
                print_chassis_status(status)

            elif key in ('q', '\x03'):   # q or Ctrl+C
                print("\nQuitting...")
                break

            else:
                print(f"  (unknown key '{key}' — press h for help)")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nStopping...")
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
