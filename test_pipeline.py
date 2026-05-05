#!/usr/bin/env python3
"""
Full pipeline test:
  cmd_vel -> rover_kinematics_node -> /wheel_targets
          -> damiao_driver         -> /to_can_bus
          -> cansend (printed or executed on can0)
"""
import math
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from indomitus_msgs.msg import WheelTargets


CAN_IFACE = "can0"

# Motor IDs — must match damiao_driver.yaml
STEER_IDS = [11, 13, 15, 17]   # FL FR RL RR
DRIVE_IDS = [10, 12, 14, 16]   # FL FR RL RR


def cansend_frame(iface: str, can_id: int, data: bytes) -> bool:
    cmd = f"{iface} {can_id:03X}#{data.hex().upper()}"
    ret = subprocess.run(["cansend"] + cmd.split(), capture_output=True, text=True)
    ok = ret.returncode == 0
    print(f"  {'OK' if ok else 'FAIL'}  cansend {cmd}")
    return ok


def hardware_init():
    """setMode then enable — must reach the wire before any motion commands."""
    print("\n--- Hardware init: setMode + enable ---")
    for esc_id in STEER_IDS:
        cansend_frame(CAN_IFACE, 0x7FF,
                      bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF, 0x55, 0x0A, 2, 0, 0, 0]))
    for esc_id in DRIVE_IDS:
        cansend_frame(CAN_IFACE, 0x7FF,
                      bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF, 0x55, 0x0A, 3, 0, 0, 0]))
    time.sleep(0.05)
    for esc_id in STEER_IDS + DRIVE_IDS:
        cansend_frame(CAN_IFACE, esc_id, bytes([0xFF] * 7 + [0xFC]))
    print()


def hardware_stop():
    """
    Correct stop sequence:
      1. Send velocity=0 to all drive motors (coast to stop)
      2. Send position=0 to all steer motors (straighten wheels)
      3. Wait for motion to settle
      4. Disable all motors (cut torque, safe power state)
    After this, call hardware_init() again before sending motion commands.
    """
    print("\n--- Stop: zero commands ---")
    zero_vel = bytes(struct.pack('<f', 0.0))
    zero_pos = struct.pack('<ff', 0.0, 5.0)   # pos=0 rad, vmax=5 rad/s (slow return to centre)

    for esc_id in DRIVE_IDS:
        cansend_frame(CAN_IFACE, 0x200 + esc_id, zero_vel)
    for esc_id in STEER_IDS:
        cansend_frame(CAN_IFACE, 0x100 + esc_id, zero_pos)

    print("\n  Waiting 1.5 s for motion to settle...")
    time.sleep(1.5)

    print("\n--- Disable all motors ---")
    for esc_id in STEER_IDS + DRIVE_IDS:
        cansend_frame(CAN_IFACE, esc_id, bytes([0xFF] * 7 + [0xFD]))
    print()


def decode_can_frame(frame: Frame) -> tuple[str, str]:
    """Returns (cansend_cmd, human_description)."""
    cid = frame.id
    data = bytes(frame.data[:frame.dlc])
    hex_data = data.hex().upper()
    cansend = f"{CAN_IFACE} {cid:03X}#{hex_data}"

    if frame.dlc == 4 and (cid & 0xFF00) == 0x0200:
        vel = struct.unpack('<f', data)[0]
        desc = f"VELOCITY  motor={cid & 0xFF}  vel={vel:+.3f} rad/s"
    elif frame.dlc == 8 and (cid & 0xFF00) == 0x0100:
        pos, vmax = struct.unpack('<ff', data)
        desc = f"POS-VEL   motor={cid & 0xFF}  pos={pos:+.3f} rad  vmax={vmax:.1f} rad/s"
    elif frame.dlc == 8 and data[7] in (0xFC, 0xFD, 0xFE):
        name = {0xFC: "ENABLE", 0xFD: "DISABLE", 0xFE: "SET_ZERO"}[data[7]]
        desc = f"SPECIAL   motor={cid}  cmd={name}"
    elif cid == 0x7FF and frame.dlc == 8 and data[2] == 0x55:
        esc = data[0] | (data[1] << 8)
        reg = data[3]
        val = int.from_bytes(data[4:8], 'little')
        desc = f"REG_WRITE motor={esc}  reg={reg}  val={val}"
    else:
        desc = f"RAW  id=0x{cid:03X}  dlc={frame.dlc}"

    return cansend, desc


class PipelineTestNode(Node):
    def __init__(self):
        super().__init__('pipeline_test')
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_vel_pub  = self.create_publisher(Twist,        '/cmd_vel',      qos)
        self.wt_sub       = self.create_subscription(WheelTargets, '/wheel_targets',
                                                     self.on_wheel_targets, qos)
        self.can_sub      = self.create_subscription(Frame,     '/to_can_bus',
                                                     self.on_can_frame,     qos)

        self.wheel_targets_msg = None
        self.can_frames: list[Frame] = []

    def on_wheel_targets(self, msg: WheelTargets):
        self.wheel_targets_msg = msg

    def on_can_frame(self, msg: Frame):
        self.can_frames.append(msg)


def spin_bg(executor):
    executor.spin()


def send_and_collect(node: PipelineTestNode, executor, vx, vy, wz, label):
    node.wheel_targets_msg = None
    node.can_frames.clear()

    tw = Twist()
    tw.linear.x  = vx
    tw.linear.y  = vy
    tw.angular.z = wz
    node.cmd_vel_pub.publish(tw)

    # Give nodes time to process
    time.sleep(0.4)

    print(f"\n{'='*60}")
    print(f"CMD_VEL: {label}  (vx={vx} m/s  vy={vy}  wz={wz} rad/s)")
    print(f"{'='*60}")

    wt = node.wheel_targets_msg
    if wt:
        print(f"\n  /wheel_targets:")
        print(f"    FL  angle={math.degrees(wt.fl_angle):+7.2f}°  speed={wt.fl_speed:+7.3f} rad/s")
        print(f"    FR  angle={math.degrees(wt.fr_angle):+7.2f}°  speed={wt.fr_speed:+7.3f} rad/s")
        print(f"    RL  angle={math.degrees(wt.rl_angle):+7.2f}°  speed={wt.rl_speed:+7.3f} rad/s")
        print(f"    RR  angle={math.degrees(wt.rr_angle):+7.2f}°  speed={wt.rr_speed:+7.3f} rad/s")
    else:
        print("  /wheel_targets: (not received — is rover_kinematics_node running?)")

    # Only send motion frames (pos-vel + velocity) — enable was already sent by hardware_init
    motion = [f for f in node.can_frames
              if (f.dlc == 4 and (f.id & 0xFF00) == 0x0200) or
                 (f.dlc == 8 and (f.id & 0xFF00) == 0x0100)]
    # Deduplicate — keep only the last frame per motor (freshest command)
    seen = {}
    for f in motion:
        seen[f.id] = f
    motion = list(seen.values())

    print(f"\n  /to_can_bus  ({len(motion)} motion frames):")
    cansend_cmds = []
    for f in sorted(motion, key=lambda f: f.id):
        cmd, desc = decode_can_frame(f)
        print(f"    {desc}")
        print(f"    └─ cansend: {cmd}")
        cansend_cmds.append(cmd)

    # Try to send via cansend if can0 is available
    if cansend_cmds:
        can_up = False
        try:
            result = subprocess.run(
                ["ip", "link", "show", CAN_IFACE],
                capture_output=True, text=True
            )
            can_up = "UP" in result.stdout
        except FileNotFoundError:
            # try /sbin/ip
            try:
                result = subprocess.run(
                    ["/sbin/ip", "link", "show", CAN_IFACE],
                    capture_output=True, text=True
                )
                can_up = "UP" in result.stdout
            except FileNotFoundError:
                pass

        if can_up:
            print(f"\n  Sending {len(cansend_cmds)} frames on {CAN_IFACE}...")
            for cmd in cansend_cmds:
                parts = cmd.split()
                ret = subprocess.run(["cansend"] + parts, capture_output=True, text=True)
                status = "OK" if ret.returncode == 0 else f"FAIL({ret.returncode})"
                print(f"    {status}  {cmd}")
        else:
            print(f"\n  [{CAN_IFACE} not up — cansend commands below]")
            for cmd in cansend_cmds:
                print(f"    cansend {cmd}")


def main():
    rclpy.init()
    node = PipelineTestNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    t = threading.Thread(target=spin_bg, args=(executor,), daemon=True)
    t.start()

    print("Pipeline test — waiting 1s for nodes to connect...")
    time.sleep(1.0)

    hardware_init()

    # Test cases
    send_and_collect(node, executor, vx=1.0,  vy=0.0, wz=0.0,  label="STRAIGHT FORWARD 1 m/s")
    # send_and_collect(node, executor, vx=-1.0, vy=0.0, wz=0.0,  label="STRAIGHT REVERSE 1 m/s")
    # send_and_collect(node, executor, vx=0.0,  vy=0.0, wz=1.0,  label="SPIN LEFT 1 rad/s")
    # send_and_collect(node, executor, vx=1.0,  vy=0.0, wz=0.5,  label="TURN LEFT (Ackermann)")

    time.sleep(3.0)   # run for 3 seconds

    hardware_stop()   # zero → settle → disable

    print("\n\nDone.")
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
