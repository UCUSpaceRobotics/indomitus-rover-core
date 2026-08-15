#!/usr/bin/env python3
"""
check_calibration.py — TORQUELESS calibration & verification tool.

The motors NEVER receive any stiffness from this tool:
  * Steadywin (20,21,22): polled with command 0xF1, which reads position
    without even entering MIT mode.
  * Damiao (23,24,25): enabled (0xFC) and polled with kp=0, kd=0, tff=0 MIT
    frames — zero torque. They are cleanly disabled (0xFD) on exit.

You can freely move the arm BY HAND while this runs.

Modes:
  live      (default)  Print raw motor angles + URDF-transformed angles at 10 Hz.
                       Move each joint by hand and watch the numbers.
  ros                  Same, but also publish sensor_msgs/JointState on
                       /joint_states so RViz mirrors the hand-moved arm.
                       Run alongside:  ros2 launch arm_bringup
                       arm_standalone.launch.py gui_only:=false use_fake_hardware:=true
                       is WRONG for this — instead run only robot_state_publisher
                       (or the launch with everything except a js source) and RViz.
                       Easiest: ros2 run robot_state_publisher robot_state_publisher
                       --ros-args -p robot_description:="$(xacro .../arm_standalone.urdf.xacro)"
  zero      DANGER-ish Save the CURRENT pose as motor zero on ALL motors
                       (Steadywin 0xB1 / Damiao FF..FE, stored in ROM).
                       Do this ONCE, with the arm held exactly at the URDF
                       zero pose. Asks for confirmation.
  snapshot             Print one line of raw angles formatted as URDF offsets,
                       for pasting into arm_macro.xacro (use when the arm is
                       held at the URDF zero pose but you don't want to
                       overwrite the motors' saved zeros).

Calibration values currently applied in 'live'/'ros' come from the DIRECTIONS
and OFFSETS dicts below — edit them to test before moving them into the xacro.

Usage:
  python3 check_calibration.py                # live text mode
  python3 check_calibration.py --mode ros
  python3 check_calibration.py --mode snapshot
  python3 check_calibration.py --mode zero
"""

import argparse
import math
import select
import signal
import socket
import struct
import sys
import time

# ============================ CONFIG ========================================
CAN_IFACE     = "can0"
STEADYWIN_IDS = [20, 21, 22]
DAMIAO_IDS    = [23, 24, 25]
ALL_IDS       = STEADYWIN_IDS + DAMIAO_IDS

# The wrist Damiao motors were re-flashed: each answers on its OWN Master ID
# (0x400 | CAN-ID) instead of the factory-default shared Master ID 0.
# e.g. motor 23 (0x17) -> replies on 0x417.  {master_id: motor_id}
DM_MASTER_IDS = {0x400 | mid: mid for mid in DAMIAO_IDS}

JOINT_NAMES = {
    20: "arm_mount_base_joint",
    21: "arm_base_shoulder_joint",
    22: "arm_shoulder_forearm_joint",
    23: "arm_forearm_wrist_1_joint",
    24: "arm_wrist_1_wrist_2_joint",
    25: "arm_wrist_2_end_effector_joint",
}

# EDIT ME while calibrating, then copy the final values into arm_macro.xacro.
DIRECTIONS = {20: -1.0, 21: -1.0, 22: 1.0, 23: -1.0, 24: 1.0, 25: 1.0}
OFFSETS    = {20: 0.0, 21: 0.0, 22: 0.0, 23: 0.0, 24: 0.0, 25: 0.0}  # rad, motor frame

SW_POS_MAX_RAD = 12.5
SW_VEL_MAX_RPS = 45.0
SW_T_MAX_NM    = 48.0   # must match steadywin_protocol.hpp
DM_P_MAX_RAD   = 12.5
DM_V_MAX_RPS   = 45.0
DM_T_MAX_NM    = 18.0

POLL_HZ  = 20
PRINT_HZ = 10
CAN_FRAME_FMT = "=IB3x8s"
# ============================================================================

DM_ERR = {0x0: "disabled", 0x1: "enabled", 0x8: "OVERVOLT", 0x9: "UNDERVOLT",
          0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "COIL_OVERTEMP",
          0xD: "COMM_LOSS", 0xE: "OVERLOAD"}


def float_to_uint(x, lo, hi, bits):
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def uint_to_float(u, lo, hi, bits):
    return (u * (hi - lo) / ((1 << bits) - 1.0)) + lo


def pack_mit_zero_gain():
    p = float_to_uint(0.0, -DM_P_MAX_RAD, DM_P_MAX_RAD, 16)
    v = float_to_uint(0.0, -DM_V_MAX_RPS, DM_V_MAX_RPS, 12)
    t = float_to_uint(0.0, -DM_T_MAX_NM, DM_T_MAX_NM, 12)
    return bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF, ((v & 0xF) << 4) | 0,
        0,
        0, ((t >> 8) & 0xF),
        t & 0xFF,
    ])


class Bus:
    def __init__(self, iface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)

    def send(self, can_id, data):
        frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b"\x00"))
        for _ in range(5):
            try:
                self.sock.send(frame)
                return
            except BlockingIOError:
                time.sleep(0.0002)

    def is_busy(self, window_s=0.3, min_frames=5):
        deadline = time.monotonic() + window_s
        count = 0
        while time.monotonic() < deadline:
            try:
                self.sock.recv(16)
                count += 1
            except BlockingIOError:
                time.sleep(0.005)
        return count >= min_frames

    def recv_all(self, handler):
        while True:
            r, _, _ = select.select([self.sock], [], [], 0)
            if not r:
                return
            try:
                frame, _ = self.sock.recvfrom(16)
            except BlockingIOError:
                return
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            handler(can_id & socket.CAN_EFF_MASK, dlc, data)

    def close(self):
        self.sock.close()


class Reader:
    def __init__(self, bus):
        self.bus = bus
        self.raw = {mid: None for mid in ALL_IDS}   # motor-frame rad
        self.dm_err = {mid: None for mid in DAMIAO_IDS}
        self.zero_gain = pack_mit_zero_gain()

    def poll_once(self):
        for mid in STEADYWIN_IDS:
            self.bus.send(0x100 | mid, bytes([0xF1]))            # read, no mode change
        for mid in DAMIAO_IDS:
            self.bus.send(mid, self.zero_gain)                   # zero torque probe
        self.bus.recv_all(self._on_frame)

    def _on_frame(self, can_id, dlc, data):
        if can_id in STEADYWIN_IDS and dlc >= 7:
            # Only 0xF1/0xCF-style replies carry position in bytes [1..2].
            if data[0] not in (0xF1, 0xCF):
                return
            u = (data[1] << 8) | data[2]
            self.raw[can_id] = uint_to_float(u, -SW_POS_MAX_RAD, SW_POS_MAX_RAD, 16)
        elif can_id in DM_MASTER_IDS and dlc >= 6:
            # Re-flashed wrists: each Damiao answers on its own Master ID
            # (0x400 | CAN-ID), so the sender is known from the frame ID alone.
            mid = DM_MASTER_IDS[can_id]
            u = (data[1] << 8) | data[2]
            self.raw[mid] = uint_to_float(u, -DM_P_MAX_RAD, DM_P_MAX_RAD, 16)
            self.dm_err[mid] = (data[0] >> 4) & 0x0F

    def urdf(self, mid):
        if self.raw[mid] is None:
            return None
        return (self.raw[mid] - OFFSETS[mid]) * DIRECTIONS[mid]


def enable_damiao(bus):
    for mid in DAMIAO_IDS:
        bus.send(mid, bytes([0xFF] * 7 + [0xFC]))
        time.sleep(0.005)


def disable_damiao(bus):
    zero = pack_mit_zero_gain()
    for mid in DAMIAO_IDS:
        bus.send(mid, zero)
    time.sleep(0.01)
    for mid in DAMIAO_IDS:
        bus.send(mid, bytes([0xFF] * 7 + [0xFD]))


def mode_live(bus, reader, publish_ros=False):
    node = pub = None
    if publish_ros:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = Node("calibration_mirror")
        pub = node.create_publisher(JointState, "joint_states", 10)
        JointState_cls = JointState
        print("[i] Publishing /joint_states — open RViz with robot_state_publisher "
              "running and move the arm BY HAND. The model must mirror reality.")

    print("[i] Torqueless live view. Move joints by hand. Ctrl+C to exit.")
    print("[i] direction is CORRECT when pushing a joint in its URDF-positive "
          "direction makes the 'urdf' number increase.")
    dt = 1.0 / POLL_HZ
    last_print = 0.0
    try:
        while True:
            reader.poll_once()
            now = time.monotonic()
            if now - last_print >= 1.0 / PRINT_HZ:
                last_print = now
                cols = []
                for mid in ALL_IDS:
                    raw = reader.raw[mid]
                    u = reader.urdf(mid)
                    if raw is None:
                        cols.append(f"m{mid}:  ---  ")
                    else:
                        cols.append(f"m{mid}: raw={math.degrees(raw):+8.2f}°  urdf={math.degrees(u):+8.2f}°")
                errs = [f"m{m}:{DM_ERR.get(reader.dm_err[m], '?')}" for m in DAMIAO_IDS
                        if reader.dm_err[m] is not None and reader.dm_err[m] >= 0x8]
                line = "  |  ".join(cols) + ("   ERR " + ",".join(errs) if errs else "")
                sys.stdout.write("\r" + line + "   ")
                sys.stdout.flush()

                if pub is not None:
                    msg = JointState_cls()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    for mid in ALL_IDS:
                        u = reader.urdf(mid)
                        if u is not None:
                            msg.name.append(JOINT_NAMES[mid])
                            msg.position.append(u)
                    pub.publish(msg)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            import rclpy
            node.destroy_node()
            rclpy.shutdown()


def mode_snapshot(bus, reader):
    print("[i] Hold the arm EXACTLY at the URDF zero pose. Reading for 2 s...")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        reader.poll_once()
        time.sleep(1.0 / POLL_HZ)
    missing = [mid for mid in ALL_IDS if reader.raw[mid] is None]
    if missing:
        print(f"[!] No reply from motors {missing} — check wiring/power. Aborting.")
        return
    print("\n# offsets (rad, motor frame) for arm_macro.xacro — offset = raw angle at URDF zero pose:")
    for mid in ALL_IDS:
        print(f'#   {JOINT_NAMES[mid]:38s} motor {mid}: offset="{reader.raw[mid]:+.4f}"')
    print("\n# NOTE: measure DIRECTIONS first (live mode); if the arm was instead at a")
    print("# non-zero reference pose q_ref, use offset = raw - direction * q_ref.")


def mode_zero(bus, reader):
    print("=" * 70)
    print("This saves the CURRENT physical pose as the zero of ALL SIX motors,")
    print("permanently (ROM). Only do this while holding the arm EXACTLY at the")
    print("URDF zero pose. All offsets in the xacro then become 0.0.")
    print("=" * 70)
    ans = input("Type 'ZERO' to proceed: ").strip()
    if ans != "ZERO":
        print("Cancelled.")
        return
    for mid in STEADYWIN_IDS:
        bus.send(0x100 | mid, bytes([0xB1]))
        time.sleep(0.05)   # manual: reply can take ~35 ms with 2nd encoder
    for mid in DAMIAO_IDS:
        bus.send(mid, bytes([0xFF] * 7 + [0xFE]))
        time.sleep(0.05)
    time.sleep(1.0)
    print("[✓] Zero saved on all motors. Set all offsets to 0.0 in arm_macro.xacro.")
    print("[i] Verify now with:  python3 check_calibration.py   (raw should read ~0°)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=CAN_IFACE)
    ap.add_argument("--mode", choices=["live", "ros", "snapshot", "zero"], default="live")
    args = ap.parse_args()

    bus = Bus(args.iface)
    if bus.is_busy():
        print(f"[!] CAN traffic already on {args.iface} — ros2_control / "
              f"ArmCanSystem looks active. Stop the control stack first: "
              f"this tool sends its own enable/zero-gain frames and would "
              f"race it (and --mode zero writes to ROM).")
        bus.close()
        return
    reader = Reader(bus)

    def cleanup(*_):
        try:
            disable_damiao(bus)
        finally:
            bus.close()
        print("\n[✓] Damiao motors disabled. Bye.")
        sys.exit(0)

    signal.signal(signal.SIGTERM, cleanup)

    try:
        enable_damiao(bus)   # needed so they answer probes; zero torque throughout
        if args.mode in ("live", "ros"):
            mode_live(bus, reader, publish_ros=(args.mode == "ros"))
        elif args.mode == "snapshot":
            mode_snapshot(bus, reader)
        elif args.mode == "zero":
            mode_zero(bus, reader)
    finally:
        try:
            disable_damiao(bus)
        finally:
            bus.close()
        print("\n[✓] Damiao motors disabled. Bye.")


if __name__ == "__main__":
    main()
