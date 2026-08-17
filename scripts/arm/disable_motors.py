#!/usr/bin/env python3
"""
disable_motors.py — explicitly cut torque on ALL SIX arm motors.

Since ArmCanSystem::on_deactivate/on_shutdown were changed to leave motors
HOLDING (not disabled) on a clean `ros2 launch` Ctrl+C — Steadywin motors
(base/shoulder/elbow) keep executing their last MIT command indefinitely,
with no ros2_control process required — you need an explicit action to
actually release them. This script is that action: it talks to the CAN bus
directly, so it doesn't need ROS running (including after a crash) — but it
refuses to run if it detects CAN traffic already on the bus, since a live
ros2_control_node would just re-enable the Steadywins on its next MIT frame.

Same disable sequence as ArmCanSystem::send_disable_frames():
  1. Damiao (23,24,25): zero-gain MIT probe first, to avoid a torque
     discontinuity, then FF..FD (disable).
  2. Steadywin (20,21,22): 0xCF (disable / free-coast).

!!! THE ARM WILL SAG/FALL under its own weight once disabled (no gravity
compensation) !!! Support it physically — or make sure it's already resting
on something — before running this.

Usage:
  python3 disable_motors.py [--iface can0]
"""

import argparse
import socket
import struct
import time

STEADYWIN_IDS = [20, 21, 22]
DAMIAO_IDS    = [23, 24, 25]

CAN_FRAME_FMT = "=IB3x8s"


def bus_is_busy(sock, window_s=0.3, min_frames=5):
    deadline = time.time() + window_s
    count = 0
    while time.time() < deadline:
        try:
            sock.recv(16)
            count += 1
        except BlockingIOError:
            time.sleep(0.005)
    return count >= min_frames


def send(sock, can_id, data):
    frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), data.ljust(8, b"\x00"))
    for _ in range(5):
        try:
            sock.send(frame)
            return
        except BlockingIOError:
            time.sleep(0.0002)


def pack_mit_zero_gain():
    """pos/vel/tff at zero, kp=kd=0 — genuinely zero torque.

    Every MIT field is a signed range encoded unsigned, so all-zero bytes do
    NOT mean zero: they decode to minus full scale, i.e. -T_MAX on the torque
    field, which with kp=kd=0 is commanded straight through to the motor.
    Zero lives at the midpoint of each field instead.
    """
    p = 0x8000        # 16-bit position, midpoint
    v = 0x800         # 12-bit velocity, midpoint
    kp = 0            # gains are unsigned [0, max]: zero really is zero
    kd = 0
    t = 0x800         # 12-bit torque, midpoint
    return bytes([
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp >> 8) & 0xF),
        kp & 0xFF,
        (kd >> 4) & 0xFF,
        ((kd & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="can0")
    args = ap.parse_args()

    print("!!! Arm WILL sag/fall once disabled — make sure it's supported. !!!")
    ans = input("Type 'DISABLE' to proceed: ").strip()
    if ans != "DISABLE":
        print("Cancelled.")
        return

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.iface,))
    sock.setblocking(False)

    if bus_is_busy(sock):
        print(f"[!] CAN traffic already on {args.iface} — ros2_control / "
              f"ArmCanSystem looks active. Stop the control stack first, "
              f"or a 0xCF here won't stick (the next MIT frame re-enables it).")
        sock.close()
        return

    zero = pack_mit_zero_gain()
    for mid in DAMIAO_IDS:
        send(sock, mid, zero)
    time.sleep(0.01)
    for mid in DAMIAO_IDS:
        send(sock, mid, bytes([0xFF] * 7 + [0xFD]))
        time.sleep(0.002)

    for mid in STEADYWIN_IDS:
        send(sock, 0x100 | mid, bytes([0xCF]))
        time.sleep(0.002)

    sock.close()
    print("[✓] Disable sent to all 6 motors.")


if __name__ == "__main__":
    main()
