#!/usr/bin/env python3
"""
sim_motors.py — virtual Steadywin (20,21,22) + Damiao (23,24,25) motors on a
(v)CAN bus, for testing the whole ROS 2 stack WITHOUT a physical arm.

Speaks the same protocol subset the hardware interface uses:

  Steadywin (per 自定义CAN通信协议 Rev.3.06b0):
    RX 0x100|id, data[0]=0xF0 (DLC 7) -> config Pos/Vel/T max; reply echo
    RX 0x100|id, data[0]=0xF1 (DLC 1) -> reply state frame (no mode change)
    RX 0x100|id, data[0]=0xCF         -> disable (free); reply state
    RX 0x100|id, data[0]=0xB1         -> set zero; reply echo
    RX 0x400|0x100|id (MIT 8 bytes)   -> enter MIT mode, execute; reply state
    Reply: StdID=id, DLC 7: [0]=cmd echo, [1..2]=pos16, [3..4hi]=vel12,
           [4lo..5]=t12, [6]=status(bit0=MIT mode)

  Damiao DM-J4340-2EC:
    RX id, FF*7+FC -> enable   (reply feedback)
    RX id, FF*7+FD -> disable  (reply feedback)
    RX id, FF*7+FE -> set zero (reply feedback)
    RX id, FF*7+FB -> clear error
    RX id, MIT 8 bytes -> execute if enabled; reply feedback
    Reply: StdID=0x400|id (re-flashed Master ID), DLC 8: [0]=id_nibble|err<<4, [1..2]=pos16,
           [3]=vel[11:4], [4]=vel[3:0]|t[11:8], [5]=t[7:0], [6]=Tmos, [7]=Trotor

Simulated physics: first-order slew toward the commanded position whenever
kp > 0.5 (rate scales mildly with kp, capped). kp≈0 == torqueless: position
holds (or drifts if you enable --gravity, which sags joints 1 and 2 slowly —
useful to see the ramp actually "catch" the arm).

Usage:
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set vcan0 up
  python3 sim_motors.py --iface vcan0 [--pose 0.3,-0.5,1.0,0.2,-0.4,0.8] [--gravity]

Then point the hardware interface / calibration scripts at vcan0.
"""

import argparse
import math
import select
import socket
import struct
import sys
import time

STEADYWIN_IDS = [20, 21, 22]
DAMIAO_IDS    = [23, 24, 25]
# Damiao feedback frame ID. The real wrist motors were re-flashed away from the
# debug-assistant default (shared Master ID 0) to a per-motor 0x400 | CAN-ID,
# so the simulator mirrors that — otherwise the hardware interface's RX filter
# would never match the simulated replies.
MASTER_ID_BASE = 0x400

SW_P, SW_V, SW_T = 95.5, 45.0, 18.0
DM_P, DM_V, DM_T = 12.5, 45.0, 18.0
KP_MAX, KD_MAX   = 500.0, 5.0

CAN_FMT = "=IB3x8s"


def f2u(x, lo, hi, bits):
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def u2f(u, lo, hi, bits):
    return (u * (hi - lo) / ((1 << bits) - 1.0)) + lo


def unpack_mit(data, p_max, v_max, t_max):
    p = (data[0] << 8) | data[1]
    v = (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0xF) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    tf = ((data[6] & 0xF) << 8) | data[7]
    return (u2f(p, -p_max, p_max, 16), u2f(v, -v_max, v_max, 12),
            u2f(kp, 0, KP_MAX, 12), u2f(kd, 0, KD_MAX, 12),
            u2f(tf, -t_max, t_max, 12))


class Motor:
    def __init__(self, mid, proto, pos0):
        self.id = mid
        self.proto = proto                    # "sw" | "dm"
        self.pos = pos0                       # motor-frame rad
        self.vel = 0.0
        self.target = pos0
        self.kp = 0.0
        self.enabled = False                  # dm: enable flag; sw: MIT mode flag
        self.last_t = time.monotonic()

    def step(self, gravity):
        now = time.monotonic()
        dt = min(now - self.last_t, 0.1)
        self.last_t = now
        if self.enabled and self.kp > 0.5:
            rate = min(3.0, 0.3 + self.kp * 0.02)          # rad/s, kp-flavored
            err = self.target - self.pos
            step = max(-rate * dt, min(rate * dt, err))
            self.pos += step
            self.vel = step / dt if dt > 0 else 0.0
        else:
            self.vel = 0.0
            if gravity and self.id in (21, 22):            # sag when torqueless
                self.pos -= 0.05 * dt


class Sim:
    def __init__(self, iface, pose, gravity, skip=()):
        self.gravity = gravity
        self.motors = {}
        for i, mid in enumerate(STEADYWIN_IDS):
            if mid not in skip:
                self.motors[mid] = Motor(mid, "sw", pose[i])
        for i, mid in enumerate(DAMIAO_IDS):
            if mid not in skip:
                self.motors[mid] = Motor(mid, "dm", pose[3 + i])

        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)
        self.iface = iface

    def send(self, can_id, data):
        frame = struct.pack(CAN_FMT, can_id, len(data), data.ljust(8, b"\x00"))
        try:
            self.sock.send(frame)
        except (BlockingIOError, OSError):
            pass

    # ---------------- Steadywin ----------------

    def sw_state_reply(self, m, echo):
        p = f2u(m.pos, -SW_P, SW_P, 16)
        v = f2u(m.vel, -SW_V, SW_V, 12)
        t = f2u(0.0, -SW_T, SW_T, 12)
        status = 0x01 if m.enabled else 0x00
        data = bytes([
            echo,
            (p >> 8) & 0xFF, p & 0xFF,
            (v >> 4) & 0xFF, ((v & 0xF) << 4) | ((t >> 8) & 0xF),
            t & 0xFF,
            status,
        ])
        self.send(m.id, data)

    def handle_sw_mgmt(self, m, dlc, data):
        cmd = data[0]
        if cmd == 0xF1:
            m.step(self.gravity)
            self.sw_state_reply(m, 0xF1)
        elif cmd == 0xF0:
            if dlc >= 7:
                pass  # accept config silently; scaling constants are fixed here
            # reply echoes configured maxima — the frame the RX filter must SKIP
            pos = f2u_le(SW_P * 10)
            vel = f2u_le(SW_V * 100)
            tmx = f2u_le(SW_T * 100)
            self.send(m.id, bytes([0xF0]) + pos + vel + tmx)
        elif cmd == 0xCF:
            m.enabled = False
            m.kp = 0.0
            self.sw_state_reply(m, 0xCF)
        elif cmd == 0xB1:
            m.pos = 0.0
            m.target = 0.0
            self.send(m.id, bytes([0xB1, 0x00, 0x00]))

    def handle_sw_mit(self, m, data):
        pos, vel, kp, kd, tff = unpack_mit(data, SW_P, SW_V, SW_T)
        m.enabled = True             # MIT frame switches the motor into MIT mode
        m.target, m.kp = pos, kp
        m.step(self.gravity)
        self.sw_state_reply(m, 0xF1)

    # ---------------- Damiao ----------------

    def dm_feedback(self, m):
        err = 0x1 if m.enabled else 0x0
        p = f2u(m.pos, -DM_P, DM_P, 16)
        v = f2u(m.vel, -DM_V, DM_V, 12)
        t = f2u(0.0, -DM_T, DM_T, 12)
        data = bytes([
            (m.id & 0x0F) | (err << 4),
            (p >> 8) & 0xFF, p & 0xFF,
            (v >> 4) & 0xFF, ((v & 0xF) << 4) | ((t >> 8) & 0xF),
            t & 0xFF,
            35, 38,                                   # plausible temps °C
        ])
        self.send(MASTER_ID_BASE | m.id, data)

    def handle_dm(self, m, data):
        if data[:7] == b"\xff" * 7:
            tail = data[7]
            if tail == 0xFC:
                m.enabled = True
            elif tail == 0xFD:
                m.enabled = False
                m.kp = 0.0
            elif tail == 0xFE:
                m.pos = 0.0
                m.target = 0.0
            elif tail == 0xFB:
                pass
            self.dm_feedback(m)
            return
        # MIT command
        pos, vel, kp, kd, tff = unpack_mit(data, DM_P, DM_V, DM_T)
        if m.enabled:
            m.target, m.kp = pos, kp
        m.step(self.gravity)
        self.dm_feedback(m)          # Damiao replies even when disabled (err=0)

    # ---------------- main loop ----------------

    def run(self):
        print(f"[sim] simulating motors {sorted(self.motors)} on {self.iface}. Ctrl+C to stop.")
        last_print = 0.0
        while True:
            r, _, _ = select.select([self.sock], [], [], 0.05)
            if r:
                try:
                    frame, _ = self.sock.recvfrom(16)
                except BlockingIOError:
                    continue
                can_id, dlc, data = struct.unpack(CAN_FMT, frame)
                can_id &= socket.CAN_EFF_MASK
                self.dispatch(can_id, dlc, data)

            now = time.monotonic()
            for m in self.motors.values():
                m.step(self.gravity)
            if now - last_print > 0.5:
                last_print = now
                cells = []
                for mid in sorted(self.motors):
                    m = self.motors[mid]
                    flag = "E" if m.enabled else "-"
                    cells.append(f"m{mid}[{flag}] {math.degrees(m.pos):+7.2f}°")
                sys.stdout.write("\r" + "  ".join(cells) + "   ")
                sys.stdout.flush()

    def dispatch(self, can_id, dlc, data):
        # Steadywin management frames: 0x100|id  (only simulated motors)
        for mid in [m for m in STEADYWIN_IDS if m in self.motors]:
            if can_id == (0x100 | mid) and dlc >= 1:
                self.handle_sw_mgmt(self.motors[mid], dlc, data)
                return
            if can_id == (0x400 | 0x100 | mid) and dlc == 8:
                self.handle_sw_mit(self.motors[mid], data)
                return
            if can_id == (0x400 | mid) and dlc == 8:
                self.handle_sw_mit(self.motors[mid], data)
                return
        for mid in [m for m in DAMIAO_IDS if m in self.motors]:
            if can_id == mid and dlc == 8:
                self.handle_dm(self.motors[mid], data)
                return


def f2u_le(v):
    v = int(round(v))
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--pose", default="0.3,-0.5,1.0,0.2,-0.4,0.8",
                    help="initial motor-frame positions (rad), comma-separated. "
                         "Deliberately non-zero by default: proves the startup "
                         "sync reads reality instead of assuming 0.")
    ap.add_argument("--skip", default="",
                    help="comma-separated motor IDs NOT to simulate because they "
                         "are real and on the same bus, e.g. --skip 20")
    ap.add_argument("--gravity", action="store_true",
                    help="joints 21/22 sag slowly while torqueless")
    ap.add_argument("--ids", default=None,
                    help="comma-separated motor IDs to emulate, e.g. --ids 24,25 "
                         "(default: all six). Use when some motors are REAL "
                         "and on the same bus!")
    args = ap.parse_args()
    active = None

    pose = [float(x) for x in args.pose.split(",")]
    if len(pose) != 6:
        print("--pose needs 6 values")
        sys.exit(1)

    skip = {int(x) for x in args.skip.split(",") if x.strip()}
    if args.ids:
        active = {int(x) for x in args.ids.split(",")}
        skip |= set(STEADYWIN_IDS + DAMIAO_IDS) - active   # ids = все інше у skip
    try:
        Sim(args.iface, pose, args.gravity, skip).run()
    except KeyboardInterrupt:
        print("\n[sim] bye")
    except OSError as e:
        print(f"\n[sim] cannot open {args.iface}: {e}\n"
              f"      sudo modprobe vcan && sudo ip link add dev vcan0 type vcan "
              f"&& sudo ip link set vcan0 up")


if __name__ == "__main__":
    main()