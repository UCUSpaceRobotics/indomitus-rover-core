#!/usr/bin/env python3
"""
chassis_driver test — standalone, NO ROS2 required.

Talks directly to the CAN bus via python-can / SocketCAN, instead of going
through rclpy topics and custom message types (WheelTargets, ChassisStatus).

Install dependency:
    pip install python-can --break-system-packages

Make sure the CAN interface is up first, e.g.:
    sudo ip link set can0 up type can bitrate 1000000

Controls:
    e       Enable all motors
    d       Disable all motors  (zero → wait 1.5s → disable)
    1       Test: straight forward 0.5 m/s
    2       Test: spin in place left
    3       Test: turn left (differential)
    4       Test: max steer angle placeholder (no steer motors configured)
    s       Stop drive (publish zero velocity)
    f       Print latest decoded feedback per motor
    q       Quit
"""
import math
import struct
import sys
import termios
import threading
import time
import tty

import can

# ── Config ──────────────────────────────────────────────────────────────────

CAN_CHANNEL = "can0"
CAN_BITRATE = 1_000_000          # must match what's actually configured on the bus

STEER_IDS = []                    # Steadywin, [FL, FR, RL, RR] — none in this build
DRIVE_IDS = [10, 12, 14, 16]      # Damiao,    [FL, FR, RL, RR]
WHEEL_NAMES = ["FL", "FR", "RL", "RR"]

# Skid-steer geometry — ADJUST to your actual rover dimensions
WHEEL_RADIUS = 0.15      # meters
TRACK_WIDTH = 0.6        # meters, distance between left/right wheel centers

# ── Frame builders (same protocol as before) ─────────────────────────────────

def _rad_to_counts(angle_rad: float) -> int:
    return int(angle_rad * 16384 / (2 * math.pi))

# Steadywin
def sw_clear_fault(esc_id: int) -> can.Message:
    return can.Message(arbitration_id=esc_id, data=bytes([0xAF]), is_extended_id=False)

def sw_abs_position(esc_id: int, angle_rad: float) -> can.Message:
    counts = _rad_to_counts(angle_rad)
    return can.Message(arbitration_id=esc_id,
                        data=bytes([0xC2]) + struct.pack('<i', counts),
                        is_extended_id=False)

def sw_disable(esc_id: int) -> can.Message:
    return can.Message(arbitration_id=esc_id, data=bytes([0xCF]), is_extended_id=False)

def sw_status_query(esc_id: int) -> can.Message:
    return can.Message(arbitration_id=esc_id, data=bytes([0xAE]), is_extended_id=False)

# Damiao
def dm_set_mode(esc_id: int, mode: int) -> can.Message:
    data = bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF, 0x55, 0x0A, mode, 0, 0, 0])
    return can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False)

def dm_enable(esc_id: int) -> can.Message:
    return can.Message(arbitration_id=esc_id, data=bytes([0xFF] * 7 + [0xFC]), is_extended_id=False)

def dm_disable(esc_id: int) -> can.Message:
    return can.Message(arbitration_id=esc_id, data=bytes([0xFF] * 7 + [0xFD]), is_extended_id=False)

def dm_velocity(esc_id: int, vel_rad_s: float) -> can.Message:
    return can.Message(arbitration_id=0x200 + esc_id,
                        data=struct.pack('<f', vel_rad_s),
                        is_extended_id=False)

# ── Feedback decode (Damiao feedback frame, per motor manual) ────────────────
# D0 = ID | ERR<<4 , D1-2 = POS(16b), D3-4 = VEL(12b)|T[11:8], D5 = T[7:0],
# D6 = T_MOS, D7 = T_Rotor
# NOTE: converting raw POS/VEL/T fields to physical units requires reading
# PMAX/VMAX/TMAX from the motor's registers (21/22/23). Raw values shown here.

def decode_feedback(msg: can.Message) -> dict | None:
    d = msg.data
    if len(d) != 8:
        return None
    ctrl_id = d[0] & 0x0F
    err = (d[0] >> 4) & 0x0F
    pos_raw = (d[1] << 8) | d[2]
    vel_raw = (d[3] << 4) | (d[4] >> 4)
    t_raw = ((d[4] & 0x0F) << 8) | d[5]
    t_mos = d[6]
    t_rotor = d[7]
    return {
        "can_id": msg.arbitration_id,
        "ctrl_id": ctrl_id,
        "err": err,
        "pos_raw": pos_raw,
        "vel_raw": vel_raw,
        "torque_raw": t_raw,
        "t_mos_c": t_mos,
        "t_rotor_c": t_rotor,
    }

ERR_NAMES = {
    0: "DISABLED", 1: "ENABLED", 8: "OVERVOLTAGE", 9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "COIL_OVERTEMP",
    0xD: "COMM_LOST", 0xE: "OVERLOAD",
}

# ── Bus wrapper with background listener ─────────────────────────────────────

class CanBus:
    def __init__(self, channel: str, bitrate: int):
        self.bus = can.interface.Bus(channel=channel, bustype="socketcan", bitrate=bitrate)
        self.latest_feedback: dict[int, dict] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.2)
            if msg is None:
                continue
            fb = decode_feedback(msg)
            if fb:
                with self._lock:
                    self.latest_feedback[msg.arbitration_id] = fb

    def send(self, msg: can.Message):
        self.bus.send(msg)

    def snapshot(self) -> dict[int, dict]:
        with self._lock:
            return dict(self.latest_feedback)

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()

# ── High-level actions ────────────────────────────────────────────────────────

def enable_all(bus: CanBus):
    print("\n[ENABLE] Sending enable sequence...")
    for esc_id in STEER_IDS:
        bus.send(sw_clear_fault(esc_id)); time.sleep(0.02)
    for esc_id in STEER_IDS:
        bus.send(sw_abs_position(esc_id, 0.0)); time.sleep(0.02)
    for esc_id in DRIVE_IDS:
        bus.send(dm_set_mode(esc_id, 3)); time.sleep(0.02)   # mode 3 = Velocity
    time.sleep(0.05)
    for esc_id in DRIVE_IDS:
        bus.send(dm_enable(esc_id)); time.sleep(0.02)
    print("[ENABLE] Done")

def disable_all(bus: CanBus):
    print("\n[DISABLE] Zeroing commands...")
    for esc_id in STEER_IDS:
        bus.send(sw_abs_position(esc_id, 0.0))
    for esc_id in DRIVE_IDS:
        bus.send(dm_velocity(esc_id, 0.0))
    print("[DISABLE] Waiting 1.5s to settle...")
    time.sleep(1.5)
    for esc_id in STEER_IDS:
        bus.send(sw_disable(esc_id))
    for esc_id in DRIVE_IDS:
        bus.send(dm_disable(esc_id))
    print("[DISABLE] All motors disabled")

def send_diff_drive(bus: CanBus, vx: float, wz: float):
    """Simple skid-steer mixing: left/right wheel speeds from vx, wz."""
    left_speed = (vx - wz * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    right_speed = (vx + wz * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    # DRIVE_IDS order = [FL, FR, RL, RR]
    speeds = [left_speed, right_speed, left_speed, right_speed]
    for esc_id, spd in zip(DRIVE_IDS, speeds):
        bus.send(dm_velocity(esc_id, spd))
    return speeds

# ── Display helpers ────────────────────────────────────────────────────────────

def print_feedback(bus: CanBus):
    snap = bus.snapshot()
    if not snap:
        print("  (no feedback frames received yet)")
        return
    print(f"\n  Latest feedback  [{len(snap)} motor(s)]")
    for can_id, fb in sorted(snap.items()):
        err_name = ERR_NAMES.get(fb["err"], f"0x{fb['err']:X}")
        print(f"    can_id=0x{can_id:03X} ctrl_id={fb['ctrl_id']:2d} "
              f"status={err_name:12s} pos_raw={fb['pos_raw']:5d} "
              f"vel_raw={fb['vel_raw']:4d} torque_raw={fb['torque_raw']:4d} "
              f"T_mos={fb['t_mos_c']}C T_rotor={fb['t_rotor_c']}C")

# ── Keyboard input ────────────────────────────────────────────────────────────

def get_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

HELP = """
╔══════════════════════════════════════════╗
║   chassis_driver standalone test (no ROS) ║
╠══════════════════════════════════════════╣
║  e   Enable all motors                   ║
║  d   Disable all motors                  ║
║  ─── scenarios ─────────────────────────║
║  1   Straight forward  0.5 m/s           ║
║  2   Spin in place     1.0 rad/s         ║
║  3   Turn left         0.5 m/s + 0.5ω   ║
║  4   (steer test placeholder — no steer  ║
║       motors configured)                 ║
║  s   Stop (velocity = 0)                 ║
║  ─── info ──────────────────────────────║
║  f   Print motor feedback                ║
║  q   Quit                                ║
╚══════════════════════════════════════════╝
"""

def main():
    print(f"Opening {CAN_CHANNEL} @ {CAN_BITRATE} bps...")
    bus = CanBus(CAN_CHANNEL, CAN_BITRATE)
    print(HELP)

    try:
        while True:
            key = get_key()

            if key == 'e':
                enable_all(bus)

            elif key == 'd':
                disable_all(bus)

            elif key == '1':
                print("\nSCENARIO: STRAIGHT FORWARD 0.5 m/s")
                speeds = send_diff_drive(bus, vx=0.5, wz=0.0)
                print(f"  wheel speeds (rad/s): {speeds}")

            elif key == '2':
                print("\nSCENARIO: SPIN IN PLACE 1.0 rad/s")
                speeds = send_diff_drive(bus, vx=0.0, wz=1.0)
                print(f"  wheel speeds (rad/s): {speeds}")

            elif key == '3':
                print("\nSCENARIO: TURN LEFT")
                speeds = send_diff_drive(bus, vx=0.5, wz=0.5)
                print(f"  wheel speeds (rad/s): {speeds}")

            elif key == '4':
                print("\n[TEST 4] No steer motors configured — skipping.")

            elif key == 's':
                print("\n[STOP] Zero velocity")
                send_diff_drive(bus, vx=0.0, wz=0.0)

            elif key == 'f':
                print_feedback(bus)

            elif key in ('q', '\x03'):
                print("\nQuitting...")
                break

            else:
                print(f"  (unknown key '{key}')")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nShutting down bus...")
        bus.shutdown()


if __name__ == '__main__':
    main()
