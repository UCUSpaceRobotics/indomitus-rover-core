#!/usr/bin/env python3
"""
chassis_driver test — standalone, NO ROS2 required.

Talks directly to the CAN bus via python-can / SocketCAN, instead of going
through rclpy topics and custom message types (WheelTargets, ChassisStatus).

Install dependency:
    pip install python-can --break-system-packages

Make sure the CAN interface is up first, e.g.:
    sudo ip link set can0 up type can bitrate 1000000

Motor selection (choose which motors get enabled/commanded):
    --all                 test all 8 motors
    --damiao [ID ...]     test damiao motors. No IDs = all 4. E.g. --damiao 1 3
    --steadywin [ID ...]  test steadywin motors. No IDs = all 4. E.g. --steadywin 2
    (IDs are wheel indices 0-3, matching FL, FR, RL, RR)

Controls (vim-style):
    e       Enable selected motors
    d       Disable selected motors (zero → wait → disable)
    k       Increase drive speed (all damiao move together)
    j       Decrease drive speed
    l       Increase steer angle (all steadywin move together)
    h       Decrease steer angle
    s       Stop drive (speed target -> 0, ramps down; angle unchanged)
    f       Print latest decoded feedback + current speed/angle
    q       Quit
"""
import argparse
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

STEER_IDS = [11, 13, 15, 17]      # Steadywin, [FL, FR, RL, RR]
DRIVE_IDS = [10, 12, 14, 16]      # Damiao,    [FL, FR, RL, RR]
WHEEL_NAMES = ["FL", "FR", "RL", "RR"]

# Ramp control for drive speed — ADJUST to taste
MAX_ACCEL = 2.0          # rad/s^2, applied when |target| > |current| (speeding up)
MAX_DECEL = 3.0          # rad/s^2, applied when |target| < |current| (slowing down)
CMD_RATE_HZ = 100.0      # command loop frequency — sends a frame to every enabled
                          # motor every cycle, whether or not the target changed.
                          # This is also what keeps each motor's CAN
                          # communication-loss watchdog (TIMEOUT register) happy.

# Step sizes for keyboard control
SPEED_STEP = 0.1          # rad/s added/removed per 'k'/'j' keypress
ANGLE_STEP_DEG = 5.0      # degrees added/removed per 'l'/'h' keypress

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
        self.bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=bitrate)
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

# ── Motion controller ──────────────────────────────────────────────────────
#
# Single shared drive speed for all active damiao motors (ramped smoothly),
# and a single shared steer angle for all active steadywin motors (sent
# directly — steering position doesn't need accel/decel limiting the way
# velocity does). Both are re-sent every cycle at CMD_RATE_HZ so each
# motor's CAN watchdog stays happy even while holding still.

class MotionController:
    def __init__(self, bus: CanBus, drive_ids: list[int], steer_ids: list[int]):
        self.bus = bus
        self.drive_ids = drive_ids
        self.steer_ids = steer_ids

        self.speed_current = 0.0
        self.speed_target = 0.0
        self.angle = 0.0   # radians, sent directly (no ramping)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._active = threading.Event()   # only send frames while enabled
        self._sent_count = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_speed_target(self, speed: float):
        with self._lock:
            self.speed_target = speed

    def set_angle(self, angle_rad: float):
        with self._lock:
            self.angle = angle_rad

    def set_active(self, on: bool):
        if on:
            self._active.set()
        else:
            self._active.clear()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "speed_current": self.speed_current,
                "speed_target": self.speed_target,
                "angle": self.angle,
            }

    def sent_count(self) -> int:
        return self._sent_count

    def wait_until_settled(self, tol: float = 0.02, timeout: float = 5.0):
        """Block until drive speed is within `tol` rad/s of its target."""
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                done = abs(self.speed_current - self.speed_target) < tol
            if done:
                return
            time.sleep(1.0 / CMD_RATE_HZ)

    def _run(self):
        dt = 1.0 / CMD_RATE_HZ
        next_tick = time.monotonic()
        while not self._stop.is_set():
            if self._active.is_set():
                with self._lock:
                    diff = self.speed_target - self.speed_current
                    speeding_up = abs(self.speed_target) > abs(self.speed_current)
                    limit = (MAX_ACCEL if speeding_up else MAX_DECEL) * dt
                    step = max(-limit, min(limit, diff))
                    self.speed_current += step
                    speed = self.speed_current
                    angle = self.angle

                for esc_id in self.drive_ids:
                    self.bus.send(dm_velocity(esc_id, speed))
                for esc_id in self.steer_ids:
                    self.bus.send(sw_abs_position(esc_id, angle))
                self._sent_count += 1

            # fixed-rate scheduling: sleep to the next tick rather than a flat
            # dt, so the loop doesn't drift below CMD_RATE_HZ over time
            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()   # we're behind; resync

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

# ── High-level actions ────────────────────────────────────────────────────────

def enable_all(bus: CanBus, motion: MotionController):
    print("\n[ENABLE] Sending enable sequence...")
    for esc_id in motion.steer_ids:
        bus.send(sw_clear_fault(esc_id)); time.sleep(0.02)
    for esc_id in motion.steer_ids:
        bus.send(sw_abs_position(esc_id, 0.0)); time.sleep(0.02)
    for esc_id in motion.drive_ids:
        bus.send(dm_set_mode(esc_id, 3)); time.sleep(0.02)   # mode 3 = Velocity
    time.sleep(0.05)
    for esc_id in motion.drive_ids:
        bus.send(dm_enable(esc_id)); time.sleep(0.02)
    motion.set_speed_target(0.0)
    motion.set_angle(0.0)
    motion.set_active(True)     # motion loop now takes over sending frames
    print("[ENABLE] Done")

def disable_all(bus: CanBus, motion: MotionController):
    print("\n[DISABLE] Ramping speed down to zero...")
    motion.set_speed_target(0.0)
    motion.wait_until_settled(timeout=5.0)
    motion.set_active(False)    # stop the motion loop from sending further frames
    for esc_id in motion.steer_ids:
        bus.send(sw_abs_position(esc_id, 0.0))
    time.sleep(0.3)
    for esc_id in motion.steer_ids:
        bus.send(sw_disable(esc_id))
    for esc_id in motion.drive_ids:
        bus.send(dm_disable(esc_id))
    print("[DISABLE] All motors disabled")

def adjust_speed(motion: MotionController, delta: float):
    snap = motion.snapshot()
    new_target = snap["speed_target"] + delta
    motion.set_speed_target(new_target)
    print(f"\n[SPEED] target = {new_target:+.2f} rad/s")

def adjust_angle(motion: MotionController, delta_rad: float):
    snap = motion.snapshot()
    new_angle = snap["angle"] + delta_rad
    motion.set_angle(new_angle)
    print(f"\n[ANGLE] target = {math.degrees(new_angle):+.1f} deg")

# ── Display helpers ────────────────────────────────────────────────────────────

def print_feedback(bus: CanBus, motion: MotionController):
    snap = bus.snapshot()
    if not snap:
        print("  (no feedback frames received yet)")
    else:
        print(f"\n  Latest feedback  [{len(snap)} motor(s)]")
        for can_id, fb in sorted(snap.items()):
            err_name = ERR_NAMES.get(fb["err"], f"0x{fb['err']:X}")
            print(f"    can_id=0x{can_id:03X} ctrl_id={fb['ctrl_id']:2d} "
                  f"status={err_name:12s} pos_raw={fb['pos_raw']:5d} "
                  f"vel_raw={fb['vel_raw']:4d} torque_raw={fb['torque_raw']:4d} "
                  f"T_mos={fb['t_mos_c']}C T_rotor={fb['t_rotor_c']}C")

    state = motion.snapshot()
    active = motion._active.is_set()
    print(f"\n  Motion state (active={active}):")
    print(f"    speed: {state['speed_current']:+.3f} -> {state['speed_target']:+.3f} rad/s")
    print(f"    angle: {math.degrees(state['angle']):+.1f} deg")

# ── Keyboard input ────────────────────────────────────────────────────────────

def get_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

HELP = f"""
╔════════════════════════════════════════════╗
║   chassis_driver standalone test           ║
╠════════════════════════════════════════════╣
║  e   Enable selected motors                ║
║  d   Disable selected motors (ramp down)   ║
║  ─── drive / steer (vim-style) ──────────  ║
║  k   Increase speed by {SPEED_STEP:.2f} rad/s          ║
║  j   Decrease speed by {SPEED_STEP:.2f} rad/s          ║
║  l   Increase steer angle by {ANGLE_STEP_DEG:.0f} deg         ║
║  h   Decrease steer angle by {ANGLE_STEP_DEG:.0f} deg         ║
║  s   Stop drive (speed -> 0, ramps down)   ║
║  ─── info ───────────────────────────────  ║
║  f   Print motor feedback + current state  ║
║  r   Measure actual command send rate      ║
║  q   Quit                                  ║
╠════════════════════════════════════════════╣
║  accel={MAX_ACCEL:.1f} rad/s²  decel={MAX_DECEL:.1f} rad/s²        ║
║  cmd_rate={CMD_RATE_HZ:.0f} Hz (continuous while enabled)║
╚════════════════════════════════════════════╝
"""

def measure_send_rate(motion: MotionController, duration: float = 1.0) -> float:
    """Sample sent_count() over `duration` seconds to report actual Hz."""
    before = motion.sent_count()
    time.sleep(duration)
    after = motion.sent_count()
    return (after - before) / duration

# ── Argument parsing ──────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test CAN bus motors")

    parser.add_argument(
        "--damiao", nargs="*", type=int, metavar="ID", choices=range(4),
        help="Test damiao motors. No IDs = all 4. E.g. --damiao 1 3"
    )
    parser.add_argument(
        "--steadywin", nargs="*", type=int, metavar="ID", choices=range(4),
        help="Test steadywin motors. No IDs = all 4. E.g. --steadywin 2"
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Test all 8 motors"
    )

    args = parser.parse_args()

    if not args.all and args.damiao is None and args.steadywin is None:
        parser.error("No motors selected. Use --all, --damiao, or --steadywin.")

    return args

def resolve_selected_ids(args: argparse.Namespace) -> tuple[list[int], list[int]]:
    if args.all:
        return list(DRIVE_IDS), list(STEER_IDS)

    drive_ids: list[int] = []
    steer_ids: list[int] = []

    if args.damiao is not None:
        drive_ids = [DRIVE_IDS[i] for i in args.damiao] if len(args.damiao) > 0 else list(DRIVE_IDS)
    if args.steadywin is not None:
        steer_ids = [STEER_IDS[i] for i in args.steadywin] if len(args.steadywin) > 0 else list(STEER_IDS)

    return drive_ids, steer_ids

# ── Main ────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    drive_ids, steer_ids = resolve_selected_ids(args)

    print(f"Selected damiao IDs:    {drive_ids or '(none)'}")
    print(f"Selected steadywin IDs: {steer_ids or '(none)'}")
    print(HELP)


    print(f"\nOpening {CAN_CHANNEL} @ {CAN_BITRATE} bps...")
    bus = CanBus(CAN_CHANNEL, CAN_BITRATE)
    motion = MotionController(bus, drive_ids, steer_ids)
    print(HELP)

    try:
        while True:
            key = get_key()

            if key == 'e':
                enable_all(bus, motion)

            elif key == 'd':
                disable_all(bus, motion)

            elif key == 'k':
                if not motion.drive_ids:
                    print("\n  (no damiao motors selected — nothing to speed up)")
                else:
                    adjust_speed(motion, SPEED_STEP)

            elif key == 'j':
                if not motion.drive_ids:
                    print("\n  (no damiao motors selected — nothing to slow down)")
                else:
                    adjust_speed(motion, -SPEED_STEP)

            elif key == 'l':
                if not motion.steer_ids:
                    print("\n  (no steadywin motors selected — nothing to steer)")
                else:
                    adjust_angle(motion, math.radians(ANGLE_STEP_DEG))

            elif key == 'h':
                if not motion.steer_ids:
                    print("\n  (no steadywin motors selected — nothing to steer)")
                else:
                    adjust_angle(motion, -math.radians(ANGLE_STEP_DEG))

            elif key == 's':
                print("\n[STOP] speed target = 0, ramping down")
                motion.set_speed_target(0.0)

            elif key == 'f':
                print_feedback(bus, motion)

            elif key in ('q', '\x03'):
                print("\nQuitting...")
                break

            else:
                print(f"  (unknown key '{key}')")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\n[SHUTDOWN] Ramping down before exit...")
        motion.set_speed_target(0.0)
        motion.wait_until_settled(timeout=3.0)
        motion.set_active(False)
        for esc_id in motion.drive_ids:
            bus.send(dm_disable(esc_id))
        for esc_id in motion.steer_ids:
            bus.send(sw_disable(esc_id))
        motion.shutdown()
        bus.shutdown()


if __name__ == '__main__':
    main()
