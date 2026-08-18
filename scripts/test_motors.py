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
    +/=     Increase vx by VX_STEP (keeps current wz)
    -/_     Decrease vx by VX_STEP (keeps current wz)
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
# STEER_IDS = [11, 13, 15, 17]      # Steadywin, [FL, FR, RL, RR] — none in this build
DRIVE_IDS = [10]      # Damiao,    [FL, FR, RL, RR]
WHEEL_NAMES = ["FL", "FR", "RL", "RR"]

# Skid-steer geometry — ADJUST to your actual rover dimensions
WHEEL_RADIUS = 0.15      # meters
TRACK_WIDTH = 0.6        # meters, distance between left/right wheel centers

# Ramp control — ADJUST to taste
MAX_ACCEL = 2.0          # rad/s^2, applied when |target| > |current| (speeding up)
MAX_DECEL = 3.0          # rad/s^2, applied when |target| < |current| (slowing down)
CMD_RATE_HZ = 100.0      # command loop frequency — sends a velocity frame to every
                          # enabled motor every cycle, whether or not the target
                          # changed. This is also what keeps the motor's CAN
                          # communication-loss watchdog (TIMEOUT register) happy.

# ── vx/wz live state ──────────────────────────────────────────────────────
VX_STEP = 0.05    # m/s added/removed per '+'/'-' keypress

drive_state = {"vx": 0.0, "wz": 0.0}

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

# ── Ramp controller (smooth accel/decel) ──────────────────────────────────────

class RampController:
    """
    Continuous command loop, running at CMD_RATE_HZ (100 Hz by default).

    Every cycle — regardless of whether the target changed since the last
    cycle — it:
      1. Steps each wheel's commanded velocity toward its target, limited
         by MAX_ACCEL (speeding up) / MAX_DECEL (slowing down).
      2. Sends the resulting velocity frame to every enabled motor.

    This means motors get a steady stream of commands the whole time
    they're enabled, not just on state changes — which also keeps each
    motor's CAN communication-loss watchdog (TIMEOUT register) satisfied
    even while sitting still at zero velocity.

    Call set_targets() to change where wheels are headed — the loop takes
    care of getting them there smoothly.
    """
    def __init__(self, bus: CanBus, esc_ids: list[int]):
        self.bus = bus
        self.esc_ids = esc_ids
        self.current = {esc_id: 0.0 for esc_id in esc_ids}
        self.target = {esc_id: 0.0 for esc_id in esc_ids}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._active = threading.Event()   # only send frames while enabled
        self._sent_count = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_targets(self, speeds: dict[int, float]):
        with self._lock:
            self.target.update(speeds)

    def set_active(self, on: bool):
        if on:
            self._active.set()
        else:
            self._active.clear()

    def snapshot_current(self) -> dict[int, float]:
        with self._lock:
            return dict(self.current)

    def sent_count(self) -> int:
        return self._sent_count

    def wait_until_settled(self, tol: float = 0.02, timeout: float = 5.0):
        """Block until all wheels are within `tol` rad/s of their targets."""
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                done = all(abs(self.current[i] - self.target[i]) < tol for i in self.esc_ids)
            if done:
                return
            time.sleep(1.0 / CMD_RATE_HZ)

    def _run(self):
        dt = 1.0 / CMD_RATE_HZ
        next_tick = time.monotonic()
        while not self._stop.is_set():
            if self._active.is_set():
                with self._lock:
                    for esc_id in self.esc_ids:
                        cur = self.current[esc_id]
                        tgt = self.target[esc_id]
                        diff = tgt - cur
                        speeding_up = abs(tgt) > abs(cur)
                        limit = (MAX_ACCEL if speeding_up else MAX_DECEL) * dt
                        step = max(-limit, min(limit, diff))
                        self.current[esc_id] = cur + step
                    speeds = dict(self.current)
                for esc_id, spd in speeds.items():
                    self.bus.send(dm_velocity(esc_id, spd))
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

def enable_all(bus: CanBus, ramp: RampController):
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
    ramp.set_targets({esc_id: 0.0 for esc_id in DRIVE_IDS})
    ramp.set_active(True)     # ramp loop now takes over sending velocity frames
    print("[ENABLE] Done")

def disable_all(bus: CanBus, ramp: RampController):
    print("\n[DISABLE] Ramping down to zero...")
    drive_state["vx"], drive_state["wz"] = 0.0, 0.0
    ramp.set_targets({esc_id: 0.0 for esc_id in DRIVE_IDS})
    ramp.wait_until_settled(timeout=5.0)
    ramp.set_active(False)    # stop the ramp loop from sending further frames
    for esc_id in STEER_IDS:
        bus.send(sw_abs_position(esc_id, 0.0))
    time.sleep(0.3)
    for esc_id in STEER_IDS:
        bus.send(sw_disable(esc_id))
    for esc_id in DRIVE_IDS:
        bus.send(dm_disable(esc_id))
    print("[DISABLE] All motors disabled")

def set_diff_drive_target(ramp: RampController, vx: float, wz: float):
    """Simple skid-steer mixing: left/right wheel target speeds from vx, wz.
    Sets the target only — the RampController smoothly accelerates/decelerates
    toward it in the background."""
    left_speed = (vx - wz * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    right_speed = (vx + wz * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
    # DRIVE_IDS order = [FL, FR, RL, RR]
    speeds = [left_speed, right_speed, left_speed, right_speed]
    ramp.set_targets(dict(zip(DRIVE_IDS, speeds)))
    return speeds

def adjust_vx(ramp: RampController, state: dict, delta: float):
    """Nudge vx up/down by `delta`, keep current wz, reapply target."""
    state["vx"] += delta
    speeds = set_diff_drive_target(ramp, vx=state["vx"], wz=state["wz"])
    print(f"\n[VX] vx = {state['vx']:+.2f} m/s   (wz = {state['wz']:+.2f} rad/s)")
    print(f"  target wheel speeds (rad/s): {speeds}")
    return state["vx"]

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

HELP = f"""
╔══════════════════════════════════════════╗
║   chassis_driver standalone test (no ROS) ║
╠══════════════════════════════════════════╣
║  e   Enable all motors                   ║
║  d   Disable all motors (smooth ramp-down)║
║  ─── scenarios (smooth ramp) ───────────║
║  1   Straight forward  0.5 m/s           ║
║  2   Spin in place     1.0 rad/s         ║
║  3   Turn left         0.5 m/s + 0.5ω   ║
║  4   (steer test placeholder — no steer  ║
║       motors configured)                 ║
║  +/= Increase vx by {VX_STEP:.2f} m/s (keeps wz)  ║
║  -/_ Decrease vx by {VX_STEP:.2f} m/s (keeps wz)  ║
║  s   Stop (target = 0, ramps down)       ║
║  ─── info ──────────────────────────────║
║  f   Print motor feedback + current speed║
║  r   Measure actual command send rate    ║
║  q   Quit                                ║
╠══════════════════════════════════════════╣
║  accel={MAX_ACCEL:.1f} rad/s²  decel={MAX_DECEL:.1f} rad/s²  ║
║  cmd_rate={CMD_RATE_HZ:.0f} Hz (continuous while enabled)   ║
╚══════════════════════════════════════════╝
"""

def print_ramp_state(ramp: RampController):
    cur = ramp.snapshot_current()
    with ramp._lock:
        tgt = dict(ramp.target)
    active = ramp._active.is_set()
    print(f"\n  Ramp state (active={active}, ~{CMD_RATE_HZ:.0f} Hz target):")
    for esc_id in DRIVE_IDS:
        print(f"    esc={esc_id:2d}  {cur[esc_id]:+.3f} → {tgt[esc_id]:+.3f} rad/s")

def measure_send_rate(ramp: RampController, duration: float = 1.0) -> float:
    """Sample sent_count() over `duration` seconds to report actual Hz."""
    before = ramp.sent_count()
    time.sleep(duration)
    after = ramp.sent_count()
    return (after - before) / duration

def main():
    print(f"Opening {CAN_CHANNEL} @ {CAN_BITRATE} bps...")
    bus = CanBus(CAN_CHANNEL, CAN_BITRATE)
    ramp = RampController(bus, DRIVE_IDS)
    print(HELP)

    try:
        while True:
            key = get_key()

            if key == 'e':
                enable_all(bus, ramp)

            elif key == 'd':
                disable_all(bus, ramp)

            elif key == '1':
                print("\nSCENARIO: STRAIGHT FORWARD 0.5 m/s (ramping...)")
                drive_state["vx"], drive_state["wz"] = 0.5, 0.0
                speeds = set_diff_drive_target(ramp, vx=drive_state["vx"], wz=drive_state["wz"])
                print(f"  target wheel speeds (rad/s): {speeds}")

            elif key == '2':
                print("\nSCENARIO: SPIN IN PLACE 1.0 rad/s (ramping...)")
                drive_state["vx"], drive_state["wz"] = 0.0, 1.0
                speeds = set_diff_drive_target(ramp, vx=drive_state["vx"], wz=drive_state["wz"])
                print(f"  target wheel speeds (rad/s): {speeds}")

            elif key == '3':
                print("\nSCENARIO: TURN LEFT (ramping...)")
                drive_state["vx"], drive_state["wz"] = 0.5, 0.5
                speeds = set_diff_drive_target(ramp, vx=drive_state["vx"], wz=drive_state["wz"])
                print(f"  target wheel speeds (rad/s): {speeds}")

            elif key == '4':
                print("\n[TEST 4] No steer motors configured — skipping.")

            elif key in ('+', '='):
                adjust_vx(ramp, drive_state, VX_STEP)

            elif key in ('-', '_'):
                adjust_vx(ramp, drive_state, -VX_STEP)

            elif key == 's':
                print("\n[STOP] Target = 0, ramping down")
                drive_state["vx"], drive_state["wz"] = 0.0, 0.0
                set_diff_drive_target(ramp, vx=0.0, wz=0.0)

            elif key == 'f':
                print_feedback(bus)
                print_ramp_state(ramp)

            elif key == 'r':
                print("\n[MEASURE] Sampling send rate for 1s...")
                hz = measure_send_rate(ramp, duration=1.0)
                print(f"  actual command rate ≈ {hz:.1f} Hz  (target: {CMD_RATE_HZ:.0f} Hz)")

            elif key in ('q', '\x03'):
                print("\nQuitting...")
                break

            else:
                print(f"  (unknown key '{key}')")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\n[SHUTDOWN] Ramping down before exit...")
        drive_state["vx"], drive_state["wz"] = 0.0, 0.0
        set_diff_drive_target(ramp, vx=0.0, wz=0.0)
        ramp.wait_until_settled(timeout=3.0)
        ramp.set_active(False)
        for esc_id in DRIVE_IDS:
            bus.send(dm_disable(esc_id))
        ramp.shutdown()
        bus.shutdown()


if __name__ == '__main__':
    main()