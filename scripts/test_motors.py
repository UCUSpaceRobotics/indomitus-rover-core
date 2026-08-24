#!/usr/bin/env python3
"""
chassis_driver test — standalone TUI, NO ROS2 required.

Talks directly to the CAN bus via python-can / SocketCAN.

Install dependencies:
    pip install python-can textual

Make sure the CAN interface is up first, e.g.:
    sudo ip link set can0 up type can bitrate 500000

Motor selection (choose which motors get enabled/commanded):
    --all                 test all 8 motors
    --damiao [ID ...]     test damiao motors by CAN ID. No IDs = all 4. E.g. --damiao 10 14
    --steadywin [ID ...]  test steadywin motors by CAN ID. No IDs = all 4. E.g. --steadywin 13

Controls (vim-style) — also shown in the footer:
    e   Enable selected motors
    d   Disable selected motors (ramp down)
    k   Increase drive speed
    j   Decrease drive speed
    l   Increase steer angle
    h   Decrease steer angle
    s   Stop drive (speed target -> 0, ramps down)
    f   Print latest decoded feedback + per-motor diagnostics
    r   Measure actual command send rate
    1   Toggle the raw candump panel (right of the log)
    q   Quit

Motor health panel (middle) — green if the motor is sending diagnostics,
red if it is not (damiao: MIT feedback frame, steadywin: 0xAE response).
"""
import argparse
import math
import queue
import re
import struct
import subprocess
import sys
import threading
import time

try:
    import can
    from textual.app import App, ComposeResult
    from textual.binding import Binding
    from textual.containers import Horizontal
    from textual.widgets import Header, Footer, Static, RichLog
except ImportError as e:
    pkg = {"can": "python-can"}.get(e.name, e.name)
    print(f"Missing dependency: {pkg}")
    print()
    print("Install with:")
    print(f"    pip install {pkg}")
    sys.exit(1)

# ── Config ──────────────────────────────────────────────────────────────

CAN_CHANNEL = "can0"
CAN_BITRATE = 500_000             # must match what's actually configured on the bus

STEER_IDS = [11, 13, 15, 17]      # Steadywin, [FL, FR, RL, RR]
DRIVE_IDS = [10, 12, 14, 16]      # Damiao,    [FL, FR, RL, RR]

MAX_ACCEL = 1.0          # rad/s^2, applied when |target| > |current| (speeding up)
MAX_DECEL = 2.0          # rad/s^2, applied when |target| < |current| (slowing down)
CMD_RATE_HZ = 100.0       # command loop frequency — see MotionController

SPEED_STEP = 0.1          # rad/s added/removed per 'k'/'j' keypress
ANGLE_STEP_DEG = 5.0      # degrees added/removed per 'l'/'h' keypress

STALE_THRESHOLD_S = 1.0   # motor ID shown red if no feedback received within this long
DIAG_POLL_HZ = 2.0        # 0xAE status query rate per steadywin motor — see DiagnosticsPoller

CANDUMP_CMD = ["candump", "-tz"]   # channel is appended at runtime; -tz = relative timestamps

# ── Frame builders ────────────────────────────────────────────────────────

def _rad_to_counts(angle_rad: float) -> int:
    return int(angle_rad * 16384 / (2 * math.pi))

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
    """0xAE — ask for bus voltage / current / temperature / mode / fault code.
    The motor answers at its own ID with a DLC 8 frame (protocol V3.06b0 p.5)."""
    return can.Message(arbitration_id=esc_id, data=bytes([0xAE]), is_extended_id=False)

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

# ── Feedback decode ─────────────────────────────────────────────────────────
#
# The two vendors speak different protocols, so they get different decoders and
# frames are routed by sender CAN ID (see CanBus._listen) rather than sniffed
# from the payload:
#
#   Damiao    — one fixed MIT feedback frame, always DLC 8 (manual p.8).
#   Steadywin — command-code-keyed responses of varying DLC (protocol V3.06b0):
#               0xAE/0xCF -> DLC 8 diagnostics, 0xA3/0xC2/0xC3/0xC4 -> DLC 7
#               position, 0xAF -> DLC 2 fault ack.
#
# Running a steadywin frame through the damiao layout yields plausible-looking
# nonsense — a 0xAE encoder-fault report decodes as "OVERCURRENT" — so each
# decoder validates the payload it is handed and returns None otherwise.

def decode_damiao_feedback(msg: can.Message) -> dict | None:
    """Damiao MIT feedback frame (manual p.8). Always DLC 8."""
    d = msg.data
    if len(d) != 8:
        return None
    return {
        "can_id": msg.arbitration_id,
        "ctrl_id": d[0] & 0x0F,
        "err": (d[0] >> 4) & 0x0F,
        "pos_raw": (d[1] << 8) | d[2],
        "vel_raw": (d[3] << 4) | (d[4] >> 4),
        "torque_raw": ((d[4] & 0x0F) << 8) | d[5],
        "t_mos_c": d[6],
        "t_rotor_c": d[7],
    }

ERR_NAMES = {
    0: "DISABLED", 1: "ENABLED", 8: "OVERVOLTAGE", 9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "COIL_OVERTEMP",
    0xD: "COMM_LOST", 0xE: "OVERLOAD",
}

SW_MODES = {0: "OFF", 1: "VOLTAGE", 2: "IQ_CURRENT", 3: "SPEED", 4: "POSITION"}

# Fault bitmask, 0xAE byte[7] (protocol V3.06b0 p.5)
SW_FAULT_BITS = [
    (0x01, "VOLTAGE"), (0x02, "CURRENT"), (0x04, "TEMPERATURE"),
    (0x08, "ENCODER"), (0x40, "HARDWARE"), (0x80, "SOFTWARE"),
]
SW_FAULT_RESERVED = 0x30   # bits 4-5 undocumented — if set, the value means something we don't know

def sw_fault_names(code: int) -> str:
    if code == 0:
        return "OK"
    names = [name for bit, name in SW_FAULT_BITS if code & bit]
    if code & SW_FAULT_RESERVED:
        names.append(f"RESERVED(0x{code & SW_FAULT_RESERVED:02X})")
    return "+".join(names)

def decode_steadywin(msg: can.Message) -> dict | None:
    """Steadywin response frame (protocol V3.06b0). Dispatches on the command
    code in D[0] and returns a partial state dict for the caller to merge —
    position and diagnostics arrive in separate frames."""
    d = msg.data
    if not d:
        return None
    cmd = d[0]

    # 0xAE status report (p.5); the 0xCF disable response carries the same
    # payload (p.11). A motor with a latched fault also emits 0xAE unprompted
    # every 200 ms — which is the only time diagnostics arrive without a poll.
    if cmd in (0xAE, 0xCF) and len(d) >= 8:
        return {
            "kind": "diag",
            "voltage": ((d[2] << 8) | d[1]) * 0.01,   # 2u, 0.01 V
            "current": ((d[4] << 8) | d[3]) * 0.01,   # 2u, 0.01 A
            "temp_c": d[5],
            "mode": d[6],
            "fault": d[7],
        }

    # 0xA3 angle query and the 0xC2/0xC3/0xC4 motion responses share one
    # payload: single-turn uint16 + multi-turn int32, 16384 counts/rev (p.5).
    # This is the frame the old length check threw away — it is DLC 7, and it
    # is the only thing a steer motor sends back during normal driving.
    if cmd in (0xA3, 0xC2, 0xC3, 0xC4) and len(d) >= 7:
        multi_counts = struct.unpack('<i', bytes(d[3:7]))[0]
        return {
            "kind": "pos",
            "angle_rad": multi_counts * (2 * math.pi) / 16384,
            "single_counts": (d[2] << 8) | d[1],
        }

    # 0xAF clear-fault ack — fault bitmask only, no other telemetry (p.6)
    if cmd == 0xAF and len(d) >= 2:
        return {"kind": "fault_ack", "fault": d[1]}

    return None

# ── Bus wrapper with background listener ─────────────────────────────────────

class CanBus:
    """Receives in the background and hands each frame to the decoder for the
    vendor that owns that CAN ID — steer IDs to the steadywin decoder,
    everything else to the damiao one.

    Tracks two clocks per motor: last decoded frame of any kind, and last frame
    that actually carried diagnostics. For damiao they are the same clock (the
    MIT feedback frame is the diagnostic channel). For steadywin they are not:
    a motor answering position commands while ignoring the 0xAE query is alive
    but mute, and the panel must not show that as healthy."""

    def __init__(self, channel: str, bitrate: int,
                 drive_ids: list[int], steer_ids: list[int]):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=bitrate)
        self.steer_ids = set(steer_ids)
        self.damiao_feedback: dict[int, dict] = {}
        self.steadywin_state: dict[int, dict] = {}  # merged across response types
        self.last_seen: dict[int, float] = {}       # can_id -> monotonic, any decoded frame
        self.last_diag: dict[int, float] = {}       # can_id -> monotonic, diagnostics only
        self.undecoded: dict[int, int] = {}         # can_id -> frames we could not decode
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.2)
            if msg is None:
                continue
            if msg.arbitration_id in self.steer_ids:
                self._on_steadywin(msg)
            else:
                self._on_damiao(msg)

    def _on_steadywin(self, msg: can.Message):
        can_id = msg.arbitration_id
        part = decode_steadywin(msg)
        if part is None:
            with self._lock:
                self.undecoded[can_id] = self.undecoded.get(can_id, 0) + 1
            return
        kind = part.pop("kind")
        now = time.monotonic()
        with self._lock:
            state = self.steadywin_state.setdefault(can_id, {})
            state.update(part)
            self.last_seen[can_id] = now
            if kind == "diag":
                state["diag_ts"] = now
                self.last_diag[can_id] = now
            elif kind == "pos":
                state["pos_ts"] = now

    def _on_damiao(self, msg: can.Message):
        fb = decode_damiao_feedback(msg)
        if not fb:
            return
        now = time.monotonic()
        with self._lock:
            self.damiao_feedback[msg.arbitration_id] = fb
            self.last_seen[msg.arbitration_id] = now
            # The MIT feedback frame carries the ERR nibble and both
            # temperatures in every frame — receiving it *is* the diagnostic.
            self.last_diag[msg.arbitration_id] = now

    def send(self, msg: can.Message):
        self.bus.send(msg)

    def snapshot_damiao(self) -> dict[int, dict]:
        with self._lock:
            return dict(self.damiao_feedback)

    def snapshot_steadywin(self) -> dict[int, dict]:
        with self._lock:
            return {k: dict(v) for k, v in self.steadywin_state.items()}

    def snapshot_ages(self) -> dict[int, float]:
        """seconds since any frame was decoded, per can_id — empty if never seen"""
        now = time.monotonic()
        with self._lock:
            return {can_id: now - ts for can_id, ts in self.last_seen.items()}

    def snapshot_diag_ages(self) -> dict[int, float]:
        """seconds since diagnostics last arrived, per can_id — empty if never"""
        now = time.monotonic()
        with self._lock:
            return {can_id: now - ts for can_id, ts in self.last_diag.items()}

    def snapshot_undecoded(self) -> dict[int, int]:
        with self._lock:
            return dict(self.undecoded)

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()

# ── Steadywin diagnostics poller ────────────────────────────────────────────

class DiagnosticsPoller:
    """Periodically asks each steadywin motor for its status (0xAE).

    Steadywin motors only volunteer diagnostics when a fault is already latched
    — every 200 ms, protocol V3.06b0 p.5. Without an explicit poll the
    voltage/current/temperature/mode/fault fields never arrive at all, so a
    healthy motor and an absent one look identical. Damiao needs no equivalent:
    its MIT feedback frame carries that data unprompted.

    Runs whether or not the motors are enabled — knowing a motor answers before
    you command it is the point."""

    def __init__(self, bus: CanBus, steer_ids: list[int], rate_hz: float = DIAG_POLL_HZ):
        self.bus = bus
        self.steer_ids = steer_ids
        self.rate_hz = rate_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        period = 1.0 / self.rate_hz
        while not self._stop.is_set():
            for esc_id in self.steer_ids:
                if self._stop.is_set():
                    return
                try:
                    self.bus.send(sw_status_query(esc_id))
                except can.CanError:
                    pass   # bus trouble shows up in the CAN link panel; keep polling
                self._stop.wait(0.005)   # space the queries out on the bus
            self._stop.wait(period)

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

# ── CAN link stats (from `ip -details -statistics link show <iface>`) ───────
# Read-only, no sudo needed. Gives us bitrate, bus-error state, and rx/tx
# error/drop counters straight from the kernel's SocketCAN driver — useful
# for spotting bus issues (wiring, termination, bitrate mismatch) that
# python-can itself won't surface.

def parse_ip_link_output(text: str) -> dict:
    result: dict = {"state": None, "bitrate": None, "berr_tx": None,
                     "berr_rx": None, "restart_ms": None, "rx": {}, "tx": {}}
    lines = text.splitlines()
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("can") and " state " in stripped:
            m = re.search(r'state\s+(\S+)', stripped)
            if m:
                result["state"] = m.group(1)
            m = re.search(r'berr-counter tx (\d+) rx (\d+)', stripped)
            if m:
                result["berr_tx"] = int(m.group(1))
                result["berr_rx"] = int(m.group(2))
            m = re.search(r'restart-ms (\d+)', stripped)
            if m:
                result["restart_ms"] = int(m.group(1))
        if "bitrate" in stripped:
            m = re.search(r'bitrate (\d+)', stripped)
            if m:
                result["bitrate"] = int(m.group(1))
        if stripped.startswith("RX:"):
            headers = stripped.split()[1:]
            if i + 1 < len(lines):
                values = lines[i + 1].split()
                result["rx"] = dict(zip(headers, (int(v) for v in values)))
        if stripped.startswith("TX:"):
            headers = stripped.split()[1:]
            if i + 1 < len(lines):
                values = lines[i + 1].split()
                result["tx"] = dict(zip(headers, (int(v) for v in values)))
    return result

class CanLinkMonitor:
    """Polls `ip -details -statistics link show <channel>` in the background
    and keeps the latest parsed snapshot available. Runs independently of
    whether python-can managed to open the bus — useful for diagnosing why
    it couldn't."""

    def __init__(self, channel: str, poll_interval: float = 1.0):
        self.channel = channel
        self.poll_interval = poll_interval
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snapshot: dict = {"error": "not polled yet"}
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            self._poll_once()
            self._stop.wait(self.poll_interval)

    def _poll_once(self):
        try:
            proc = subprocess.run(
                ["ip", "-details", "-statistics", "link", "show", self.channel],
                capture_output=True, text=True, timeout=2.0,
            )
        except Exception as e:
            with self._lock:
                self._snapshot = {"error": str(e)}
            return
        if proc.returncode != 0:
            with self._lock:
                self._snapshot = {"error": proc.stderr.strip() or f"exit code {proc.returncode}"}
            return
        parsed = parse_ip_link_output(proc.stdout)
        parsed["error"] = None
        with self._lock:
            self._snapshot = parsed

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snapshot)

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

# ── candump reader ──────────────────────────────────────────────────────────

class CandumpMonitor:
    """Runs `candump <channel>` and buffers its output for the TUI to drain.

    Started only while the dump panel is visible: candump on a loaded bus emits
    a few hundred lines a second, and there is no reason to pay for that while
    nobody is looking. Lines land in a bounded queue — if the UI falls behind,
    the oldest are dropped and counted rather than growing without limit."""

    def __init__(self, channel: str, max_buffered: int = 2000):
        self.channel = channel
        self.lines: queue.Queue = queue.Queue(maxsize=max_buffered)
        self.dropped = 0
        self.error: str | None = None
        self._proc: subprocess.Popen | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> bool:
        """Spawn candump. Returns False and sets .error if it could not start."""
        cmd = CANDUMP_CMD + [self.channel]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except FileNotFoundError:
            self.error = f"{cmd[0]} not found — install can-utils"
            return False
        except OSError as e:
            self.error = str(e)
            return False
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()
        return True

    def _read(self):
        stdout = self._proc.stdout if self._proc else None
        if stdout is None:
            return
        # Iteration ends when stop() terminates the process and closes the pipe.
        for line in stdout:
            if self._stop.is_set():
                break
            try:
                self.lines.put_nowait(line.rstrip("\n"))
            except queue.Full:
                self.dropped += 1

    def drain(self, limit: int = 300) -> list[str]:
        """Pop up to `limit` buffered lines. Capped so one slow frame of the UI
        cannot block on a bus that is producing faster than we can render."""
        out = []
        for _ in range(limit):
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                break
        return out

    def stop(self):
        self._stop.set()
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

# ── Motion controller ──────────────────────────────────────────────────────

class MotionController:
    def __init__(self, bus: CanBus, drive_ids: list[int], steer_ids: list[int]):
        self.bus = bus
        self.drive_ids = drive_ids
        self.steer_ids = steer_ids

        self.speed_current = 0.0
        self.speed_target = 0.0
        self.angle = 0.0   # radians

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._active = threading.Event()
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
        (self._active.set if on else self._active.clear)()

    def is_active(self) -> bool:
        return self._active.is_set()

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

            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

# ── High-level actions (blocking — always call from a worker thread) ────────

def enable_all(bus: CanBus, motion: MotionController):
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
    motion.set_active(True)

def disable_all(bus: CanBus, motion: MotionController):
    motion.set_speed_target(0.0)
    motion.wait_until_settled(timeout=5.0)
    motion.set_active(False)
    for esc_id in motion.steer_ids:
        bus.send(sw_abs_position(esc_id, 0.0))
    time.sleep(0.3)
    for esc_id in motion.steer_ids:
        bus.send(sw_disable(esc_id))
    for esc_id in motion.drive_ids:
        bus.send(dm_disable(esc_id))

def measure_send_rate(motion: MotionController, duration: float = 1.0) -> float:
    before = motion.sent_count()
    time.sleep(duration)
    after = motion.sent_count()
    return (after - before) / duration

# ── Argument parsing ──────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test CAN bus motors")
    parser.add_argument(
        "--damiao", nargs="*", type=int, metavar="ID",
        help=f"Test damiao motors by CAN ID. No IDs = known set {DRIVE_IDS}. "
             f"Any ID works, including ones not in that list. E.g. --damiao 10 14 20"
    )
    parser.add_argument(
        "--steadywin", nargs="*", type=int, metavar="ID",
        help=f"Test steadywin motors by CAN ID. No IDs = known set {STEER_IDS}. "
             f"Any ID works, including ones not in that list. E.g. --steadywin 13 19"
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Test all motors in the known set"
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
        drive_ids = list(args.damiao) if len(args.damiao) > 0 else list(DRIVE_IDS)
    if args.steadywin is not None:
        steer_ids = list(args.steadywin) if len(args.steadywin) > 0 else list(STEER_IDS)
    return drive_ids, steer_ids

# ── TUI app ─────────────────────────────────────────────────────────────────

class ChassisTUI(App):
    CSS = """
    Screen {
        background: #121214;
    }
    #status_row {
        height: 5;
    }
    #can_status {
        width: 2fr;
        height: 100%;
        background: #1c1c1e;
        border: round green;
        padding: 0 1;
        content-align: left middle;
    }
    #motors_status {
        width: 1fr;
        height: 100%;
        background: #1c1c1e;
        border: round yellow;
        padding: 0 1;
        content-align: left middle;
    }
    #motion_status {
        width: 1fr;
        height: 100%;
        background: #1c1c1e;
        border: round cyan;
        padding: 0 1;
        content-align: left middle;
    }
    #body_row {
        height: 1fr;
    }
    #log {
        width: 1fr;
        background: #1c1c1e;
        border: round #4a4a4a;
        scrollbar-size-vertical: 1;
        scrollbar-background: #1c1c1e;
        scrollbar-background-hover: #1c1c1e;
        scrollbar-background-active: #1c1c1e;
        scrollbar-color: #4a4a4a;
        scrollbar-color-hover: #888888;
        scrollbar-color-active: #aaaaaa;
    }
    #candump {
        width: 1fr;
        display: none;
        background: #1c1c1e;
        border: round magenta;
        scrollbar-size-vertical: 1;
        scrollbar-background: #1c1c1e;
        scrollbar-color: #4a4a4a;
    }
    """

    BINDINGS = [
        Binding("e", "enable", "Enable"),
        Binding("d", "disable", "Disable"),
        Binding("k", "speed_up", "Speed +"),
        Binding("j", "speed_down", "Speed -"),
        Binding("l", "angle_up", "Angle +"),
        Binding("h", "angle_down", "Angle -"),
        Binding("s", "stop", "Stop"),
        Binding("f", "feedback", "Feedback"),
        Binding("r", "measure_rate", "Rate"),
        Binding("1", "toggle_candump", "candump"),
        Binding("q", "quit_app", "Quit"),
    ]

    def __init__(self, drive_ids: list[int], steer_ids: list[int]):
        super().__init__()
        self.drive_ids = drive_ids
        self.steer_ids = steer_ids
        self.bus: CanBus | None = None
        self.motion: MotionController | None = None
        self.link_monitor: CanLinkMonitor | None = None
        self.diag_poller: DiagnosticsPoller | None = None
        self.candump: CandumpMonitor | None = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal(id="status_row"):
            yield Static(id="can_status")
            yield Static(id="motors_status")
            yield Static(id="motion_status")
        with Horizontal(id="body_row"):
            yield RichLog(id="log", wrap=True, highlight=True, markup=True)
            # Raw candump text: no markup (frames contain "[8]", which Textual
            # would try to parse as a tag) and no wrap, so frames stay aligned.
            yield RichLog(id="candump", wrap=False, highlight=False,
                          markup=False, max_lines=2000)
        yield Footer()

    def on_mount(self) -> None:
        log = self.query_one("#log", RichLog)
        log.write(f"[bold]damiao IDs:[/bold] {self.drive_ids or '(none)'}")
        log.write(f"[bold]steadywin IDs:[/bold] {self.steer_ids or '(none)'}")

        # Link stats poll independently of python-can — useful for diagnosing
        # bus issues even when the python-can bus below fails to open.
        self.link_monitor = CanLinkMonitor(CAN_CHANNEL)

        log.write(f"Opening {CAN_CHANNEL} @ {CAN_BITRATE} bps...")
        try:
            self.bus = CanBus(CAN_CHANNEL, CAN_BITRATE, self.drive_ids, self.steer_ids)
            self.motion = MotionController(self.bus, self.drive_ids, self.steer_ids)
            if self.steer_ids:
                # Steadywin sends nothing diagnostic unless asked — see DiagnosticsPoller
                self.diag_poller = DiagnosticsPoller(self.bus, self.steer_ids)
                log.write(f"[green]Ready.[/green] polling steadywin 0xAE "
                          f"at {DIAG_POLL_HZ:.0f} Hz")
            else:
                log.write("[green]Ready.[/green]")
        except Exception as e:
            log.write(f"[bold red]Failed to open CAN bus: {e}[/bold red]")

        self.set_interval(1 / 10, self.update_status)
        self.set_interval(1 / 10, self.drain_candump)

    def update_status(self) -> None:
        # ── left panel: CAN link state ──
        can_lines: list[str] = []
        link = self.link_monitor.snapshot() if self.link_monitor else {}
        if link.get("error"):
            can_lines.append(f"{CAN_CHANNEL}: [red]{link['error']}[/red]")
        else:
            can_state = link.get("state") or "?"
            color = {
                "ERROR-ACTIVE": "green",
                "ERROR-WARNING": "yellow",
                "ERROR-PASSIVE": "red",
                "BUS-OFF": "bold red",
            }.get(can_state, "white")
            bitrate = link.get("bitrate")
            bitrate_s = f"{bitrate / 1000:.0f} kbps" if bitrate else "?"
            berr_tx = link.get("berr_tx")
            berr_rx = link.get("berr_rx")
            rx = link.get("rx", {})
            tx = link.get("tx", {})
            can_lines.append(
                f"{CAN_CHANNEL}: [{color}]{can_state}[/{color}]  {bitrate_s}  "
                f"berr(tx={berr_tx} rx={berr_rx})"
            )
            can_lines.append(
                f"rx: errors={rx.get('errors', '?')} dropped={rx.get('dropped', '?')}   "
                f"tx: errors={tx.get('errors', '?')} dropped={tx.get('dropped', '?')}"
            )
        self.query_one("#can_status", Static).update("\n".join(can_lines))

        # ── middle panel: per-motor health ──
        # green = diagnostics arriving, red = not. For damiao that is the MIT
        # feedback frame, for steadywin the 0xAE response to our poll.
        diag_ages = self.bus.snapshot_diag_ages() if self.bus else {}

        def has_diag(esc_id: int) -> bool:
            age = diag_ages.get(esc_id)
            return age is not None and age < STALE_THRESHOLD_S

        def fmt_id(esc_id: int) -> str:
            color = "green" if has_diag(esc_id) else "red"
            return f"[{color}]{esc_id}[/{color}]"

        drive_line = "D: " + (" ".join(fmt_id(i) for i in self.drive_ids) if self.drive_ids else "(none)")
        steer_line = "S: " + (" ".join(fmt_id(i) for i in self.steer_ids) if self.steer_ids else "(none)")
        selected = self.drive_ids + self.steer_ids
        n_diag = sum(1 for i in selected if has_diag(i))
        summary = f"diag {n_diag}/{len(selected)}" if selected else ""
        self.query_one("#motors_status", Static).update(
            f"{drive_line}\n{steer_line}\n{summary}")

        # ── right panel: current speed / angle ──
        if self.motion:
            state = self.motion.snapshot()
            motion_text = (
                f"speed: {state['speed_current']:+.2f} rad/s\n"
                f"angle: {math.degrees(state['angle']):+.1f} deg"
            )
        else:
            motion_text = "speed: --\nangle: --"
        self.query_one("#motion_status", Static).update(motion_text)

    def log_write(self, msg: str) -> None:
        self.query_one("#log", RichLog).write(msg)

    # ── actions ──────────────────────────────────────────────────────────

    def action_enable(self) -> None:
        if not self.motion:
            self.log_write("[red]e: CAN bus not open[/red]")
            return
        self.log_write("[green]e[/green] enabling...")
        self.run_worker(self._enable_worker, thread=True)

    def _enable_worker(self) -> None:
        enable_all(self.bus, self.motion)
        self.call_from_thread(self.log_write, "  -> enable sequence sent")

    def action_disable(self) -> None:
        if not self.motion:
            return
        self.log_write("[yellow]d[/yellow] disabling (ramping down)...")
        self.run_worker(self._disable_worker, thread=True)

    def _disable_worker(self) -> None:
        disable_all(self.bus, self.motion)
        self.call_from_thread(self.log_write, "  -> all motors disabled")

    def action_speed_up(self) -> None:
        if not self.motion or not self.motion.drive_ids:
            self.log_write("[red]k: no damiao motors selected[/red]")
            return
        new_target = self.motion.snapshot()["speed_target"] + SPEED_STEP
        self.motion.set_speed_target(new_target)
        self.log_write(f"[cyan]k[/cyan] speed target -> {new_target:+.2f} rad/s")

    def action_speed_down(self) -> None:
        if not self.motion or not self.motion.drive_ids:
            self.log_write("[red]j: no damiao motors selected[/red]")
            return
        new_target = self.motion.snapshot()["speed_target"] - SPEED_STEP
        self.motion.set_speed_target(new_target)
        self.log_write(f"[cyan]j[/cyan] speed target -> {new_target:+.2f} rad/s")

    def action_angle_up(self) -> None:
        if not self.motion or not self.motion.steer_ids:
            self.log_write("[red]l: no steadywin motors selected[/red]")
            return
        new_angle = self.motion.snapshot()["angle"] + math.radians(ANGLE_STEP_DEG)
        self.motion.set_angle(new_angle)
        self.log_write(f"[cyan]l[/cyan] angle target -> {math.degrees(new_angle):+.1f} deg")

    def action_angle_down(self) -> None:
        if not self.motion or not self.motion.steer_ids:
            self.log_write("[red]h: no steadywin motors selected[/red]")
            return
        new_angle = self.motion.snapshot()["angle"] - math.radians(ANGLE_STEP_DEG)
        self.motion.set_angle(new_angle)
        self.log_write(f"[cyan]h[/cyan] angle target -> {math.degrees(new_angle):+.1f} deg")

    def action_stop(self) -> None:
        if not self.motion:
            return
        self.motion.set_speed_target(0.0)
        self.log_write("[yellow]s[/yellow] stop -> speed target = 0")

    def action_feedback(self) -> None:
        if not self.bus:
            return
        dm = self.bus.snapshot_damiao()
        sw = self.bus.snapshot_steadywin()
        ages = self.bus.snapshot_ages()

        def age_s(esc_id: int) -> str:
            age = ages.get(esc_id)
            return f"{age:.1f}s ago" if age is not None else "never"

        self.log_write("f: feedback")

        if self.drive_ids:
            self.log_write("  [bold]damiao[/bold] — MIT feedback frame")
        for esc_id in self.drive_ids:
            fb = dm.get(esc_id)
            if fb is None:
                self.log_write(f"   [red]{esc_id:3d} silent — no feedback frame received[/red]")
                continue
            err_name = ERR_NAMES.get(fb["err"], f"0x{fb['err']:X}")
            self.log_write(
                f"   {esc_id:3d} status={err_name:12s} pos={fb['pos_raw']:5d} "
                f"vel={fb['vel_raw']:4d} torque={fb['torque_raw']:4d} "
                f"T_mos={fb['t_mos_c']}C T_rotor={fb['t_rotor_c']}C  ({age_s(esc_id)})"
            )

        if self.steer_ids:
            self.log_write("  [bold]steadywin[/bold] — 0xAE status response")
        for esc_id in self.steer_ids:
            state = sw.get(esc_id)
            if state is None:
                self.log_write(f"   [red]{esc_id:3d} silent — no response to any command[/red]")
                continue
            pos = (f"pos={math.degrees(state['angle_rad']):+8.1f}deg"
                   if "angle_rad" in state else "pos=      --")
            if "diag_ts" in state:
                mode = SW_MODES.get(state["mode"], f"0x{state['mode']:02X}")
                fault = sw_fault_names(state["fault"])
                healthy = state["fault"] == 0 and state["mode"] != 0
                color = "green" if healthy else "yellow"
                diag = (f"[{color}]{state['voltage']:5.2f}V {state['current']:5.2f}A "
                        f"{state['temp_c']:3d}C mode={mode} fault={fault}[/{color}]")
            else:
                diag = "[red]no diagnostics — replying, but not to the 0xAE query[/red]"
            self.log_write(f"   {esc_id:3d} {pos}  {diag}  ({age_s(esc_id)})")

        selected = set(self.drive_ids) | set(self.steer_ids)
        noise = {c: n for c, n in self.bus.snapshot_undecoded().items() if c in selected}
        if noise:
            self.log_write("   [yellow]undecodable frames:[/yellow] " +
                           ", ".join(f"{c}x{n}" for c, n in sorted(noise.items())))

    def action_toggle_candump(self) -> None:
        panel = self.query_one("#candump", RichLog)
        if panel.display:
            if self.candump:
                self.candump.stop()
                self.candump = None
            panel.display = False
            self.log_write("[magenta]1[/magenta] candump panel off")
            return

        panel.display = True
        panel.clear()
        panel.border_title = f"candump {CAN_CHANNEL}"
        monitor = CandumpMonitor(CAN_CHANNEL)
        if not monitor.start():
            panel.write(f"cannot start candump: {monitor.error}")
            self.log_write(f"[red]1: candump failed — {monitor.error}[/red]")
            return
        self.candump = monitor
        panel.write("$ " + " ".join(CANDUMP_CMD + [CAN_CHANNEL]))
        self.log_write("[magenta]1[/magenta] candump panel on")

    def drain_candump(self) -> None:
        if not self.candump:
            return
        lines = self.candump.drain()
        if not lines:
            return
        panel = self.query_one("#candump", RichLog)
        for line in lines:
            panel.write(line)
        if self.candump.dropped:
            panel.border_title = (f"candump {CAN_CHANNEL} — "
                                  f"{self.candump.dropped} dropped")

    def action_measure_rate(self) -> None:
        if not self.motion:
            return
        self.log_write("r: measuring send rate for 1s...")
        self.run_worker(self._measure_rate_worker, thread=True)

    def _measure_rate_worker(self) -> None:
        hz = measure_send_rate(self.motion, duration=1.0)
        self.call_from_thread(
            self.log_write, f"   actual rate ~= {hz:.1f} Hz (target {CMD_RATE_HZ:.0f} Hz)"
        )

    def action_quit_app(self) -> None:
        self.log_write("q: shutting down...")
        self._cleanup_hardware()
        self.exit()

    def _cleanup_hardware(self) -> None:
        if self.candump:
            self.candump.stop()
            self.candump = None
        if self.diag_poller:
            self.diag_poller.shutdown()
        if self.motion:
            self.motion.set_speed_target(0.0)
            self.motion.wait_until_settled(timeout=3.0)
            self.motion.set_active(False)
            for esc_id in self.motion.drive_ids:
                self.bus.send(dm_disable(esc_id))
            for esc_id in self.motion.steer_ids:
                self.bus.send(sw_disable(esc_id))
            self.motion.shutdown()
        if self.bus:
            self.bus.shutdown()
        if self.link_monitor:
            self.link_monitor.shutdown()

# ── Main ────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    drive_ids, steer_ids = resolve_selected_ids(args)
    ChassisTUI(drive_ids, steer_ids).run()

if __name__ == '__main__':
    main()
