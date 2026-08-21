#!/usr/bin/env python3
"""
chassis_driver test — standalone TUI, NO ROS2 required.

Talks directly to the CAN bus via python-can / SocketCAN. Uses Textual for
a proper terminal UI: a fixed status panel at the top (live speed/angle/
active state) and a scrolling event log below it showing every keypress
and action as it happens.

Install dependencies:
    pip install python-can textual --break-system-packages

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
    f   Print latest decoded feedback
    r   Measure actual command send rate
    q   Quit
"""
import argparse
import math
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
    print(f"    pip install {pkg} --break-system-packages")
    sys.exit(1)

# ── Config ──────────────────────────────────────────────────────────────

CAN_CHANNEL = "can0"
CAN_BITRATE = 500_000             # must match what's actually configured on the bus

STEER_IDS = [11, 13, 15, 17]      # Steadywin, [FL, FR, RL, RR]
DRIVE_IDS = [10, 12, 14, 16]      # Damiao,    [FL, FR, RL, RR]

MAX_ACCEL = 1.0          # rad/s^2, applied when |target| > |current| (speeding up)
MAX_DECEL = 2.0          # rad/s^2, applied when |target| < |current| (slowing down)
CMD_RATE_HZ = 10.0       # command loop frequency — see MotionController

SPEED_STEP = 0.1          # rad/s added/removed per 'k'/'j' keypress
ANGLE_STEP_DEG = 5.0      # degrees added/removed per 'l'/'h' keypress

STALE_THRESHOLD_S = 1.0   # motor ID shown red if no feedback received within this long

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

def decode_feedback(msg: can.Message) -> dict | None:
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

# ── Bus wrapper with background listener ─────────────────────────────────────

class CanBus:
    def __init__(self, channel: str, bitrate: int):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=bitrate)
        self.latest_feedback: dict[int, dict] = {}
        self.last_seen: dict[int, float] = {}   # can_id -> time.monotonic() of last feedback
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
                    self.last_seen[msg.arbitration_id] = time.monotonic()

    def send(self, msg: can.Message):
        self.bus.send(msg)

    def snapshot(self) -> dict[int, dict]:
        with self._lock:
            return dict(self.latest_feedback)

    def snapshot_ages(self) -> dict[int, float]:
        """seconds since feedback was last received, per can_id — empty if never seen"""
        now = time.monotonic()
        with self._lock:
            return {can_id: now - ts for can_id, ts in self.last_seen.items()}

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()

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
    #status_row {
        height: 5;
    }
    #can_status {
        width: 2fr;
        border: solid green;
        padding: 0 1;
        content-align: left middle;
    }
    #motors_status {
        width: 1fr;
        border: solid yellow;
        padding: 0 1;
        content-align: left middle;
    }
    #motion_status {
        width: 1fr;
        border: solid cyan;
        padding: 0 1;
        content-align: left middle;
    }
    #log {
        border: solid #4a4a4a;
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
        Binding("q", "quit_app", "Quit"),
    ]

    def __init__(self, drive_ids: list[int], steer_ids: list[int]):
        super().__init__()
        self.drive_ids = drive_ids
        self.steer_ids = steer_ids
        self.bus: CanBus | None = None
        self.motion: MotionController | None = None
        self.link_monitor: CanLinkMonitor | None = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal(id="status_row"):
            yield Static(id="can_status")
            yield Static(id="motors_status")
            yield Static(id="motion_status")
        yield RichLog(id="log", wrap=True, highlight=True, markup=True)
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
            self.bus = CanBus(CAN_CHANNEL, CAN_BITRATE)
            self.motion = MotionController(self.bus, self.drive_ids, self.steer_ids)
            log.write("[green]Ready.[/green]")
        except Exception as e:
            log.write(f"[bold red]Failed to open CAN bus: {e}[/bold red]")

        self.set_interval(1 / 10, self.update_status)

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

        # ── middle panel: per-motor health (green = fresh feedback, red = stale/none) ──
        ages = self.bus.snapshot_ages() if self.bus else {}

        def fmt_id(esc_id: int) -> str:
            age = ages.get(esc_id)
            ok = age is not None and age < STALE_THRESHOLD_S
            color = "green" if ok else "red"
            return f"[{color}]{esc_id}[/{color}]"

        drive_line = "D: " + (" ".join(fmt_id(i) for i in self.drive_ids) if self.drive_ids else "(none)")
        steer_line = "S: " + (" ".join(fmt_id(i) for i in self.steer_ids) if self.steer_ids else "(none)")
        self.query_one("#motors_status", Static).update(f"{drive_line}\n{steer_line}")

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
        snap = self.bus.snapshot()
        if not snap:
            self.log_write("f: (no feedback frames received yet)")
            return
        self.log_write(f"f: feedback [{len(snap)} motor(s)]")
        for can_id, fb in sorted(snap.items()):
            err_name = ERR_NAMES.get(fb["err"], f"0x{fb['err']:X}")
            self.log_write(
                f"   0x{can_id:03X} ctrl={fb['ctrl_id']:2d} status={err_name:12s} "
                f"pos={fb['pos_raw']:5d} vel={fb['vel_raw']:4d} torque={fb['torque_raw']:4d} "
                f"T_mos={fb['t_mos_c']}C T_rotor={fb['t_rotor_c']}C"
            )

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
