"""Keyboard input front-end for ``ServoController`` (see ``servo_controller.py``).

Reads raw keyboard events via evdev and drives a ``ServoController``.
See ``keyboard_teleop_node.py`` for the runnable entry point and the
full keyboard controls reference.
"""

import fcntl
import os
import select
import sys
import termios
import threading

import evdev
from evdev import ecodes

from arm_teleop.servo_controller import SAMPLING_DRILL_MODES_ENABLED


def _list_keyboard_candidates():
    """Return evdev devices that look like QWERTY keyboards (path, name, score)."""
    required = {ecodes.KEY_R, ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_ESC}
    candidates = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
        except (FileNotFoundError, PermissionError, OSError):
            continue
        keys = set(device.capabilities().get(ecodes.EV_KEY, []))
        if not required.issubset(keys):
            continue
        name = device.name or ''
        phys = device.phys or ''
        name_l = name.lower()
        # Skip obvious non-keyboards that still expose a few KEY_* codes.
        if any(bad in name_l for bad in ('sleep', 'lid', 'power', 'video bus', 'hdmi', 'headphone')):
            continue
        score = 0
        if 'usb' in phys or phys.startswith('usb-'):
            score += 100
        if '/input0' in phys:
            score += 20  # main HID collection on multi-interface boards
        if 'keychron' in name_l or 'keyboard' in name_l:
            score += 10
        if 'at translated' in name_l or phys.startswith('isa'):
            score -= 50  # laptop PS/2 — usually wrong when an external KB is plugged in
        score += min(len(keys), 200) / 200.0
        candidates.append((score, path, name, phys))
    candidates.sort(reverse=True)
    return candidates



def _resolve_keyboard_device_path(requested: str) -> str | None:
    """Resolve ``auto`` / empty to the best keyboard path, else return ``requested``."""
    requested = (requested or '').strip()
    if requested and requested.lower() != 'auto':
        return requested
    candidates = _list_keyboard_candidates()
    if not candidates:
        return None
    return candidates[0][1]


HELP = """
╔══════════════════════════════════════════════════╗
║  Keyboard Servo — EEF control                    ║
╠══════════════════════════════════════════════════╣
║  EEF translation (absolute, arm_mount_link):     ║
║    w / s  — +X / -X                              ║
║    a / d  — +Y / -Y                              ║
║    q / e  — +Z / -Z                              ║
║  EEF translation (view-relative, camera):        ║
║    Up/Dn  — forward / back                       ║
║    Lt/Rt  — left / right                         ║
║    t / g  — up / down                            ║
║  EEF rotation (about arm_tcp_link):              ║
║    i / k  — pitch (wx)                           ║
║    u / o  — yaw   (wy)                           ║
║    j / l  — roll  (wz)                           ║
║  Gripper:                                        ║
║    b / v  — open / close                         ║
║  Panel:                                          ║
║    p      — align to detected panel              ║
║    m      — reorient gripper only (p first)      ║
║  Other:                                          ║
║    r      — move to home + start servo           ║
║    f      — level tool (collision-checked; locks ║
║             pitch/yaw after — 'r' unlocks)       ║
║  p/r/f wait 5s (activity indicator) before moving║
║    ESC/x  — exit                                 ║
╚══════════════════════════════════════════════════╝
"""

class KeyboardInputLoop:
    """Reads raw keyboard events via evdev and drives a ``ServoController``.

    Maintains the set of currently pressed direction keys, recomputes the
    combined Cartesian velocity whenever the set changes, and handles the
    special "safe pose" and "exit" key bindings.
    """

    _DIRECTIONS = {
        ecodes.KEY_W: ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_S: (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_A: ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_D: ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_Q: ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_E: ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_I: ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0),  # pitch
        ecodes.KEY_K: ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_U: ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0),  # yaw
        ecodes.KEY_O: ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_J: ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0),  # roll
        ecodes.KEY_L: ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
        ecodes.KEY_UP:    ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  1.0,  0.0,  0.0),
        ecodes.KEY_DOWN:  ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,  0.0,  0.0),
        ecodes.KEY_LEFT:  ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  1.0,  0.0),
        ecodes.KEY_RIGHT: ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, -1.0,  0.0),
        ecodes.KEY_T:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  1.0),
        ecodes.KEY_G:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0, -1.0),
    }

    _PITCH_YAW_KEYS = {ecodes.KEY_I, ecodes.KEY_K, ecodes.KEY_U, ecodes.KEY_O}

    _GRIPPER_KEYS = {
        ecodes.KEY_B: 1.0,   # open
        ecodes.KEY_V: -1.0,  # close
    }

    _KEYSTATE_UP = 0
    _KEYSTATE_DOWN = 1
    _KEYSTATE_REPEAT = 2

    _PANEL_LOST_CONFIRM_SEC = 2.0

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and initialize input state.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from keyboard input.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._gripper_speed = controller.gripper_speed
        self._device_path = controller.keyboard_device_path
        self._lock = threading.Lock()
        self._pressed = set()
        self._gripper_pressed = set()
        self._exit_event = threading.Event()
        self._devices = []
        self._read_thread = None
        self._servo_started = False
        self._safe_pose_running = threading.Lock()
        self._level_running = threading.Lock()
        self._panel_align_running = threading.Lock()
        self._orient_gripper_running = threading.Lock()

        self._panel_was_visible = False
        self._panel_prompt_pending = False
        self._panel_lost_since = None
        self._panel_watch_timer = controller.create_timer(0.2, self._check_panel_visibility)

    def _open_device(self) -> bool:
        """Open evdev keyboard(s) for teleop.

        ``keyboard_device_path:=auto`` (default) opens every QWERTY-capable
        keyboard and merges events. Pinning only ``/dev/input/event3`` (laptop
        AT keyboard) while typing on a Keychron produced zero key events.

        Returns:
            bool: True if at least one device opened.
        """
        requested = (self._device_path or '').strip()
        if requested and requested.lower() != 'auto':
            paths = [requested]
        else:
            paths = [p for _s, p, _n, _ph in _list_keyboard_candidates()]

        if not paths:
            self._controller.get_logger().error(
                'No suitable keyboard found via evdev. Set keyboard_device_path '
                'to an explicit /dev/input/eventN.'
            )
            return False

        opened = []
        for path in paths:
            try:
                device = evdev.InputDevice(path)
                # Older python-evdev has no set_nonblocking(); use fcntl.
                flag = fcntl.fcntl(device.fd, fcntl.F_GETFL)
                fcntl.fcntl(device.fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)
            except (FileNotFoundError, PermissionError, OSError) as e:
                self._controller.get_logger().warn(f'Skipping {path!r}: {e!r}')
                continue
            opened.append(device)
            self._controller.get_logger().info(f'Listening on {path} ({device.name})')

        if not opened:
            self._controller.get_logger().error(f'Could not open any of {paths!r}')
            return False

        self._devices = opened
        self._device_path = opened[0].path
        print(
            '\nKeyboard input:\n  '
            + '\n  '.join(f'{d.name} ({d.path})' for d in opened)
            + '\nPress r = home + start Servo, then WASD to move.\n'
        )
        return True

    def _recompute_velocity(self):
        """Recompute and apply the combined velocity from all pressed keys.

        Sums the per-axis direction vectors of every currently pressed key
        in ``self._pressed``, scales the result by the controller's linear
        and angular speed settings, and forwards it to
        ``ServoController.set_velocity``.
        """
        vx = vy = vz = wx = wy = wz = 0.0
        cvx = cvy = cvz = 0.0
        with self._lock:
            active = list(self._pressed)
        for code in active:
            d = self._DIRECTIONS.get(code)
            if d is None:
                continue
            vx += d[0]
            vy += d[1]
            vz += d[2]
            wx += d[3]
            wy += d[4]
            wz += d[5]
            cvx += d[6]
            cvy += d[7]
            cvz += d[8]
        self._controller.set_velocity(
            vx * self._linear_speed, vy * self._linear_speed, vz * self._linear_speed,
            wx * self._angular_speed, wy * self._angular_speed, wz * self._angular_speed,
            view_vx=cvx * self._linear_speed,
            view_vy=cvy * self._linear_speed,
            view_vz=cvz * self._linear_speed,
        )

    def _recompute_gripper_velocity(self):
        """Recompute and apply gripper velocity from currently pressed b/v."""
        with self._lock:
            active = list(self._gripper_pressed)
        vel = sum(self._GRIPPER_KEYS.get(c, 0.0) for c in active) * self._gripper_speed
        self._controller.set_gripper_velocity(vel)

    def _check_panel_visibility(self):
        """Poll panel visibility and arm the one-shot prompt on a rising edge.

        Runs on a ROS timer (not tied to key events) since the panel can
        appear in frame without the operator pressing anything. Only
        fires the prompt/gate on a *confirmed* False -> True transition —
        'p' stays usable at any time the panel is visible (or a position
        is already remembered) regardless of this flag (see _read_loop),
        so re-detecting an already-visible panel does nothing here.

        "Confirmed" (via _panel_lost_since/_PANEL_LOST_CONFIRM_SEC) means
        is_panel_visible() must have been continuously False for a real
        stretch of time, not just one poll tick — a single marker at a
        marginal angle realistically drops detection for a second or more
        while the operator is actively driving, and reacting to every one
        of those gaps as "the panel left and came back" would re-fire the
        prompt on every flicker.
        """
        now = self._controller.get_clock().now()
        # Jaw-only, same as the P/M key handlers above and GamepadInputLoop's
        # equivalent lockout — suppresses the prompt entirely for other tools.
        raw_visible = (self._controller.end_effector == 'jaw'
                       and self._controller.is_panel_visible())
        if raw_visible:
            self._panel_lost_since = None
            visible = True
        elif self._panel_was_visible:
            if self._panel_lost_since is None:
                self._panel_lost_since = now
            lost_sec = (now - self._panel_lost_since).nanoseconds / 1e9
            visible = lost_sec < self._PANEL_LOST_CONFIRM_SEC
        else:
            visible = False

        if visible and not self._panel_was_visible:
            self._panel_prompt_pending = True
            self._controller.stop()
            print('\n>>> Panel detected! Press P to align to it. <<<')
        self._panel_was_visible = visible

    def _handle_panel_align(self):
        """Run panel alignment and hand control back to the operator either way.

        Unlike _handle_safe_pose, Servo is restarted on failure too: most
        align_to_panel() failures (stale detection, no remembered
        position, planning rejected) never move the arm at all, and even
        the execution-failure path only happens after a real
        collision-checked plan or a deterministic replay — so there's no
        equivalent of move_to_safe_pose()'s "arm may be stopped
        mid-trajectory, don't hand back control blindly" risk. Stranding
        the operator with no teleop just because alignment didn't succeed
        would defeat the point of it being an assistive, not mandatory,
        action.

        Guarded by a non-blocking lock (mirrors _handle_safe_pose) so a
        second 'p' press while an align is already in flight is ignored
        instead of racing a second FollowJointTrajectory/align call
        against the first — confirmed live: nothing here previously
        stopped that, since KEY_P has no already-pressed dedup the way
        direction/gripper keys do.
        """
        if not self._panel_align_running.acquire(blocking=False):
            return
        try:
            print('Aligning to panel...')
            if self._controller.run_planned_activity(self._controller.align_to_panel, 'align_to_panel'):
                print('Panel align succeeded.')
            else:
                print('Panel align failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._panel_align_running.release()

    def _handle_orient_gripper(self):
        """Rotate the gripper in place to face the remembered panel direction."""
        if not self._orient_gripper_running.acquire(blocking=False):
            return
        try:
            print('Orienting gripper toward panel...')
            if self._controller.run_planned_activity(
                    self._controller.orient_gripper_to_panel, 'orient_gripper_to_panel'):
                print('Gripper oriented.')
            else:
                print('Gripper orient failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._orient_gripper_running.release()

    def _handle_safe_pose(self):
        """Clear pressed keys, stop motion, and move to the safe pose.

        Servo is only started if the safe-pose move actually succeeded;
        starting it after a rejected/aborted/timed-out move would let the
        operator resume Cartesian teleop from a pose that never reached the
        intended safe configuration.

        Intended to run in its own thread (spawned from ``_read_loop``) so
        that the blocking safe-pose and servo-start calls do not stall
        keyboard event processing.
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        try:
            with self._lock:
                self._pressed.clear()
                self._gripper_pressed.clear()
            self._controller.stop()
            print('Moving to home...')
            if self._controller.run_planned_activity(self._controller.move_to_safe_pose, 'move_to_safe_pose'):
                if self._exit_event.is_set():
                    print('Exit requested during home move — Servo not started.')
                    return
                print('Starting servo...')
                if self._controller.start_servo():
                    self._servo_started = True
                else:
                    print('Servo failed to start — staying on trajectory controller.')
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_running.release()

    def _handle_level(self):
        """Reorient the tool straight down via a collision-checked plan.

        Servo is restarted regardless of outcome, same reasoning as
        move_to_safe_pose's failure path being the ONE case that does
        not: a rejected/failed plan here never moved the arm from
        wherever it already safely was, so there's no "resumed
        mid-trajectory" risk to guard against.
        """
        if not self._level_running.acquire(blocking=False):
            return
        try:
            print('Leveling tool...')
            if self._controller.run_planned_activity(self._controller.level_tool, 'level_tool'):
                print('Tool leveled.')
            else:
                print('Level move failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._level_running.release()

    def _read_loop(self):
        """Continuously read raw key events from all opened keyboards."""
        try:
            while not self._exit_event.is_set():
                if not self._devices:
                    break
                try:
                    ready, _, _ = select.select(
                        [dev.fd for dev in self._devices], [], [], 0.2
                    )
                except (ValueError, OSError) as e:
                    self._controller.get_logger().error(f'Keyboard select failed: {e!r}')
                    break
                if not ready:
                    continue
                fd_to_dev = {dev.fd: dev for dev in self._devices}
                for fd in ready:
                    device = fd_to_dev.get(fd)
                    if device is None:
                        continue
                    try:
                        for event in device.read():
                            if event.type != ecodes.EV_KEY:
                                continue
                            code, value = event.code, event.value

                            if code in (ecodes.KEY_ESC, ecodes.KEY_X) and value == self._KEYSTATE_DOWN:
                                self._exit_event.set()
                                return

                            if code == ecodes.KEY_R and value == self._KEYSTATE_DOWN:
                                threading.Thread(
                                    target=self._handle_safe_pose, daemon=True
                                ).start()
                                continue

                            if code == ecodes.KEY_F and value == self._KEYSTATE_DOWN:
                                if SAMPLING_DRILL_MODES_ENABLED:
                                    threading.Thread(
                                        target=self._handle_level, daemon=True
                                    ).start()
                                else:
                                    print('level_tool() is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.')
                                continue

                            if code == ecodes.KEY_P and value == self._KEYSTATE_DOWN:
                                if not self._servo_started:
                                    continue
                                # Panel align/orient is jaw-only — only the
                                # jaw gripper physically interacts with the
                                # panel (see GamepadInputLoop's identical
                                # lockout for the reasoning).
                                if self._controller.end_effector != 'jaw':
                                    print(
                                        f"Panel align is locked out with end_effector="
                                        f"'{self._controller.end_effector}' — only the "
                                        'jaw gripper interacts with the panel.'
                                    )
                                    continue
                                self._panel_prompt_pending = False
                                if (self._controller.is_panel_visible()
                                        or self._controller.has_remembered_panel_position):
                                    threading.Thread(
                                        target=self._handle_panel_align, daemon=True
                                    ).start()
                                else:
                                    print('No panel currently in view and no panel position remembered yet.')
                                continue

                            if code == ecodes.KEY_M and value == self._KEYSTATE_DOWN:
                                if not self._servo_started:
                                    continue
                                if self._controller.end_effector != 'jaw':
                                    print(
                                        f"Gripper orient is locked out with end_effector="
                                        f"'{self._controller.end_effector}' — only the "
                                        'jaw gripper interacts with the panel.'
                                    )
                                    continue
                                if self._controller.has_remembered_panel_position:
                                    threading.Thread(
                                        target=self._handle_orient_gripper, daemon=True
                                    ).start()
                                else:
                                    print('No panel position remembered yet — align (p) first.')
                                continue

                            if code in self._GRIPPER_KEYS:
                                if not self._servo_started:
                                    continue
                                if self._panel_prompt_pending and value == self._KEYSTATE_DOWN:
                                    self._panel_prompt_pending = False
                                    print('Continuing manual control (panel align not triggered).')
                                if value == self._KEYSTATE_DOWN:
                                    with self._lock:
                                        already_pressed = code in self._gripper_pressed
                                        other = (ecodes.KEY_V if code == ecodes.KEY_B
                                                 else ecodes.KEY_B)
                                        self._gripper_pressed.discard(other)
                                        self._gripper_pressed.add(code)
                                    if not already_pressed:
                                        self._recompute_gripper_velocity()
                                        key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                        print(f'{key_name} gripper_vel={self._controller.gripper_vel:.4f}')
                                elif value == self._KEYSTATE_UP:
                                    with self._lock:
                                        self._gripper_pressed.discard(code)
                                    self._recompute_gripper_velocity()
                                continue

                            if code not in self._DIRECTIONS:
                                continue

                            if not self._servo_started:
                                continue

                            if self._panel_prompt_pending and value == self._KEYSTATE_DOWN:
                                self._panel_prompt_pending = False
                                print('Continuing manual control (panel align not triggered).')

                            if value == self._KEYSTATE_DOWN:
                                with self._lock:
                                    already_pressed = code in self._pressed
                                    self._pressed.add(code)
                                if not already_pressed:
                                    self._recompute_velocity()
                                    if not (self._controller._pitch_yaw_locked
                                            and code in self._PITCH_YAW_KEYS):
                                        key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                        # view_* included or the arrow keys would
                                        # report all-zero and look like a no-op.
                                        print(
                                            f'{key_name} vx={self._controller.vx:.2f} '
                                            f'vy={self._controller.vy:.2f} '
                                            f'vz={self._controller.vz:.2f} '
                                            f'wx={self._controller.wx:.2f} '
                                            f'wy={self._controller.wy:.2f} '
                                            f'wz={self._controller.wz:.2f} '
                                            f'| fwd={self._controller.view_vx:.2f} '
                                            f'left={self._controller.view_vy:.2f} '
                                            f'up={self._controller.view_vz:.2f}'
                                        )
                            elif value == self._KEYSTATE_UP:
                                with self._lock:
                                    self._pressed.discard(code)
                                self._recompute_velocity()
                    except BlockingIOError:
                        continue
                    except OSError as e:
                        self._controller.get_logger().warn(
                            f'Lost keyboard {device.path}: {e!r}'
                        )
                        self._devices = [d for d in self._devices if d.fd != fd]
                        if not self._devices:
                            raise
        except OSError as e:
            self._controller.get_logger().error(f'Keyboard read loop failed: {e!r}')
        finally:
            self._exit_event.set()

    def run(self):
        """Open the keyboard device and run the input loop until exit.

        Opens the evdev device (returning early if this fails), prints the
        help banner, disables terminal echo where possible, starts the
        event-reading loop in a daemon thread, and blocks until the exit
        event is set. On exit, stops the controller's motion and restores
        the original terminal settings.
        """
        if not self._open_device():
            return
        print(HELP, flush=True)
        self._controller.get_logger().info(
            'Keyboard teleop ready. Press r = home + Servo, then WASD. '
            'Do not start a second keyboard_teleop_node.'
        )

        # ros2 launch often has no TTY; fileno()/tcgetattr would abort the node.
        old_term_settings = None
        stdin_fd = None
        try:
            stdin_fd = sys.stdin.fileno()
        except (AttributeError, ValueError, OSError):
            stdin_fd = None
        if stdin_fd is not None:
            try:
                old_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings[3] &= ~termios.ECHO
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, new_term_settings)
            except (termios.error, OSError):
                old_term_settings = None

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            with self._safe_pose_running:
                pass
            self._controller.stop()
            if old_term_settings is not None:
                termios.tcflush(stdin_fd, termios.TCIFLUSH)
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)


