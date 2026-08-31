"""Gamepad input front-end for ``ServoController`` (see ``servo_controller.py``).

Reads sensor_msgs/Joy messages and drives a ``ServoController``. See
``gamepad_teleop_node.py`` for the runnable entry point and the full
gamepad controls reference.
"""

import functools
import threading

from sensor_msgs.msg import Joy

from arm_teleop.servo_controller import SAMPLING_DRILL_MODES_ENABLED
from arm_teleop.end_effector_client import EndEffectorClient


GAMEPAD_HELP = """
╔══════════════════════════════════════════════╗
║  Gamepad — EEF control (view-relative)       ║
╠══════════════════════════════════════════════╣
║  Left stick   ←→  — left / right  (camera)   ║
║               ↑↓  — forward / back (camera)  ║
║  Right stick  ↑↓  — up / down     (camera)   ║
║               ←→  — yaw   (TCP)              ║
║  R1 + right   ↑↓  — pitch (TCP)              ║
║               ←→  — roll  (TCP)              ║
║  9 (button)       — push boost (hold)        ║
║  11 (button)      — jaw: gripper OPEN        ║
║                     drill_sampling: claw     ║
║                     OPEN, or drill UP in     ║
║                     drill mode               ║
║  13 (button)      — jaw: gripper CLOSE       ║
║                     drill_sampling: claw     ║
║                     CLOSE, or drill DOWN in  ║
║                     drill mode               ║
║  12 (button)      — drill_sampling: LOCK     ║
║  14 (button)      — drill_sampling: UNLOCK   ║
║   (SAFE gripper / claw+drill+lock firmware,  ║
║   over CAN — A/B/Y below own sampling/drill  ║
║   mode entry)                                ║
║   drill: right ←→ fwd/back, right ↑↓         ║
║   left/right, left ↑↓ up/down, no roll       ║
║   sampling: right ↑↓/←→ inverted, left ←→    ║
║   roll                                       ║
║   (pitch/yaw locked in both modes)           ║
║  6 (button)       — point tool straight down ║
║   (collision-checked; sampling/drill modes)  ║
║  A                — jaw/astrobio: home        ║
║                     drill_sampling: sampling_ ║
║                     home. Also exits drill    ║
║                     mode back to sampling —   ║
║                     but only from drill_home  ║
║  B                — jaw/astrobio: locked out ║
║                     drill_sampling: drill_    ║
║                     home. Enters drill mode   ║
║                     from sampling — but only  ║
║                     from sampling_home. Once  ║
║                     already in drill mode,    ║
║                     freely returns to it      ║
║  Y                — jaw/astrobio: locked out ║
║                     drill_sampling: context-  ║
║                     dependent — from          ║
║                     sampling_home -> sampling_║
║                     container; from drill_home║
║                     -> drill_container; else  ║
║                     refused (needs one first) ║
║  Button 7          — align to panel          ║
║  Button 8          — reorient gripper only   ║
║  A/B/Y/7/8: 5s indicator wait before move    ║
║  X                — exit                     ║
╚══════════════════════════════════════════════╝
"""

class GamepadInputLoop:
    """Reads sensor_msgs/Joy messages and drives a ``ServoController``.

    Replaces the raw-keyboard evdev input of ``KeyboardInputLoop`` with a
    subscription to the ``joy`` package's ``/joy`` topic — as this module's
    docstring already promises, ``ServoController`` itself needs no changes.

    Launch via ``arm_teleop/launch/gamepad.launch.py``, which starts
    ``game_controller_node`` (not plain ``joy_node``): it maps raw HID
    reports through SDL's GameController DB into a fixed canonical index
    order (A=0, X=2, LEFTSHOULDER=9, RIGHTSHOULDER=10, DPAD_UP=11,
    DPAD_LEFT=13, ... — same on every machine/controller SDL recognizes),
    instead of joy_node's raw per-device layout, which was observed to
    shift index depending on kernel/Bluetooth stack even for the identical
    physical pad. The indices below match that canonical table directly.

    Trigger axes (4/5) still get a per-axis rest sample while sticks are
    centered (``_trigger_amount``) as a safety net — GameController's
    TRIGGERLEFT/TRIGGERRIGHT are specified to rest at 0.0, but this costs
    nothing if that already holds.
    """

    # Gamepad translation is view-relative (camera frame), not mount-frame —
    # the operator is looking through the camera, so "forward" should follow
    # it. Mount-frame XYZ stays available on the keyboard.
    AXIS_LEFT_X = 0     # view +Y / -Y  (left / right)
    AXIS_LEFT_Y = 1     # view +X / -X  (forward / back)
    AXIS_RIGHT_X = 2    # yaw (-wy)     — roll  (-wz) while R1 held
    AXIS_RIGHT_Y = 3    # view +Z / -Z  — pitch (+wx) while R1 held
    AXIS_L2 = 4         # unmapped; used only for trigger rest calibration
    AXIS_R2 = 5

    BUTTON_SAFE_POSE = 0   # 'A' — move to home + start servo
    # 'B' — force sampling mode on and go straight to sampling_home.
    # One-shot, not a toggle — there is no button that turns it back off
    # short of restarting with a different end_effector.
    BUTTON_SAMPLING_HOME = 1
    BUTTON_EXIT = 2        # 'X' — exit
    # 'Y' — force drill mode on and go straight to drill_home. One-shot,
    # mirrors BUTTON_SAMPLING_HOME above.
    BUTTON_DRILL_HOME = 3
    BUTTON_LB = 4          # unmapped (settle check only)
    # LEFTSHOULDER/L1. Held to scale up commanded velocity — raises the
    # per-cycle position step Servo re-anchors from, which is what caps
    # static push force (not kp/kd).
    BUTTON_PUSH_BOOST = 9
    # Bounded by max_cmd_speed_rad_s (arm_macro.xacro): the hardware
    # interface rate-limits actual motion to that many rad/s regardless
    # of what Servo asks for, so beyond a certain multiplier this stops
    # helping and max_cmd_speed_rad_s becomes the real ceiling instead.
    PUSH_BOOST_MULTIPLIER = 3.0
    # Shift button has no class constant — parameterized (DEFAULT_GAMEPAD_SHIFT_BUTTON),
    # read via self._shift_button, in case a pad's SDL mapping is ever wrong.

    BUTTON_GRIPPER_OPEN = 11
    BUTTON_GRIPPER_CLOSE = 13
    # DPAD_DOWN / DPAD_RIGHT — drill_sampling tool's electric lock
    # (EndEffectorClient.send('lock'/'unlock')). No jaw/astrobio
    # equivalent, so these are no-ops unless end_effector == 'drill_sampling'.
    BUTTON_LOCK = 12
    BUTTON_UNLOCK = 14
    # On-demand collision-checked "point straight down now" (mirrors
    # KeyboardInputLoop's 'f') — see ServoController.level_tool() for why
    # this needs to be its own move_group plan, not just a harder nudge
    # from _level_hold. Index unverified — see the comment above.
    BUTTON_LEVEL = 6
    # NOT 12/14 — those are BUTTON_LOCK/BUTTON_UNLOCK above, gated on
    # end_effector=='drill_sampling'. panel_align/orient_gripper have no
    # such gate (any end_effector, whenever a panel is visible/remembered),
    # so sharing an index with LOCK/UNLOCK let one button press fire both
    # a drill command and an arm motion at once (review-flagged). L3/R3
    # (stick clicks) are otherwise unused.
    BUTTON_PANEL_ALIGN = 7      # 'p' equivalent — align to detected panel
    BUTTON_ORIENT_GRIPPER = 8   # 'm' equivalent — reorient gripper only

    _DEADZONE = 0.2
    _JOY_TIMEOUT_SEC = 0.2
    _WATCHDOG_PERIOD_SEC = 0.1
    # See KeyboardInputLoop's identically-named constant.
    _PANEL_LOST_CONFIRM_SEC = 2.0

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and subscribe to ``/joy``.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from gamepad input. Its own
                ``create_subscription`` is reused for the ``/joy`` topic
                so the callback runs on the node's existing executor —
                no separate thread is needed, unlike the keyboard's
                blocking evdev read loop.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._shift_button = controller.gamepad_shift_button
        # Mirrors ServoController's own _sampling_mode/_drill_mode — needed
        # here too since they decide how the right stick maps to wx/wy/wz/view_vz.
        self._sampling_mode = False
        self._drill_mode = False
        # True only right after a confirmed, successful arrival at
        # sampling_home/drill_home respectively — distinct from
        # _sampling_mode/_drill_mode, which stay True while at
        # sampling_container/drill_container too (sub-positions within
        # that mode). B (drill_home) and Y (sampling_container/
        # drill_container) gate on THESE, not _sampling_mode/_drill_mode,
        # so no hazardous pose is reachable from any of the others —
        # only from its own hub. Cleared by any real stick input (see
        # _on_joy) so driving away from the hub with the sticks also
        # revokes it. Mutually exclusive in practice (see _handle_safe_pose).
        self._at_sampling_home = False
        self._at_drill_home = False
        self._exit_event = threading.Event()
        self._prev_buttons = None
        self._safe_pose_running = threading.Lock()
        self._panel_align_running = threading.Lock()
        self._safe_pose_active = False
        self._level_running = threading.Lock()
        self._level_active = False
        self._orient_gripper_running = threading.Lock()
        self._orient_gripper_active = False
        self._prev_cmd = (0.0,) * 6

        self._last_joy_time = None
        self._joy_silent = False
        self._teleop_locked = True

        self._joy_settling = False
        # Per-trigger rest samples (axis index -> float). None until the
        # first centered settle so we do not assume both are +1.0.
        self._trigger_rest = {}

        self._panel_was_visible = False
        self._panel_prompt_pending = False
        self._panel_lost_since = None

        self._sub = controller.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._watchdog_timer = controller.create_timer(
            self._WATCHDOG_PERIOD_SEC, self._check_joy_timeout
        )

        self._ee_client = EndEffectorClient(controller)

    @classmethod
    def _deadzone(cls, value: float) -> float:
        """Zero out small stick values so resting drift doesn't creep the arm."""
        return 0.0 if abs(value) < cls._DEADZONE else value

    def _axis(self, axes, index: int) -> float:
        """Return ``axes[index]`` with deadzone applied, or 0.0 if out of range."""
        if index >= len(axes):
            return 0.0
        return self._deadzone(axes[index])

    def _calibrate_triggers(self, axes) -> None:
        """Record L2/R2 rest values from a centered Joy snapshot."""
        for index in (self.AXIS_L2, self.AXIS_R2):
            if index < len(axes):
                self._trigger_rest[index] = float(axes[index])
        self._controller.get_logger().info(
            f'Trigger rest L2={self._trigger_rest.get(self.AXIS_L2, float("nan")):.2f} '
            f'R2={self._trigger_rest.get(self.AXIS_R2, float("nan")):.2f}'
        )

    def _sticks_centered(self, axes) -> bool:
        """True when both sticks are inside the deadzone (triggers ignored)."""
        return all(
            self._axis(axes, i) == 0.0
            for i in (self.AXIS_LEFT_X, self.AXIS_LEFT_Y,
                      self.AXIS_RIGHT_X, self.AXIS_RIGHT_Y)
        )

    def _trigger_amount(self, axes, index: int) -> float:
        """Return how far a trigger (L2/R2) is pressed: 0.0 .. 1.0.

        Supports both common rest conventions on this Stadia over ``joy_node``:
        rest near +1 (press toward -1) and rest near 0 (press toward ±1).
        Before calibration, only the +1-rest formula is used, and a raw
        value near 0 is treated as released — otherwise an uncalibrated
        0-rest R2 looks like a constant half-press (wz ≈ 0.5 * angular).
        """
        if index >= len(axes):
            return 0.0
        raw = float(axes[index])
        rest = self._trigger_rest.get(index)

        if rest is None:
            # Uncalibrated: never treat raw≈0 as a half-press (0-rest R2
            # phantom). Full +1-rest presses still register via raw≤-0.5.
            if raw >= 0.5 or raw <= -0.5:
                amount = (1.0 - raw) / 2.0
            else:
                amount = 0.0
        elif rest > 0.5:
            # Classic: +1 released → -1 fully pressed.
            amount = (rest - raw) / (rest - (-1.0))
        else:
            # Rest near 0: any deflection toward ±1 is a press.
            amount = abs(raw - rest)

        if amount < 0.0:
            amount = 0.0
        elif amount > 1.0:
            amount = 1.0
        return 0.0 if amount < self._DEADZONE else amount

    def _button_pressed(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` is currently held down."""
        return index < len(buttons) and buttons[index] == 1

    def _button_rising_edge(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` was just pressed this message.

        Requires a real previous reading — see ``_prev_buttons`` in
        ``__init__`` for why ``None`` (no baseline yet) always reports
        no edge rather than comparing against an assumed all-zero state.
        """
        if self._prev_buttons is None:
            return False
        was_pressed = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        return self._button_pressed(buttons, index) and not was_pressed

    @staticmethod
    def _active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift: bool) -> str:
        """Describe which physical control(s) are driving a nonzero command."""
        # wy (yaw) only ever comes from the plain right stick, wx/wz (pitch/
        # roll) only from the shifted one — so the axes identify the source.
        parts = []
        if view_vx or view_vy:
            parts.append('left stick')
        if view_vz or wy:
            parts.append('right stick')
        if wx or wz:
            parts.append('R1+right stick')
        return '+'.join(parts) if parts else ('R1' if shift else 'idle')

    def _check_joy_timeout(self):
        """Stop the arm if no ``/joy`` message has arrived recently.

        Runs on the node's own timer, independent of message arrival, so
        a disconnected controller or a dead ``joy_node`` can't
        leave the last commanded velocity republishing forever — Servo's
        own command timeout does not help here because ServoController
        keeps re-publishing that last twist every tick regardless of
        whether new input has arrived. This only zeroes the current
        command; it does not lock teleop out, so control resumes on its
        own the moment ``/joy`` messages start arriving again (e.g. a
        Bluetooth reconnect) — no separate re-arm step.
        """
        if self._last_joy_time is None:
            return
        elapsed = (self._controller.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if elapsed > self._JOY_TIMEOUT_SEC:
            if not self._joy_silent:
                self._joy_silent = True
                self._joy_settling = True
                self._trigger_rest.clear()
                self._prev_buttons = None
                self._controller.get_logger().warn(
                    f'/joy timed out after {elapsed:.2f}s — stopping arm.'
                )
            self._controller.stop()

    def _log_raw_joy(self, axes, buttons, note: str = ''):
        """Log the full Joy message — an out-of-range index fails silently otherwise."""
        axes_str = ', '.join(f'{i}:{v:+.2f}' for i, v in enumerate(axes))
        buttons_str = ', '.join(f'{i}:{b}' for i, b in enumerate(buttons))
        self._controller.get_logger().info(
            f'/joy raw{note} — axes[{len(axes)}]: {{{axes_str}}}  '
            f'buttons[{len(buttons)}]: {{{buttons_str}}}'
        )

    def _on_joy(self, msg: Joy):
        """Translate one Joy snapshot into a velocity command and edge-triggered actions."""
        axes = msg.axes
        buttons = msg.buttons

        if self._joy_silent:
            self._joy_silent = False
            self._controller.get_logger().info('/joy resumed.')
        self._last_joy_time = self._controller.get_clock().now()

        if self._prev_buttons is None:
            # First message (also re-fires after a /joy dropout). Warn early
            # if gamepad_shift_button is out of range for this controller.
            self._log_raw_joy(axes, buttons, ' (first message)')
            if self._shift_button >= len(buttons):
                self._controller.get_logger().warn(
                    f'gamepad_shift_button={self._shift_button} but this '
                    f'/joy only reports {len(buttons)} button(s) '
                    f'(0..{len(buttons) - 1}) — that index can never be '
                    f'pressed. Pick a real index from the buttons[] list above.'
                )

        safe_pose_pressed = self._button_rising_edge(buttons, self.BUTTON_SAFE_POSE)
        sampling_home_pressed = self._button_rising_edge(buttons, self.BUTTON_SAMPLING_HOME)
        drill_home_pressed = self._button_rising_edge(buttons, self.BUTTON_DRILL_HOME)
        exit_pressed = self._button_rising_edge(buttons, self.BUTTON_EXIT)
        gripper_open_pressed = self._button_rising_edge(buttons, self.BUTTON_GRIPPER_OPEN)
        gripper_close_pressed = self._button_rising_edge(buttons, self.BUTTON_GRIPPER_CLOSE)
        lock_pressed = self._button_rising_edge(buttons, self.BUTTON_LOCK)
        unlock_pressed = self._button_rising_edge(buttons, self.BUTTON_UNLOCK)
        level_pressed = self._button_rising_edge(buttons, self.BUTTON_LEVEL)
        panel_align_pressed = self._button_rising_edge(buttons, self.BUTTON_PANEL_ALIGN)
        orient_gripper_pressed = self._button_rising_edge(buttons, self.BUTTON_ORIENT_GRIPPER)

        # Log the raw state on any button change, not just the mapped ones,
        # so an unmapped shift button still shows up.
        if self._prev_buttons is not None:
            width = max(len(buttons), len(self._prev_buttons))
            changed = [
                i for i in range(width)
                if self._button_pressed(buttons, i)
                != (i < len(self._prev_buttons) and self._prev_buttons[i] == 1)
            ]
            if changed:
                self._log_raw_joy(
                    axes, buttons,
                    f' (button(s) {changed} changed; shift configured as '
                    f'{self._shift_button})',
                )

        self._prev_buttons = list(buttons)

        if exit_pressed:
            self._exit_event.set()

        end_effector = self._controller.end_effector

        if gripper_open_pressed:
            if end_effector == 'drill_sampling':
                if self._drill_mode:
                    self._ee_client.send('drill_up')
                    self._controller.get_logger().info('Drill: UP sent.')
                else:
                    self._ee_client.send('open')
                    self._controller.get_logger().info('Claw: OPEN sent.')
            else:
                self._ee_client.send('open')
                self._controller.set_gripper_target(self._controller.gripper_stroke)
                self._controller.get_logger().info('Gripper: SAFE_OPEN sent.')

        if gripper_close_pressed:
            if end_effector == 'drill_sampling':
                if self._drill_mode:
                    self._ee_client.send('drill_down')
                    self._controller.get_logger().info('Drill: DOWN sent.')
                else:
                    self._ee_client.send('close')
                    self._controller.get_logger().info('Claw: CLOSE sent.')
            else:
                self._ee_client.send('close')
                self._controller.set_gripper_target(0.0)
                self._controller.get_logger().info('Gripper: SAFE_CLOSE sent.')

        if lock_pressed and end_effector == 'drill_sampling':
            self._ee_client.send('lock')
            self._controller.get_logger().info('Claw/drill: LOCK sent.')

        if unlock_pressed and end_effector == 'drill_sampling':
            self._ee_client.send('unlock')
            self._controller.get_logger().info('Claw/drill: UNLOCK sent.')

        # drill_sampling remaps A/B/Y from their jaw/astrobio meaning, as
        # two hubs (sampling_home, drill_home) each with their own
        # container spoke — sampling is the default/start mode:
        #   A -> sampling_home. Freely available UNLESS currently in
        #        drill_mode, in which case it also EXITS drill mode back to
        #        sampling — but only from drill_home (mirrors B below).
        #   B -> drill_home. Freely available once ALREADY in drill_mode
        #        (e.g. returning from drill_container) — but if still in
        #        sampling_mode, this is what ENTERS drill mode, and that
        #        entry is only allowed from sampling_home.
        #   Y -> sampling_container / drill_container, context-dependent
        #        on which hub is currently confirmed (see its own block).
        # Mode itself is NOT flipped on button press — only once the move
        # actually succeeds (see _handle_safe_pose's target_mode handling).
        # Flipping it immediately would change axis mapping/collision
        # assumptions for a physical configuration the arm may never reach
        # (review-flagged: a rejected/failed move used to leave software
        # mode and physical pose out of sync).
        if safe_pose_pressed and not SAMPLING_DRILL_MODES_ENABLED and end_effector == 'drill_sampling':
            self._controller.get_logger().warn(
                'Sampling home is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif safe_pose_pressed and end_effector == 'drill_sampling' and self._drill_mode and not self._at_drill_home:
            # Exiting drill mode requires being confirmed at ITS hub first —
            # same reasoning as entering it (see sampling_home_pressed's
            # guard below), just mirrored for the other direction.
            self._controller.get_logger().warn(
                'A (sampling_home) requires drill_home first — press B.'
            )
        elif safe_pose_pressed and end_effector == 'drill_sampling':
            self._controller.get_logger().info(
                'A pressed — going straight to sampling_home.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('sampling',), daemon=True
            ).start()
        elif safe_pose_pressed:
            threading.Thread(target=self._handle_safe_pose, daemon=True).start()

        if sampling_home_pressed and not SAMPLING_DRILL_MODES_ENABLED:
            self._controller.get_logger().warn(
                'Drill home is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif sampling_home_pressed and end_effector in ('jaw', 'astrobio'):
            self._controller.get_logger().warn(
                f"B (drill_home) is locked out with end_effector='{end_effector}' "
                '— no drill/sampling tool mounted.'
            )
        elif sampling_home_pressed and not self._drill_mode and not self._at_sampling_home:
            # Not in drill mode yet, so this press would ENTER it — only
            # allowed from the known-safe sampling_home hub
            # (_at_sampling_home, not _sampling_mode: that flag alone stays
            # True at sampling_container too, which previously let
            # drill_home be reached from there). Once already in drill_mode,
            # this same button just returns to drill_home from anywhere in
            # that mode (e.g. drill_container) — no extra gate needed, see
            # the plain elif below.
            self._controller.get_logger().warn(
                'B (drill_home) requires sampling_home first — press A.'
            )
        elif sampling_home_pressed:
            self._controller.get_logger().info(
                'B pressed — going straight to drill_home.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('drill',), daemon=True
            ).start()

        # Y is context-dependent: from sampling_home -> sampling_container,
        # from drill_home -> drill_container. Neither hub, or having driven
        # away from one with the sticks since arriving (see _on_joy) -> refuse.
        if drill_home_pressed and not SAMPLING_DRILL_MODES_ENABLED:
            self._controller.get_logger().warn(
                'sampling_container/drill_container is disabled '
                '(SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif drill_home_pressed and end_effector in ('jaw', 'astrobio'):
            self._controller.get_logger().warn(
                f"Y (sampling_container/drill_container) is locked out with "
                f"end_effector='{end_effector}' — no drill/sampling tool mounted."
            )
        elif drill_home_pressed and self._at_sampling_home:
            # sampling_container is a sub-position within the current
            # sampling context, not a mode-engage move — target_mode=
            # 'sampling_container' leaves _sampling_mode/_drill_mode
            # untouched on success, but does clear _at_sampling_home (see
            # _handle_safe_pose) since we've left the hub.
            self._controller.get_logger().info(
                'Y pressed — going straight to sampling_container.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('sampling_container',), daemon=True
            ).start()
        elif drill_home_pressed and self._at_drill_home:
            self._controller.get_logger().info(
                'Y pressed — going straight to drill_container.'
            )
            threading.Thread(
                target=self._handle_safe_pose, args=('drill_container',), daemon=True
            ).start()
        elif drill_home_pressed:
            self._controller.get_logger().warn(
                'Y (sampling_container/drill_container) requires sampling_home '
                '(press A) or drill_home (press B) first.'
            )

        if level_pressed and not SAMPLING_DRILL_MODES_ENABLED:
            self._controller.get_logger().warn(
                'level_tool() is disabled (SAMPLING_DRILL_MODES_ENABLED=False) — ignored.'
            )
        elif level_pressed:
            threading.Thread(target=self._handle_level, daemon=True).start()

        if self._teleop_locked or self._safe_pose_active or self._level_active or self._orient_gripper_active:
            self._controller.stop()
            return

        if self._joy_settling:
            centered = (
                self._sticks_centered(axes)
                and self._trigger_amount(axes, self.AXIS_L2) == 0.0
                and self._trigger_amount(axes, self.AXIS_R2) == 0.0
                and not self._button_pressed(buttons, self.BUTTON_PUSH_BOOST)
                and not self._button_pressed(buttons, self._shift_button)
            )
            self._controller.stop()
            if centered:
                self._calibrate_triggers(axes)
                self._joy_settling = False
                self._controller.get_logger().info('Sticks centered — resuming control.')
            return

        if not self._trigger_rest and self._sticks_centered(axes):
            self._calibrate_triggers(axes)

        # Held BUTTON_PUSH_BOOST scales up the commanded velocity — see its
        # comment for why that's what actually raises the arm's static
        # push force against resistance, not kp/kd/gravity-ff.
        boost = (self.PUSH_BOOST_MULTIPLIER
                 if self._button_pressed(buttons, self.BUTTON_PUSH_BOOST) else 1.0)
        linear_speed = self._linear_speed * boost
        angular_speed = self._angular_speed * boost

        now = self._controller.get_clock().now()
        # Panel detection/prompting is jaw-only — see panel_align_pressed's
        # own lockout below for why (only the jaw gripper interacts with
        # the panel). Suppressed entirely for other tools, or a
        # drill/astrobio operator would get an unhelpful "press button 7"
        # prompt (and an unwanted stop()) for a feature that just refuses.
        raw_panel_visible = end_effector == 'jaw' and self._controller.is_panel_visible()
        if raw_panel_visible:
            self._panel_lost_since = None
            panel_visible = True
        elif self._panel_was_visible:
            if self._panel_lost_since is None:
                self._panel_lost_since = now
            panel_visible = (now - self._panel_lost_since).nanoseconds / 1e9 < self._PANEL_LOST_CONFIRM_SEC
        else:
            panel_visible = False

        if panel_visible and not self._panel_was_visible:
            self._panel_prompt_pending = True
            self._controller.stop()
            print('\n>>> Panel detected! Press button 7 to align to it. <<<')
        self._panel_was_visible = panel_visible

        if panel_align_pressed and end_effector != 'jaw':
            self._controller.get_logger().warn(
                f"Panel align is locked out with end_effector='{end_effector}' "
                '— only the jaw gripper interacts with the panel.'
            )
        elif panel_align_pressed:
            self._panel_prompt_pending = False
            if panel_visible or self._controller.has_remembered_panel_position:
                threading.Thread(target=self._handle_panel_align, daemon=True).start()
            else:
                print('No panel currently in view and no panel position remembered yet.')

        if orient_gripper_pressed and end_effector != 'jaw':
            self._controller.get_logger().warn(
                f"Gripper orient is locked out with end_effector='{end_effector}' "
                '— only the jaw gripper interacts with the panel.'
            )
        elif orient_gripper_pressed:
            if self._controller.has_remembered_panel_position:
                threading.Thread(target=self._handle_orient_gripper, daemon=True).start()
            else:
                print('No panel position remembered yet — align (button 7) first.')

        if self._panel_prompt_pending:
            # Same "any real stick input dismisses it" rule as
            # KeyboardInputLoop — checked on just the 4 sticks, since the
            # gripper/panel buttons aren't "driving".
            any_axis_active = (
                self._axis(axes, self.AXIS_LEFT_X) != 0.0
                or self._axis(axes, self.AXIS_LEFT_Y) != 0.0
                or self._axis(axes, self.AXIS_RIGHT_X) != 0.0
                or self._axis(axes, self.AXIS_RIGHT_Y) != 0.0
            )
            if any_axis_active:
                self._panel_prompt_pending = False
                print('Continuing manual control (panel align not triggered).')
            else:
                self._controller.stop()
                return

        left_x = self._axis(axes, self.AXIS_LEFT_X)
        left_y = self._axis(axes, self.AXIS_LEFT_Y)
        view_vx = left_y * linear_speed

        right_x = self._axis(axes, self.AXIS_RIGHT_X)
        right_y = self._axis(axes, self.AXIS_RIGHT_Y)
        shift = self._button_pressed(buttons, self._shift_button)

        view_vz = 0.0
        wx = wy = wz = 0.0
        if self._sampling_mode:
            view_vy = -right_y * linear_speed      # inverted
            view_vz = right_x * linear_speed       # inverted
            wz = -left_x * angular_speed           # roll — pitch/yaw locked level
        elif self._drill_mode:
            view_vx = -right_x * linear_speed
            view_vy = -right_y * linear_speed
            view_vz = left_y * linear_speed
        elif shift:
            view_vy = left_x * linear_speed
            wx = right_y * angular_speed          # pitch
            wz = -right_x * angular_speed         # roll
        else:
            view_vy = left_x * linear_speed
            view_vz = -right_y * linear_speed      # stick up = view +Z
            wy = -right_x * angular_speed         # yaw

        # Mount-frame translation is unused here — every gamepad axis is
        # view-relative or a rotation.
        self._controller.set_velocity(
            0.0, 0.0, 0.0, wx, wy, wz,
            view_vx=view_vx, view_vy=view_vy, view_vz=view_vz,
            hold_boost=boost,
        )

        cmd = (view_vx, view_vy, view_vz, wx, wy, wz)
        if any(c != 0.0 for c in cmd):
            # Any real stick-driven motion invalidates "confirmed at
            # sampling_home/drill_home" — the arm may no longer physically
            # be there, so B/Y's hazardous-pose guards must not keep
            # trusting it.
            self._at_sampling_home = False
            self._at_drill_home = False
        if cmd != self._prev_cmd and any(c != 0.0 for c in cmd):
            label = self._active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift)
            print(f'{label} fwd={view_vx:.2f} left={view_vy:.2f} up={view_vz:.2f} '
                  f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f}')
        self._prev_cmd = cmd

    def _handle_panel_align(self):
        """Run panel alignment and hand control back to the operator either way.

        Mirrors KeyboardInputLoop._handle_panel_align — see its docstring
        for why Servo is restarted on failure too, unlike _handle_safe_pose,
        and for why this is guarded by a non-blocking lock.
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
        """Rotate the gripper in place to face the remembered panel direction
        (mirrors KeyboardInputLoop._handle_orient_gripper)."""
        if not self._orient_gripper_running.acquire(blocking=False):
            return
        self._orient_gripper_active = True
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
            self._orient_gripper_active = False
            self._orient_gripper_running.release()

    def _handle_safe_pose(self, target_mode=None):
        """Stop motion and move to the safe pose (mirrors KeyboardInputLoop's 'r').

        Args:
            target_mode: ``'sampling'``/``'drill'`` if this move was
                triggered by a mode-engage press (goes to sampling_home/
                drill_home and, on success, commits that mode), ``None``
                for plain A (jaw/astrobio home), or ``'sampling_container'``/
                ``'drill_container'`` for the sampling_home ->
                sampling_container / drill_home -> drill_container
                sub-position (drill_sampling mode's Y — does not change
                sampling/drill mode itself, the arm is already in that mode
                when this fires; see its own caller in _on_joy for the
                sampling_home/drill_home precondition). Only decides which
                pose to target and which mode to commit AFTER a successful
                move — see below.

        Guarded by a non-blocking lock so a second button press while a
        move is already in progress is ignored instead of racing a
        redundant safe-pose goal against the first. Also sets
        ``_safe_pose_active`` for the duration so ``_on_joy`` ignores
        stick/trigger input while it's set — otherwise a stick held
        during the move would resume Cartesian motion the instant
        ``start_servo()`` re-enables Servo, before the operator has a
        chance to let go.

        This is also the only place ``_teleop_locked`` is cleared (see
        its declaration in ``__init__``).

        Sampling/drill mode itself is committed (on both this loop and
        the controller) ONLY after ``home_ok`` — review-flagged: setting
        it immediately on button press let the software mode (and its
        axis mapping / _level_hold target) change even when the move was
        rejected/failed, leaving the arm physically in its old
        configuration while teleop already assumed the new one.
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        self._safe_pose_active = True
        try:
            self._controller.stop()
            print('Moving to home...')
            if target_mode == 'sampling':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.sampling_home_pose,
                    name=self._controller.sampling_home_pose_name,
                )
            elif target_mode == 'drill':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.drill_home_pose,
                    name=self._controller.drill_home_pose_name,
                )
            elif target_mode == 'sampling_container':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.sampling_container_pose,
                    name=self._controller.sampling_container_pose_name,
                )
            elif target_mode == 'drill_container':
                action = functools.partial(
                    self._controller.move_to_safe_pose,
                    positions=self._controller.drill_container_pose,
                    name=self._controller.drill_container_pose_name,
                )
            else:
                action = self._controller.move_to_safe_pose
            home_ok = self._controller.run_planned_activity(action, 'move_to_safe_pose')
            if home_ok:
                if target_mode == 'sampling':
                    self._sampling_mode = True
                    self._drill_mode = False
                    self._controller.set_sampling_mode(True)
                    self._at_sampling_home = True
                    self._at_drill_home = False
                elif target_mode == 'drill':
                    self._drill_mode = True
                    self._sampling_mode = False
                    self._controller.set_drill_mode(True)
                    self._at_sampling_home = False
                    self._at_drill_home = True
                elif target_mode == 'sampling_container':
                    # Left the sampling_home hub for a spoke pose — B/Y's
                    # own guards must not treat this as still being at
                    # sampling_home (that was the bug: sampling_container
                    # -> drill_home was wrongly allowed before this flag
                    # existed, because _sampling_mode alone doesn't
                    # distinguish "at the hub" from "at this spoke").
                    self._at_sampling_home = False
                elif target_mode == 'drill_container':
                    # Same reasoning as sampling_container above, mirrored
                    # for the drill_home hub.
                    self._at_drill_home = False
                print('Starting servo...')
                if self._controller.start_servo():
                    self._teleop_locked = False
                    self._controller.get_logger().info('Teleop enabled.')
                else:
                    self._controller.get_logger().warn(
                        'Servo failed to start — staying on trajectory controller.'
                    )
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_active = False
            self._safe_pose_running.release()

    def _handle_level(self):
        """Reorient the tool straight down via a collision-checked plan
        (mirrors KeyboardInputLoop's 'f' — see level_tool's own docstring
        for why this needs to be its own move_group plan).

        ``_level_active`` mirrors ``_safe_pose_active``'s own role: held
        for the duration so ``_on_joy`` ignores stick input mid-move,
        same reasoning as that flag's own declaration.
        """
        if not self._level_running.acquire(blocking=False):
            return
        self._level_active = True
        try:
            self._controller.stop()
            print('Leveling tool...')
            if self._controller.run_planned_activity(self._controller.level_tool, 'level_tool'):
                print('Tool leveled.')
            else:
                print('Level move failed.')
            print('Resuming manual control...')
            self._controller.start_servo()
        finally:
            self._level_active = False
            self._level_running.release()

    def run(self):
        """Print the help banner and block until the exit button is pressed."""
        print(GAMEPAD_HELP)
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()


