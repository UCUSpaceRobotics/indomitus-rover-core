#!/usr/bin/env python3
"""
Drive-state indicator on a PlayStation controller's light bar.

Deliberately free of any ROS import: the colour rules and the sysfs writing
are the parts worth testing, and they test far more cheaply on their own.

Everything here is best-effort. A controller with no light bar, a missing udev
rule, a light bar that has not reappeared yet after a reconnect — none of these
may disturb teleop, so every failure path ends in a log line and nothing else.
"""

import glob
import os


# DualSense (and DualSense Edge): one multicolor LED carrying an RGB triple.
DUALSENSE_GLOB = '/sys/class/leds/*:rgb:indicator'
# DualShock 4: three single-colour LEDs plus a ':global' on/off gate. Same
# hid-playstation driver, different sysfs shape — see docs/hardware/joystick.md.
DUALSHOCK_GLOB = '/sys/class/leds/*:global'

DUALSHOCK_CHANNELS = ('red', 'green', 'blue')

# The light bar index changes on every reconnect (input19 → input21 → …),
# hence the globbing on each repaint rather than a path resolved once.

LED_MOTORS_OFF       = (255, 0, 0)      # red     — hardware inactive
LED_MOTORS_INHIBITED = (255, 0, 255)    # magenta — faults cleared, needs a motor-button cycle
LED_CONTROLLER_OFF   = (255, 120, 0)    # orange  — motors on, controller inactive
LED_NAV              = (0, 0, 255)      # blue    — yielding to nav
LED_DRIVING          = (0, 255, 0)      # green   — joystick in command


def led_colour(motors_enabled: bool, motors_inhibited: bool,
               controller_active: bool, joystick_active: bool):
    """Resolve the drive state to a colour, most-serious state first.

    The ordering is the whole point: the light bar may only ever show a state
    at least as permissive as reality. Green in particular must mean the
    joystick can actually move the rover right now.
    """
    if not motors_enabled:
        return LED_MOTORS_OFF
    if motors_inhibited:
        # Motors report enabled, but the hardware was told to drop its latched
        # faults and will not drive until the motor button is cycled. Showing
        # green here would promise control that does not exist.
        return LED_MOTORS_INHIBITED
    if not controller_active:
        return LED_CONTROLLER_OFF
    if not joystick_active:
        return LED_NAV
    return LED_DRIVING


def _max_brightness(base: str, fallback: str = '255') -> str:
    """The device's full-scale brightness, as a string ready to write."""
    try:
        with open(os.path.join(base, 'max_brightness')) as handle:
            value = handle.read().strip()
    except OSError:
        return fallback
    return value or fallback


class ControllerLed:
    """Paints a PlayStation light bar with the current drive state.

    Repaints are retried for a short while because a controller that has just
    been plugged in races us: the LED device may not exist yet, and when it
    does, udev may not have handed it to `plugdev` yet. Without the retry a
    reconnect can leave the bar showing the driver's default blue — a state the
    operator would read as 'yielding to navigation'.
    """

    def __init__(self, logger, retry_attempts: int = 6,
                 dualsense_glob: str = DUALSENSE_GLOB,
                 dualshock_glob: str = DUALSHOCK_GLOB):
        self._log = logger
        self._retry_attempts = retry_attempts
        self._dualsense_glob = dualsense_glob
        self._dualshock_glob = dualshock_glob

        self._colour = None
        self._retries_left = 0
        # Failure kinds already reported at warn level. Keyed by errno rather
        # than by path so a permission problem is announced once, not once per
        # reconnect — the path carries an input index that keeps changing.
        self._warned = set()

    @property
    def colour(self):
        """Colour last asked for, or None if nothing has been painted yet."""
        return self._colour

    @property
    def retry_pending(self) -> bool:
        return self._retries_left > 0

    def set(self, colour) -> bool:
        """Paint `colour` now, and keep retrying briefly if it does not take."""
        self._colour = colour
        self._retries_left = self._retry_attempts
        return self._attempt()

    def retry_tick(self) -> bool:
        """Re-attempt a repaint that has not succeeded yet. Cheap no-op once
        the light bar is showing the right colour, or once we have given up."""
        if not self._retries_left or self._colour is None:
            return False
        return self._attempt()

    def _attempt(self) -> bool:
        targets = self._targets()

        if not targets:
            # No light bar. Either the pad has none (any Xbox controller), or
            # it is a PlayStation pad whose LED device has not appeared yet.
            # Retries cover the second case and cost nothing in the first, so
            # this is never worth a warning.
            self._retries_left = max(0, self._retries_left - 1)
            return False

        ok = True
        for path, value in targets:
            try:
                with open(path, 'w') as handle:
                    handle.write(value)
            except OSError as exc:
                ok = False
                self._report(path, exc)

        self._retries_left = 0 if ok else max(0, self._retries_left - 1)
        return ok

    def _report(self, path: str, exc: OSError):
        """A found-but-unwritable light bar is a real misconfiguration — say so
        out loud, but only the first time each kind of failure shows up."""
        self._log.debug(f'controller LED write to {path} failed: {exc!r}')

        if exc.errno in self._warned:
            return
        self._warned.add(exc.errno)
        self._log.warning(
            f'controller LED found at {path} but not writable: {exc!r} — '
            f'the light bar will not track the drive state. Install the udev '
            f'rule with: scripts/setup_host.sh <rover|local> --joystick-led '
            f'(and make sure this user is in the plugdev group)')

    def _targets(self):
        """(path, value) pairs to write, in the order they must be written."""
        if self._colour is None:
            return []

        red, green, blue = self._colour
        targets = []

        for base in sorted(glob.glob(self._dualsense_glob)):
            # Output is intensity × brightness, and the driver resets
            # brightness on reconnect, so pinning the intensities alone can
            # leave the bar dark. brightness goes last: writing it is what
            # latches the new colour out to the controller.
            targets.append((os.path.join(base, 'multi_intensity'),
                            f'{red} {green} {blue}'))
            targets.append((os.path.join(base, 'brightness'),
                            _max_brightness(base)))

        for gate in sorted(glob.glob(self._dualshock_glob)):
            prefix = gate[: -len(':global')]
            channels = [(f'{prefix}:{name}', value) for name, value
                        in zip(DUALSHOCK_CHANNELS, (red, green, blue))]
            # ':global' is not a PlayStation-only name; only treat it as a
            # light bar when the three colour channels sit beside it.
            if not all(os.path.isdir(path) for path, _ in channels):
                continue
            targets.extend((os.path.join(path, 'brightness'), str(value))
                           for path, value in channels)
            targets.append((os.path.join(gate, 'brightness'),
                            _max_brightness(gate)))

        return targets
