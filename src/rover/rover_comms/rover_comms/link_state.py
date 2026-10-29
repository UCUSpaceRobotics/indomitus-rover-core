#!/usr/bin/env python3
"""Failsafe and command state for the LoRa link, with no ROS in it.

Split out of lora_rover_node so the safety behaviour can be tested without a
ROS environment. Everything that decides whether the rover moves lives here;
the node is the part that talks to rclpy and the serial port. The rules:

  * Start failsafed. A rover that has never heard from the mast is in exactly
    the state a rover that stopped hearing from it is in.
  * Zero on timeout, and zero immediately on a known link loss. Waiting out
    failsafe_timeout after the port has already errored means up to a second
    of driving on a command nobody can retract.
  * Never fail to last-known-good.
  * Cap what is driven, here, rather than trusting the sender to have capped
    it. See lora_rover_node's docstring for wire scale vs cap.
  * A failsafed link gets a short burst of zero publishes, then goes quiet -
    same policy as JoyWatchdog (rover_teleop/teleop_state.py) and the same
    reason: LoRa is twist_mux's lowest-priority input (see
    rover_bringup/config/twist_mux.yaml), so there is nothing beneath it to
    fall through to. Publishing zero forever would instead make LoRa look
    like a live "someone is driving" source to anything watching twist_mux's
    inputs - drive_source_lamp_node in particular, which would read a
    permanently-fresh zero as manual control rather than nobody driving.
    Going quiet is exactly as safe as JoyWatchdog going quiet: the swerve
    controller's own cmd_vel_timeout_s stops the rover once twist_mux has no
    fresh input left at all, whichever priority that was.

The clock is injectable and defaults to time.monotonic, deliberately. A link
watchdog measured on a wall clock can be defeated by an NTP step or a paused
/clock - both real on a Jetson that syncs time some seconds after boot - and a
backward jump would hold the failsafe off indefinitely.
"""

import threading
import time

from rover_comms import lora_frame


def clamp_percent(value):
    """Wire percentages are -100..100; the signed byte carrying them is not."""
    return max(-100, min(100, value))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class LinkState:
    """Thread-safe: the reader thread feeds it, the publish timer reads it."""

    def __init__(self, failsafe_timeout, max_linear, max_angular,
                 limit_linear, limit_angular, zero_burst=3, clock=time.monotonic):
        self.failsafe_timeout = float(failsafe_timeout)
        # Wire scale: must match lora_gateway_node's. Not the speed limit.
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        # The speed limit, ours alone.
        self.limit_linear = abs(float(limit_linear))
        self.limit_angular = abs(float(limit_angular))
        self._clock = clock

        self._lock = threading.RLock()
        self._command = lora_frame.Teleop()
        self._last_frame_at = None
        self._failsafe = True
        self._zero_burst = max(0, int(zero_burst))
        self._zeros_left = self._zero_burst

    @property
    def failsafe(self):
        with self._lock:
            return self._failsafe

    @property
    def command(self):
        with self._lock:
            return self._command

    def on_teleop(self, payload):
        """Accept a decoded teleop frame. True if this brought the link back."""
        command = lora_frame.unpack_teleop(payload)
        if command.flags & lora_frame.FLAG_ESTOP:
            # Zero it here rather than trusting the sender to have done so.
            command = lora_frame.Teleop(0, 0, 0, command.flags)
        else:
            # The payload is a signed byte, so it carries -128..127, but the
            # protocol says percent. lora_gateway_node clamps before sending;
            # that is not a reason for this end to act on 127%.
            command = lora_frame.Teleop(
                clamp_percent(command.vx), clamp_percent(command.vy),
                clamp_percent(command.wz), command.flags)
        with self._lock:
            recovered = self._failsafe
            self._command = command
            self._last_frame_at = self._clock()
            self._failsafe = False
            # Every live frame re-arms the burst, so the next disconnect gets
            # a full stop of its own however brief the reconnect was.
            self._zeros_left = self._zero_burst
        return recovered

    def on_link_lost(self):
        """The port errored: stop now rather than waiting out the timeout.

        Returns True if this changed anything. Flags are kept - a dead serial
        port is not the operator releasing the e-stop.
        """
        with self._lock:
            if self._failsafe:
                return False
            self._command = lora_frame.Teleop(0, 0, 0, self._command.flags)
            self._last_frame_at = None
            self._failsafe = True
            return True

    def check_timeout(self):
        """Returns the age in seconds if this call tripped the failsafe."""
        with self._lock:
            if self._failsafe or self._last_frame_at is None:
                return None
            age = self._clock() - self._last_frame_at
            if age <= self.failsafe_timeout:
                return None
            self._failsafe = True
            self._command = lora_frame.Teleop(0, 0, 0, self._command.flags)
            return age

    def should_publish(self) -> bool:
        """Whether this publish tick should put anything on cmd_vel_lora.

        Live commands publish every tick, unmetered. A failsafed link gets
        only zero_burst more publishes of the zero it is already holding, then
        this returns False forever until the next on_teleop re-arms it - see
        the class docstring for why going quiet here is safe.
        """
        with self._lock:
            if not self._failsafe:
                return True
            if self._zeros_left <= 0:
                return False
            self._zeros_left -= 1
            return True

    def twist_components(self):
        """(vx, vy, wz) in m/s and rad/s, decoded at the wire scale then capped."""
        with self._lock:
            if self._failsafe:
                return 0.0, 0.0, 0.0
            command = self._command
            return (
                clamp(command.vx / 100.0 * self.max_linear, self.limit_linear),
                clamp(command.vy / 100.0 * self.max_linear, self.limit_linear),
                clamp(command.wz / 100.0 * self.max_angular, self.limit_angular),
            )

    def status_flags(self):
        with self._lock:
            return ((lora_frame.STATUS_FAILSAFE if self._failsafe else 0) |
                    (lora_frame.STATUS_ESTOP if self._command.flags &
                     lora_frame.FLAG_ESTOP else 0))

    def estop_active(self):
        with self._lock:
            return bool(self._command.flags & lora_frame.FLAG_ESTOP)
