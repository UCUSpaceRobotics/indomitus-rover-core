#!/usr/bin/env python3
"""Failsafe and command state for the LoRa link, with no ROS in it.

Split out of lora_rover_node so the safety behaviour can be tested without a
ROS environment. Everything that decides whether the rover moves lives here;
the node is the part that talks to rclpy and the serial port. The rules:

  * Start failsafed. A rover that has never heard from the mast is in exactly
    the state a rover that stopped hearing from it is in.
  * Zero on timeout.
  * Never fail to last-known-good.
  * Cap what is driven, here, rather than trusting the sender to have capped
    it. See lora_rover_node's docstring for wire scale vs cap.

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
                 limit_linear, limit_angular, clock=time.monotonic):
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
        return recovered

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
