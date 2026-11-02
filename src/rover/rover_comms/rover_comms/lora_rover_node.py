#!/usr/bin/env python3
"""Rover end of the 433 MHz fallback link.

When the ground station's Wi-Fi dies, `link_status_node` on the GS PC flips
/link/active_path to LORA and `lora_gateway_node` starts pushing the operator's
commands into `lora_bridge.py` on the mast Pi, which polls this node over the
radio. This is where those commands re-enter the rover's ROS graph.

It replaces the ESP32-S3 bench rig that stood in for the rover during bring-up
(indomitus-ground-station, microcontrollers_indomitus/esp32s3_lora_rover). The
protocol is identical - that firmware is the reference implementation and is
worth keeping around as a test peer.

The link is slow and this does not pretend otherwise. Measured on the bench:
240 ms round trip, 3 Hz polls, 168 B/s one way. Roughly 0.3-0.6 s from the
operator's stick to this node, three updates a second. It exists so a rover that
has lost Wi-Fi can be crawled somewhere recoverable, not so it can be driven.

This node is the ROS and serial half. Everything that decides whether the rover
moves is in link_state.LinkState, which has no ROS in it and is tested directly
in test/test_link_state.py:

  * It only ever transmits in reply. The channel is half-duplex and shared, so
    if both ends talk at once the frames collide and both are lost. The mast
    polls; this answers, immediately, inside the same read that consumed the
    poll. Nothing here is on a timer.
  * It publishes zero - a burst of zero_burst publishes, then goes quiet -
    when the polls stop. Failing to stopped, never to last-known-good, and it
    starts in that state rather than waiting for a first timeout. A serial
    error zeroes the command immediately rather than waiting out
    failsafe_timeout: the port is known dead at that point, and the timeout
    only exists for the case where it is not. Going quiet after the burst is
    safe, not a compromise: LoRa is twist_mux's lowest-priority input, so
    there is nothing beneath it for the topic to protect by staying open.
  * It clamps what it drives, at this end, to limit_linear/limit_angular. The
    crawl-home cap is enforced by the machine with the motors, not asked for
    politely of whoever is transmitting.

Scale and cap are separate on purpose, and conflating them is the easiest way
to break this link quietly:

  * max_linear/max_angular are the *wire scale*. The frame carries percent of
    full scale, not m/s, so these MUST equal lora_gateway_node's on the ground
    station. Mismatch them and every command is silently rescaled - it still
    drives, just not at the speed the operator asked for, with nothing logged.
  * limit_linear/limit_angular are the *cap*, and are ours alone. They are what
    makes this a crawl-home link rather than a driving one, and the ground
    station neither knows nor needs to know them.

Using a deliberately-too-small wire scale to get a lower top speed would appear
to work and would also shrink every command below the cap by the same ratio,
which is a rescale, not a limit.

FLAG_ESTOP stops *this path*, not the rover. It zeroes cmd_vel_lora and is
reported back to the mast, but twist_mux will still select any higher-priority
input that is active, so nav2 in particular can keep driving through it. Making
it a rover-wide stop means a twist_mux lock, and twist_mux locks fail closed -
`isLocked()` is `hasExpired() || data`, so a lock topic whose publisher is
missing immobilises the rover completely. That would make a rover with
rover_comms unbuilt refuse to move at all, which is the failure this node's
whole startup path is written to avoid. If a real rover-wide e-stop is wanted
it needs a lock published by something that is always running, which is a
change to core bringup rather than to the backup radio.

Command output goes to cmd_vel_lora, which twist_mux carries at a priority
below cmd_vel_ext. That means the rover-side failover needs no logic at all:
while Wi-Fi is alive the external input outranks this one, and when it goes
stale twist_mux switches over on its own.

Mode 0 only. M0 and M1 are expected to be strapped low in hardware and AUX is
not wired - see the wiring notes in the ground-station repo's mast/README.md.
This node never changes the module's mode, so it needs no GPIO at all, which is
also why it does not care that Jetson.GPIO cannot identify the Seeed carrier.
"""

import threading

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from rover_comms import link_state, lora_frame


class LoraRoverNode(Node):

    def __init__(self):
        super().__init__("lora_rover_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baud", 9600)
        # Three missed polls at the mast's 3 Hz. Long enough to ride out a
        # single dropped frame, short enough that a dead link stops the rover
        # before it travels far. At 0.3 m/s this is still ~30 cm of coasting,
        # which is why the LoRa speed limits below are deliberately low. It is
        # the backstop for a link that goes quiet without the port noticing; a
        # port that errors is handled immediately instead.
        self.declare_parameter("failsafe_timeout", 1.0)
        # Full-scale values the wire percentages map back to. These MUST match
        # lora_gateway_node's max_linear/max_angular on the ground station -
        # the wire carries percent, not m/s, so a mismatch here silently scales
        # every command the operator gives. Those in turn match the joystick's
        # linear_x_scale/angular_z_scale, so 100% means the same thing all the
        # way from the stick to here. This is not where the speed limit lives.
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 1.0)
        # The speed limit, applied after decoding. Lower than the Wi-Fi path on
        # purpose: a link with 0.5 s of lag and three updates a second should
        # not be driving at full speed. Enforced here rather than trusted from
        # the sender, because this end is the one attached to the motors.
        self.declare_parameter("limit_linear", 0.3)
        self.declare_parameter("limit_angular", 0.6)
        self.declare_parameter("publish_rate_hz", 10.0)
        # Publishes of cmd_vel_lora on the way into failsafe before this goes
        # quiet - see LinkState's docstring. Matches JoyWatchdog's
        # timeout_zero_burst default.
        self.declare_parameter("zero_burst", 3)

        self.port_name = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if self.publish_rate_hz <= 0.0:
            # A zero here would be a ZeroDivisionError in the constructor, which
            # takes bringup down - the one thing this node must never do.
            self.get_logger().warn(
                f"publish_rate_hz={self.publish_rate_hz} is not positive, "
                f"falling back to 10 Hz")
            self.publish_rate_hz = 10.0

        # time.monotonic, not the ROS clock: this is a link watchdog, and it
        # must not be defeated by an NTP step or a paused /clock. A Jetson
        # syncs time some seconds after boot, which is exactly when a rover is
        # most likely to be sitting there waiting for its first frame.
        self.state = link_state.LinkState(
            failsafe_timeout=float(self.get_parameter("failsafe_timeout").value),
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            limit_linear=float(self.get_parameter("limit_linear").value),
            limit_angular=float(self.get_parameter("limit_angular").value),
            zero_burst=int(self.get_parameter("zero_burst").value),
        )

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel_lora", 10)
        self.pub_state = self.create_publisher(String, "lora/rover_state", 10)
        self.pub_rx_ok = self.create_publisher(Float32, "lora/rover_rx_ok", 10)
        self.pub_estop = self.create_publisher(Bool, "lora/rover_estop", 10)

        self.parser = lora_frame.Parser()
        self.tx_seq = 0
        self.tx_frames = 0

        # Opened in the reader thread, not here. This is the fallback link: if
        # the radio is unplugged, or the cable is loose, or nvgetty grabbed the
        # port, the node must degrade to "permanently failsafed" rather than
        # refuse to start - it is included in the rover's bringup, and taking
        # the whole rover down because a backup radio is missing would be
        # exactly the wrong failure.
        self.ser = None

        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

        # The wire scale is logged because it is half of a cross-repository
        # contract with nothing enforcing it at runtime: if this line and
        # lora_gateway_node's disagree, that is the bug, and it is otherwise
        # invisible - the link works, at the wrong speed.
        self.get_logger().info(
            f"lora_rover_node: {self.port_name} at {self.baud}, "
            f"failsafe {self.state.failsafe_timeout}s, "
            f"wire scale {self.state.max_linear} m/s / "
            f"{self.state.max_angular} rad/s (must match lora_gateway_node), "
            f"capped at {self.state.limit_linear} m/s / "
            f"{self.state.limit_angular} rad/s")

    # -- radio -------------------------------------------------------------

    def _reader(self):
        """Own the serial port: open it, answer every poll, reopen if it dies.

        A thread rather than a timer because the answer has to go out as soon
        as the poll lands: the mast will not send again until it has the reply
        or times out, and any delay here shows up directly as round-trip time.
        """
        backoff = 1.0
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.port_name, self.baud, timeout=0)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(
                    f"cannot open {self.port_name}: {exc}. Check the user is in "
                    f"dialout and that nvgetty/serial-getty are disabled. "
                    f"Retrying in {backoff:.0f}s - the rover stays failsafed "
                    f"on this path meanwhile.")
                threading.Event().wait(backoff)
                backoff = min(backoff * 2, 30.0)
                continue

            # Whatever was half-received when the port died must not be joined
            # to what arrives now. Counters survive; framing state does not.
            self.parser.reset()
            self.get_logger().info(f"listening on {self.port_name}")
            backoff = 1.0
            self._pump()

            # The port is known dead here, so there is nothing to wait for:
            # zero the command now rather than letting it stand for up to
            # failsafe_timeout while the reopen is attempted.
            if self.state.on_link_lost():
                self.get_logger().warn(
                    "serial link lost - command zeroed immediately")
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None

    def _pump(self):
        """Read and answer until the port errors out."""
        while rclpy.ok():
            try:
                waiting = self.ser.in_waiting
                chunk = self.ser.read(waiting) if waiting else b""
            except (serial.SerialException, OSError, TypeError) as exc:
                self.get_logger().error(f"serial read failed, reopening: {exc}")
                return
            if not chunk:
                # Short enough not to add measurably to the round trip, long
                # enough not to spin a core on an idle link.
                threading.Event().wait(0.002)
                continue

            for frame_type, seq, payload in self.parser.feed(chunk):
                if frame_type != lora_frame.TYPE_TELEOP:
                    # Our own reply looped back, or a throughput-test frame.
                    continue
                if self.state.on_teleop(payload):
                    self.get_logger().info("link up - taking commands over LoRa")
                if not self._reply(seq):
                    return

    def _reply(self, echo_seq):
        status = lora_frame.Status(
            echo_seq=echo_seq,
            rx_ok=self.parser.ok & 0xFF,
            rx_bad=self.parser.bad & 0xFF,
            flags=self.state.status_flags(),
        )
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        frame = lora_frame.encode(
            lora_frame.TYPE_STATUS, self.tx_seq, lora_frame.pack_status(status))
        try:
            self.ser.write(frame)
            self.tx_frames += 1
            return True
        except (serial.SerialException, OSError, AttributeError) as exc:
            self.get_logger().error(f"serial write failed, reopening: {exc}")
            return False

    # -- ROS ---------------------------------------------------------------

    def _publish(self):
        age = self.state.check_timeout()
        if age is not None:
            self.get_logger().warn(
                f"no valid teleop frame for {age:.1f}s - command zeroed")

        # Zero while failsafed, and only a short burst of those - then quiet.
        # See LinkState's docstring for why: LoRa is twist_mux's lowest
        # priority, so there is nothing to protect by holding the topic open,
        # and a permanently-fresh zero reads to anything watching twist_mux's
        # inputs (drive_source_lamp_node) as "someone is driving" when nobody
        # is.
        if self.state.should_publish():
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = \
                self.state.twist_components()
            self.pub_cmd.publish(twist)

        self.pub_state.publish(
            String(data="FAILSAFE" if self.state.failsafe else "LINKED"))
        self.pub_rx_ok.publish(Float32(data=float(self.parser.ok)))
        self.pub_estop.publish(Bool(data=self.state.estop_active()))


def main():
    rclpy.init()
    node = LoraRoverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
