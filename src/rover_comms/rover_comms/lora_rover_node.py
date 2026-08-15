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

Two things keep it safe:

  * It only ever transmits in reply. The channel is half-duplex and shared, so
    if both ends talk at once the frames collide and both are lost. The mast
    polls; this answers, immediately, inside the same read that consumed the
    poll. Nothing here is on a timer.
  * It publishes zero and keeps publishing zero when the polls stop. Failing to
    stopped, never to last-known-good, and it starts in that state rather than
    waiting for a first timeout.

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

from rover_comms import lora_frame


class LoraRoverNode(Node):

    def __init__(self):
        super().__init__("lora_rover_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baud", 9600)
        # Three missed polls at the mast's 3 Hz. Long enough to ride out a
        # single dropped frame, short enough that a dead link stops the rover
        # before it travels far. At 0.3 m/s this is still ~30 cm of coasting,
        # which is why the LoRa speed limits below are deliberately low.
        self.declare_parameter("failsafe_timeout", 1.0)
        # Full-scale values the wire percentages map back to. These MUST match
        # lora_gateway_node's max_linear/max_angular on the ground station -
        # the wire carries percent, not m/s, so a mismatch here silently scales
        # every command the operator gives. Lower than the Wi-Fi path's limits
        # on purpose: a link with 0.5 s of lag and three updates a second should
        # not be driving at full speed.
        self.declare_parameter("max_linear", 0.3)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.port_name = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.failsafe_timeout = float(self.get_parameter("failsafe_timeout").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel_lora", 10)
        self.pub_state = self.create_publisher(String, "lora/rover_state", 10)
        self.pub_rtt_frames = self.create_publisher(Float32, "lora/rover_rx_ok", 10)
        self.pub_estop = self.create_publisher(Bool, "lora/rover_estop", 10)

        self.parser = lora_frame.Parser()
        self.command = lora_frame.Teleop()
        self.tx_seq = 0
        self.tx_frames = 0
        self.last_frame_at = None
        # Start failsafed: a rover that has never heard from the mast is in
        # exactly the state a rover that stopped hearing from it is in.
        self.failsafe = True
        self._lock = threading.Lock()

        # Opened in the reader thread, not here. This is the fallback link: if
        # the radio is unplugged, or the cable is loose, or nvgetty grabbed the
        # port, the node must degrade to "permanently failsafed" rather than
        # refuse to start - it is included in the rover's bringup, and taking
        # the whole rover down because a backup radio is missing would be
        # exactly the wrong failure.
        self.ser = None

        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value), self._publish)

        self.get_logger().info(
            f"lora_rover_node: {self.port_name} at {self.baud}, "
            f"failsafe {self.failsafe_timeout}s, "
            f"scales {self.max_linear} m/s / {self.max_angular} rad/s")

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

            self.get_logger().info(f"listening on {self.port_name}")
            backoff = 1.0
            self._pump()
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
                self._on_teleop(payload)
                if not self._reply(seq):
                    return

    def _on_teleop(self, payload):
        command = lora_frame.unpack_teleop(payload)
        if command.flags & lora_frame.FLAG_ESTOP:
            # Zero it here rather than trusting the sender to have done so.
            command = lora_frame.Teleop(0, 0, 0, command.flags)
        with self._lock:
            self.command = command
            self.last_frame_at = self.get_clock().now()
            if self.failsafe:
                self.get_logger().info("link up - taking commands over LoRa")
            self.failsafe = False

    def _reply(self, echo_seq):
        with self._lock:
            flags = ((lora_frame.STATUS_FAILSAFE if self.failsafe else 0) |
                     (lora_frame.STATUS_ESTOP
                      if self.command.flags & lora_frame.FLAG_ESTOP else 0))
        status = lora_frame.Status(
            echo_seq=echo_seq,
            rx_ok=self.parser.ok & 0xFF,
            rx_bad=self.parser.bad & 0xFF,
            flags=flags,
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
        with self._lock:
            if not self.failsafe and self.last_frame_at is not None:
                age = (self.get_clock().now() - self.last_frame_at).nanoseconds / 1e9
                if age > self.failsafe_timeout:
                    self.failsafe = True
                    self.command = lora_frame.Teleop()
                    self.get_logger().warn(
                        f"no valid teleop frame for {age:.1f}s - command zeroed")
            command = self.command
            failsafe = self.failsafe

        twist = Twist()
        if not failsafe:
            twist.linear.x = command.vx / 100.0 * self.max_linear
            twist.linear.y = command.vy / 100.0 * self.max_linear
            twist.angular.z = command.wz / 100.0 * self.max_angular
        # A failsafed node keeps publishing zeros rather than going silent. Once
        # twist_mux has this input selected, going quiet would let it fall
        # through to a lower priority instead of commanding a stop.
        self.pub_cmd.publish(twist)

        self.pub_state.publish(String(data="FAILSAFE" if failsafe else "LINKED"))
        self.pub_rtt_frames.publish(Float32(data=float(self.parser.ok)))
        self.pub_estop.publish(
            Bool(data=bool(command.flags & lora_frame.FLAG_ESTOP)))


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
