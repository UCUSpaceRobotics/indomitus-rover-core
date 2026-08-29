#!/usr/bin/env python3
"""
ROS2 node - lights control via CAN bus (ros2_socketcan).

This node owns the light state. Two operators command the lights and they ask
differently: the onboard joystick has momentary buttons and calls the /toggle
services, the ground station has latching switches and calls the SetBool ones
with an absolute value. Neither keeps a copy, so neither can drift; both read
the truth back off lights/state.

Service: lights/spotlight         (std_srvs/srv/SetBool)  - absolute
Service: lights/spotlight/toggle  (std_srvs/srv/Trigger)  - invert
Service: lights/beautiful         (std_srvs/srv/SetBool)  - absolute
Service: lights/beautiful/toggle  (std_srvs/srv/Trigger)  - invert
Service: lights/traffic_light     (indomitus_interfaces/srv/SetTrafficLight)
  request:  int8 red/yellow/green/blue, each KEEP | OFF | ON
  response: bool success, string message

Topic:   lights/state             (indomitus_interfaces/msg/LightsState)
  Latched. Republished on a 2 Hz heartbeat so a console can tell live state
  from a dead publisher, and pushed out at once on a real change so an
  operator's switch does not appear to lag - see _heartbeat and _publish_change.

CAN TX (PC -> ESP32)  ID cmd_id:
  Spotlight:     byte 0 = cmd_spotlight_on | cmd_spotlight_off
  Traffic light: byte 0 = cmd_traffic_light, byte 1 = bitmask (R=bit0 Y=bit1 G=bit2 B=bit3)

CAN RX (ESP32 -> PC):
  ID resp_id  byte 0 = echo cmd, byte 1 = 0x00 OK | 0x01 ERROR
"""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from can_msgs.msg import Frame
from std_srvs.srv import SetBool, Trigger
from indomitus_interfaces.msg import LightsState as LightsStateMsg
from indomitus_interfaces.srv import SetTrafficLight

import threading
from dataclasses import replace

from rover_peripherals.lights_state import (
    LightsState,
    describe_traffic,
    resolve_traffic,
    traffic_mask,
)


class LightsCanNode(Node):
    def __init__(self):
        super().__init__("lights_can_node")

        self._declare_parameters()
        self._load_parameters()

        # --- callback groups ---
        # Every light service shares one group: they are serialised by
        # _can_lock anyway, and each one blocks for up to ack_s waiting on the
        # ESP32. The CAN subscription gets its own group so it stays free to
        # deliver the very ACK a blocked service is waiting for — putting it in
        # with the services would deadlock every call until the timeout.
        self._sub_cbg      = MutuallyExclusiveCallbackGroup()
        self._service_cbg  = MutuallyExclusiveCallbackGroup()
        self._state_cbg    = MutuallyExclusiveCallbackGroup()

        # --- state ---
        self._state = LightsState()
        # Serialises a whole send -> wait -> commit transaction. _resp_lock
        # below only guards the response fields; without this one, two
        # concurrent calls would overwrite each other's _resp_pending_cmd and
        # a toggle could read a state another call was midway through changing.
        self._can_lock = threading.Lock()
        self._state_dirty = True
        self._state_period = 1.0 / max(0.1, self._state_pub_rate)
        # Floor between out-of-turn publishes. A real change goes out at once
        # rather than waiting for the heartbeat; this only stops a burst of
        # changes from turning into a burst of packets.
        self._state_min_period = 0.05
        self._last_state_publish = 0.0

        # LightsState field -> (log label, CAN 'on' command, CAN 'off' command).
        # Both the SetBool and the /toggle handler for a light go through the
        # same entry, so the two can never disagree about what they send.
        self._bool_lights = {
            "spotlight": ("SPOTLIGHT", self._cmd_spotlight_on, self._cmd_spotlight_off),
            "beautiful": ("BEAUTIFUL_LIGHT", self._cmd_beautiful_on, self._cmd_beautiful_off),
        }

        # --- CAN pub/sub ---
        self._pub = self.create_publisher(Frame, "to_can_bus", 10)
        self._sub = self.create_subscription(
            Frame, "from_can_bus", self._on_can_msg, 10,
            callback_group=self._sub_cbg,
        )

        # --- State topic ---
        # TRANSIENT_LOCAL on top of the heartbeat: a late subscriber (the
        # ground station UI after a reconnect) gets the last value on the spot
        # rather than up to a heartbeat period later.
        self._state_pub = self.create_publisher(
            LightsStateMsg,
            "lights/state",
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._state_timer = self.create_timer(
            self._state_period,
            self._publish_state,
            callback_group=self._state_cbg,
        )

        # --- Services ---
        self._spotlight_srv = self.create_service(
            SetBool, "lights/spotlight", self._on_spotlight_request,
            callback_group=self._service_cbg,
        )
        self._spotlight_toggle_srv = self.create_service(
            Trigger, "lights/spotlight/toggle", self._on_spotlight_toggle,
            callback_group=self._service_cbg,
        )
        self._beautiful_srv = self.create_service(
            SetBool, "lights/beautiful", self._on_beautiful_request,
            callback_group=self._service_cbg,
        )
        self._beautiful_toggle_srv = self.create_service(
            Trigger, "lights/beautiful/toggle", self._on_beautiful_toggle,
            callback_group=self._service_cbg,
        )
        self._traffic_srv = self.create_service(
            SetTrafficLight, "lights/traffic_light", self._on_traffic_request,
            callback_group=self._service_cbg,
        )

        # --- response state ---
        self._resp_lock  = threading.Lock()
        self._resp_event = threading.Event()
        self._resp_pending_cmd: int | None = None
        self._resp_status: int | None = None

        self._publish_state()

        self.get_logger().info(
            f"LightsCanNode ready\n"
            f"  CAN TX id=0x{self._cmd_id:03X}\n"
            f"  CAN RX id=0x{self._resp_id:03X}\n"
            f"  /lights/spotlight         (Service) - SetBool, absolute\n"
            f"  /lights/spotlight/toggle  (Service) - Trigger, invert\n"
            f"  /lights/beautiful         (Service) - SetBool, absolute\n"
            f"  /lights/beautiful/toggle  (Service) - Trigger, invert\n"
            f"  /lights/traffic_light     (Service) - per-colour KEEP/OFF/ON\n"
            f"  /lights/state             (Topic)   - latched, {self._state_pub_rate} Hz + on change\n"
        )

    # =======================================================================
    # Parameters
    # =======================================================================

    def _declare_parameters(self):
        self.declare_parameter("can.cmd_id",              0x300)
        self.declare_parameter("can.resp_id",             0x301)
        self.declare_parameter("can.cmd_spotlight_on",    0x01)
        self.declare_parameter("can.cmd_spotlight_off",   0x02)
        self.declare_parameter("can.cmd_traffic_light",   0x03)
        self.declare_parameter("can.status_ok",           0x00)
        self.declare_parameter("can.status_error",        0x01)
        self.declare_parameter("timeouts.ack_s",          2.0)
        self.declare_parameter("can.cmd_beautiful_light_on",  0x04)
        self.declare_parameter("can.cmd_beautiful_light_off", 0x05)
        # Capped low on purpose: lights/state crosses the Wi-Fi link to the
        # ground station, and nothing downstream needs it faster than the
        # joystick repaints its light bar.
        self.declare_parameter("state_pub_rate",          2.0)

    def _load_parameters(self):
        self._cmd_id             = self.get_parameter("can.cmd_id").value
        self._resp_id            = self.get_parameter("can.resp_id").value
        self._cmd_spotlight_on   = self.get_parameter("can.cmd_spotlight_on").value
        self._cmd_spotlight_off  = self.get_parameter("can.cmd_spotlight_off").value
        self._cmd_traffic_light  = self.get_parameter("can.cmd_traffic_light").value
        self._status_ok          = self.get_parameter("can.status_ok").value
        self._status_error       = self.get_parameter("can.status_error").value
        self._ack_timeout        = self.get_parameter("timeouts.ack_s").value
        self._cmd_beautiful_on  = self.get_parameter("can.cmd_beautiful_light_on").value
        self._cmd_beautiful_off = self.get_parameter("can.cmd_beautiful_light_off").value
        self._state_pub_rate    = float(self.get_parameter("state_pub_rate").value)

    # =======================================================================
    # State topic
    # =======================================================================

    def _publish_state(self):
        """Publish the current state. Also the heartbeat, on a timer.

        Republishing unchanged state is not redundancy: it is the only thing
        that tells a console the value on screen is still live. Without it a
        dead lights_can_node leaves the last latched state up forever. It also
        makes the topic usable from a VOLATILE subscriber - the ground station
        UI reaches it through rosbridge, which need not match TRANSIENT_LOCAL.
        """
        state = self._state
        self._state_dirty = False
        self._last_state_publish = self._now_seconds()

        msg = LightsStateMsg()
        msg.spotlight      = state.spotlight
        msg.beautiful      = state.beautiful
        msg.traffic_red    = state.traffic_red
        msg.traffic_yellow = state.traffic_yellow
        msg.traffic_green  = state.traffic_green
        msg.traffic_blue   = state.traffic_blue
        self._state_pub.publish(msg)

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _publish_change(self):
        """Push a real change out now instead of waiting for the heartbeat.

        Waiting costs up to a full period, which an operator sees as the
        console lagging the switch they just flipped.
        """
        if self._now_seconds() - self._last_state_publish < self._state_min_period:
            return
        self._publish_state()

    def _commit(self, state: LightsState):
        """Adopt a state the ESP32 has confirmed, and announce it."""
        if state == self._state:
            return
        self._state = state
        self._state_dirty = True
        self._publish_change()

    # =======================================================================
    # Services
    # =======================================================================

    def _on_spotlight_request(self, request, response):
        return self._apply_bool_light(
            "spotlight", lambda _state: request.data, response)

    def _on_spotlight_toggle(self, request, response):
        return self._apply_bool_light(
            "spotlight", lambda state: not state.spotlight, response)

    def _on_beautiful_request(self, request, response):
        return self._apply_bool_light(
            "beautiful", lambda _state: request.data, response)

    def _on_beautiful_toggle(self, request, response):
        return self._apply_bool_light(
            "beautiful", lambda state: not state.beautiful, response)

    def _apply_bool_light(self, field: str, desired_from, response):
        """Drive one on/off light.

        `desired_from` reads the target off the current state, which is what
        makes a toggle atomic: the read and the write both happen inside
        _can_lock, so two toggles racing cannot both see the same 'before'.
        """
        label, on_cmd, off_cmd = self._bool_lights[field]

        with self._can_lock:
            desired = bool(desired_from(self._state))
            state_str = "ON" if desired else "OFF"
            self.get_logger().info(f"Service call: {label} {state_str}")

            cmd = on_cmd if desired else off_cmd
            ok, status = self._send_and_wait(cmd, [cmd])

            if ok and status == self._status_ok:
                self._commit(replace(self._state, **{field: desired}))
                response.success = True
                response.message = f"{label} {state_str} OK"
            else:
                response.success = False
                response.message = (
                    f"{label} {state_str} FAIL"
                    + (" (timeout)" if not ok else " (ESP32 error)")
                )

        return response

    def _on_traffic_request(self, request, response):
        with self._can_lock:
            try:
                desired = resolve_traffic(
                    self._state,
                    red=request.red,
                    yellow=request.yellow,
                    green=request.green,
                    blue=request.blue,
                )
            except ValueError as exc:
                response.success = False
                response.message = f"TRAFFIC_LIGHT bad request: {exc}"
                self.get_logger().warn(response.message)
                return response

            mask = traffic_mask(desired)
            self.get_logger().info(
                f"Service call: TRAFFIC_LIGHT mask=0x{mask:02X} "
                f"({describe_traffic(desired)})"
            )

            ok, status = self._send_and_wait(
                self._cmd_traffic_light, [self._cmd_traffic_light, mask])

            if ok and status == self._status_ok:
                self._commit(desired)
                response.success = True
                response.message = (
                    f"TRAFFIC_LIGHT OK mask=0x{mask:02X} ({describe_traffic(desired)})")
            else:
                response.success = False
                response.message = "TRAFFIC_LIGHT FAIL" + (
                    " (timeout)" if not ok else " (ESP32 error)")

        return response

    # =======================================================================
    # CAN helpers
    # =======================================================================

    def _send_and_wait(self, expected_cmd: int, data: list) -> tuple[bool, int | None]:
        """Send a CAN command and wait for ACK. Returns (got_reply, status).

        Callers hold _can_lock, so only one transaction is ever outstanding —
        _resp_pending_cmd belongs to this call and nobody else's.
        """
        with self._resp_lock:
            self._resp_pending_cmd = expected_cmd
            self._resp_status = None
            self._resp_event.clear()

        self._send_cmd(self._cmd_id, data)

        got_reply = self._resp_event.wait(timeout=self._ack_timeout)

        with self._resp_lock:
            status = self._resp_status
            self._resp_pending_cmd = None

        if not got_reply:
            self.get_logger().warn(f"No ACK from ESP32 (cmd=0x{expected_cmd:02X})")

        return got_reply, status

    def _send_cmd(self, can_id: int, data: list):
        msg = Frame()
        msg.id          = can_id
        msg.dlc         = len(data)
        msg.data        = data + [0] * (8 - len(data))
        msg.is_extended = False
        msg.is_rtr      = False
        msg.is_error    = False
        self._pub.publish(msg)
        self.get_logger().info(
            f"TX CAN 0x{can_id:03X}  data={[f'0x{b:02X}' for b in data]}"
        )

    def _on_can_msg(self, msg: Frame):
        if msg.id != self._resp_id or msg.dlc < 2:
            return

        cmd    = msg.data[0]
        status = msg.data[1]

        self.get_logger().info(
            f"RX CAN 0x{self._resp_id:03X}  cmd=0x{cmd:02X}  status=0x{status:02X}"
        )

        with self._resp_lock:
            if self._resp_pending_cmd is None or cmd != self._resp_pending_cmd:
                expected_str = (
                    f"0x{self._resp_pending_cmd:02X}"
                    if self._resp_pending_cmd is not None
                    else "None"
                )
                self.get_logger().warn(
                    f"Unexpected response cmd=0x{cmd:02X}, "
                    f"expected={expected_str}, ignoring"
                )
                return
            self._resp_status = status
            self._resp_event.set()


# ===========================================================================
# main
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    # Three groups, three threads: one service at a time, the CAN subscription
    # always free to land the ACK it is waiting on, and the state timer.
    executor = MultiThreadedExecutor(num_threads=3)
    node = LightsCanNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
