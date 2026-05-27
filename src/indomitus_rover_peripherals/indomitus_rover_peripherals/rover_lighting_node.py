#!/usr/bin/env python3
"""
ROS2 node - lights control via CAN bus (ros2_socketcan).

Service: /lights/spotlight  (std_srvs/srv/SetLights)
  request:  bool data   -> True = on, False = off
  response: bool success, string message

Service: /lights/traffic_light  (indomitus_interfaces/srv/SetTrafficLight)
  request:  bool red, bool yellow, bool green, bool blue
  response: bool success, string message

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

from can_msgs.msg import Frame
from std_srvs.srv import SetBool
from indomitus_interfaces.srv import SetTrafficLight

import threading


class LightsCanNode(Node):
    def __init__(self):
        super().__init__("lights_can_node")

        self._declare_parameters()
        self._load_parameters()

        # --- callback groups ---
        self._sub_cbg      = MutuallyExclusiveCallbackGroup()
        self._spotlight_cbg = MutuallyExclusiveCallbackGroup()
        self._traffic_cbg  = MutuallyExclusiveCallbackGroup()
        self._beautiful_cbg = MutuallyExclusiveCallbackGroup()

        # --- CAN pub/sub ---
        self._pub = self.create_publisher(Frame, "/to_can_bus", 10)
        self._sub = self.create_subscription(
            Frame, "/from_can_bus", self._on_can_msg, 10,
            callback_group=self._sub_cbg,
        )

        # --- Services ---
        self._spotlight_srv = self.create_service(
            SetBool, "lights/spotlight", self._on_spotlight_request,
            callback_group=self._spotlight_cbg,
        )
        self._traffic_srv = self.create_service(
            SetTrafficLight, "lights/traffic_light", self._on_traffic_request,
            callback_group=self._traffic_cbg,
        )

        self._beautiful_srv = self.create_service(
            SetBool, "lights/beautiful", self._on_beautiful_request,
            callback_group=self._beautiful_cbg,
        )

        # --- response state ---
        self._resp_lock  = threading.Lock()
        self._resp_event = threading.Event()
        self._resp_pending_cmd: int | None = None
        self._resp_status: int | None = None

        self.get_logger().info(
            f"LightsCanNode ready\n"
            f"  CAN TX id=0x{self._cmd_id:03X}\n"
            f"  CAN RX id=0x{self._resp_id:03X}\n"
            f"  /lights/spotlight      (Service) - bool spotlight\n"
            f"  /lights/traffic_light  (Service) - bool red/yellow/green/blue\n"
            f"  /lights/beautiful      (Service) - bool beautiful light\n"
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

    # =======================================================================
    # Services
    # =======================================================================

    def _on_spotlight_request(self, request, response):
        cmd = self._cmd_spotlight_on if request.data else self._cmd_spotlight_off
        label = "ON" if request.data else "OFF"
        self.get_logger().info(f"Service call: SPOTLIGHT {label}")

        ok, status = self._send_and_wait(cmd, [cmd])

        if ok and status == self._status_ok:
            response.success = True
            response.message = f"SPOTLIGHT {label} OK"
        else:
            response.success = False
            response.message = f"SPOTLIGHT {label} FAIL" + (" (timeout)" if not ok else " (ESP32 error)")

        return response

    def _on_traffic_request(self, request, response):
        # bitmask: R=bit0, Y=bit1, G=bit2, B=bit3
        mask = (
            (0x01 if request.red    else 0) |
            (0x02 if request.yellow else 0) |
            (0x04 if request.green  else 0) |
            (0x08 if request.blue   else 0)
        )
        self.get_logger().info(
            f"Service call: TRAFFIC_LIGHT mask=0x{mask:02X} "
            f"(R={int(request.red)} Y={int(request.yellow)} "
            f"G={int(request.green)} B={int(request.blue)})"
        )

        ok, status = self._send_and_wait(self._cmd_traffic_light, [self._cmd_traffic_light, mask])

        if ok and status == self._status_ok:
            response.success = True
            response.message = f"TRAFFIC_LIGHT OK mask=0x{mask:02X}"
        else:
            response.success = False
            response.message = "TRAFFIC_LIGHT FAIL" + (" (timeout)" if not ok else " (ESP32 error)")

        return response

    def _on_beautiful_request(self, request, response):
        cmd = self._cmd_beautiful_on if request.data else self._cmd_beautiful_off
        label = "ON" if request.data else "OFF"
        self.get_logger().info(f"Service call: BEAUTIFUL_LIGHT {label}")

        ok, status = self._send_and_wait(cmd, [cmd])

        if ok and status == self._status_ok:
            response.success = True
            response.message = f"BEAUTIFUL_LIGHT {label} OK"
        else:
            response.success = False
            response.message = f"BEAUTIFUL_LIGHT {label} FAIL" + (" (timeout)" if not ok else " (ESP32 error)")

        return response

    # =======================================================================
    # CAN helpers
    # =======================================================================

    def _send_and_wait(self, expected_cmd: int, data: list) -> tuple[bool, int | None]:
        """Send a CAN command and wait for ACK. Returns (got_reply, status)."""
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
                self.get_logger().warn(
                    f"Unexpected response cmd=0x{cmd:02X}, "
                    f"expected=0x{self._resp_pending_cmd:02X}, ignoring"
                )
                return
            self._resp_status = status
            self._resp_event.set()


# ===========================================================================
# main
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)
    node = LightsCanNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
