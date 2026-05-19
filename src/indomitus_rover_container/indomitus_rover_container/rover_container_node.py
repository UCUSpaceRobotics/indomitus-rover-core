#!/usr/bin/env python3
"""
ROS2 node — container lid control via CAN bus (ros2_socketcan).

Service:  /container_lid  (std_srvs/srv/SetBool)
  request.data = True   → open lid   (CAN 0x200, byte 0 = 0x01)
  request.data = False  → close lid  (CAN 0x200, byte 0 = 0x02)
  response.success      → True if ESP32 replied OK within timeout
  response.message      → human-readable status

Extra service:
  /container_lid/work_done  (std_srvs/srv/Trigger) — send CMD_WORK_DONE to ESP32

CAN ID 0x201 from ESP32:
  byte 0 = echo of cmd sent
  byte 1 = 0x00 OK | 0x01 FAIL
"""

import threading
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_srvs.srv import SetBool, Trigger

CMD_ID  = 0x200
RESP_ID = 0x201

CMD_OPEN      = 0x01
CMD_CLOSE     = 0x02
CMD_WORK_DONE = 0x03

RESP_TIMEOUT_S = 1.0


class ContainerCanNode(Node):
    def __init__(self):
        super().__init__("container_can_node")

        self._pub = self.create_publisher(Frame, "/to_can_bus", 10)
        self._sub = self.create_subscription(
            Frame, "/from_can_bus", self._on_can_msg, 10
        )

        self._srv = self.create_service(
            SetBool, "container_lid", self._on_lid_request
        )
        self._work_done_srv = self.create_service(
            Trigger, "container_lid/work_done", self._on_work_done_request
        )

        # pending response state
        self._resp_lock = threading.Lock()
        self._pending_cmd: int | None = None
        self._resp_event = threading.Event()
        self._resp_ok: bool = False

        self.get_logger().info(
            "ContainerCanNode ready\n"
            "  /container_lid           (SetBool) — True=open, False=close\n"
            "  /container_lid/work_done (Trigger) — send work-done signal"
        )

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _on_lid_request(self, request: SetBool.Request, response: SetBool.Response):
        cmd = CMD_OPEN if request.data else CMD_CLOSE
        label = "OPEN" if request.data else "CLOSE"

        self.get_logger().info(f"Service call: {label}")
        ok = self._send_and_wait(cmd)

        response.success = ok
        response.message = f"{label} {'OK' if ok else 'FAIL (timeout or ESP32 error)'}"
        return response

    def _on_work_done_request(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Service call: WORK_DONE")
        ok = self._send_and_wait(CMD_WORK_DONE)

        response.success = ok
        response.message = "WORK_DONE " + ("OK" if ok else "FAIL")
        return response

    # ------------------------------------------------------------------
    # CAN send + wait for response
    # ------------------------------------------------------------------

    def _send_and_wait(self, cmd: int) -> bool:
        with self._resp_lock:
            self._pending_cmd = cmd
            self._resp_event.clear()

        self._send_cmd(cmd)

        got_reply = self._resp_event.wait(timeout=RESP_TIMEOUT_S)

        with self._resp_lock:
            self._pending_cmd = None
            result = self._resp_ok if got_reply else False

        if not got_reply:
            self.get_logger().warn(
                f"Timeout waiting for ESP32 response to cmd=0x{cmd:02X}"
            )

        return result

    def _send_cmd(self, cmd: int):
        msg = Frame()
        msg.id = CMD_ID
        msg.dlc = 1
        msg.data = [cmd, 0, 0, 0, 0, 0, 0, 0]
        msg.is_extended = False
        msg.is_rtr = False
        msg.is_error = False
        self._pub.publish(msg)
        self.get_logger().info(f"TX CAN 0x{CMD_ID:03X}  cmd=0x{cmd:02X}")

    # ------------------------------------------------------------------
    # CAN receive
    # ------------------------------------------------------------------

    def _on_can_msg(self, msg: Frame):
        if msg.id != RESP_ID or msg.dlc < 2:
            return

        cmd    = msg.data[0]
        status = msg.data[1]
        ok     = status == 0x00

        self.get_logger().info(
            f"RX CAN 0x{RESP_ID:03X}  cmd=0x{cmd:02X}  "
            f"status={'OK' if ok else 'FAIL'}"
        )

        with self._resp_lock:
            if self._pending_cmd == cmd:
                self._resp_ok = ok
                self._resp_event.set()


def main(args=None):
    rclpy.init(args=args)
    # MultiThreadedExecutor потрібен щоб сервіс і підписка працювали паралельно
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    node = ContainerCanNode()
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