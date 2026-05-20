#!/usr/bin/env python3
"""
ROS2 node — container control via CAN bus (ros2_socketcan).

Action:  /container/lid  (indomitus_interfaces/action/ContainerLid)
  goal.open = True    → open lid   (CAN 0x200, byte 0 = 0x01)
  goal.open = False   → close lid  (CAN 0x200, byte 0 = 0x02)
  feedback.status     → "opening" | "closing" | "done" | "error"
  result.success      → True if ESP32 replied OK within timeout
  result.message      → human-readable status

Service: /container/weight  (indomitus_interfaces/srv/GetWeight)
  request:  (empty)
  response: float32 weight, bool success, string message

CAN TX (PC -> ESP32)  ID 0x200:
  byte 0 = 0x01 open | 0x02 close | 0x10 get_weight

CAN RX (ESP32 -> PC):
  ID 0x201  byte 0 = echo cmd, byte 1 = 0x00 OK | 0x01 FAIL  (lid response)
  ID 0x202  bytes 0-3 = float32 weight little-endian           (weight response)
"""

import struct
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from can_msgs.msg import Frame
from indomitus_interfaces.action import ContainerLid
from indomitus_interfaces.srv import GetWeight

# ---------------------------------------------------------------------------
# CAN IDs
# ---------------------------------------------------------------------------
CMD_ID         = 0x200
LID_RESP_ID    = 0x201
WEIGHT_RESP_ID = 0x202

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
CMD_OPEN       = 0x01
CMD_CLOSE      = 0x02
CMD_GET_WEIGHT = 0x10

# ---------------------------------------------------------------------------
# Timeouts
# ---------------------------------------------------------------------------
LID_TIMEOUT_S    = 15.0   # lid open/close takes time
WEIGHT_TIMEOUT_S = 3.0    # weight sensor responds quickly


class ContainerCanNode(Node):
    def __init__(self):
        super().__init__("container_can_node")

        # --- callback groups ---
        self._sub_cbg    = MutuallyExclusiveCallbackGroup()
        self._action_cbg = MutuallyExclusiveCallbackGroup()
        self._weight_cbg = MutuallyExclusiveCallbackGroup()

        # --- CAN pub/sub ---
        self._pub = self.create_publisher(Frame, "/to_can_bus", 10)
        self._sub = self.create_subscription(
            Frame, "/from_can_bus", self._on_can_msg, 10,
            callback_group=self._sub_cbg,
        )

        # --- Action server: lid open/close ---
        self._lid_action = ActionServer(
            self,
            ContainerLid,
            "container/lid",
            goal_callback=self._lid_goal_cb,
            cancel_callback=self._lid_cancel_cb,
            execute_callback=self._lid_execute_cb,
            callback_group=self._action_cbg,
        )

        # --- Service: weight ---
        self._weight_srv = self.create_service(
            GetWeight, "container/weight", self._on_weight_request,
            callback_group=self._weight_cbg,
        )

        # --- waiting response from ESP32 ---
        # lid
        self._lid_lock    = threading.Lock()
        self._lid_pending: int | None = None
        self._lid_event   = threading.Event()
        self._lid_ok      = False

        # weight
        self._wgt_lock    = threading.Lock()
        self._wgt_pending = False
        self._wgt_event   = threading.Event()
        self._wgt_value: float | None = None

        self.get_logger().info(
            "ContainerCanNode ready\n"
            "  /container/lid     (Action)  — goal.open: true=open, false=close\n"
            "  /container/weight  (Service) — get weight sensor reading"
        )

    # =======================================================================
    # Action: lid
    # =======================================================================

    def _lid_goal_cb(self, goal_handle):
        self.get_logger().info(
            f"Action goal received: {'OPEN' if goal_handle.open else 'CLOSE'}"
        )
        return GoalResponse.ACCEPT

    def _lid_cancel_cb(self, goal_handle):
        self.get_logger().info("Action cancel requested")
        return CancelResponse.ACCEPT

    def _lid_execute_cb(self, goal_handle):
        open_lid = goal_handle.request.open
        cmd      = CMD_OPEN if open_lid else CMD_CLOSE
        label    = "opening" if open_lid else "closing"

        feedback = ContainerLid.Feedback()
        feedback.status = label
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"Action execute: {label.upper()}")

        with self._lid_lock:
            self._lid_pending = cmd
            self._lid_event.clear()

        self._send_cmd(CMD_ID, [cmd])

        got_reply = self._lid_event.wait(timeout=LID_TIMEOUT_S)

        with self._lid_lock:
            self._lid_pending = None
            ok = self._lid_ok if got_reply else False

        result = ContainerLid.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = f"{label.upper()} CANCELED"
            return result

        if not got_reply:
            self.get_logger().warn(f"Timeout waiting for ESP32 lid response (cmd=0x{cmd:02X})")
            feedback.status = "error"
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            result.success = False
            result.message = f"{label.upper()} FAIL (timeout)"
            return result

        if not ok:
            feedback.status = "error"
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            result.success = False
            result.message = f"{label.upper()} FAIL (ESP32 error)"
            return result

        feedback.status = "done"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = f"{label.upper()} OK"
        return result

    # =======================================================================
    # Service: weight
    # =======================================================================

    def _on_weight_request(self, request, response):
        self.get_logger().info("Service call: GET_WEIGHT")

        with self._wgt_lock:
            self._wgt_pending = True
            self._wgt_event.clear()
            self._wgt_value = None

        self._send_cmd(CMD_ID, [CMD_GET_WEIGHT])

        got_reply = self._wgt_event.wait(timeout=WEIGHT_TIMEOUT_S)

        with self._wgt_lock:
            self._wgt_pending = False
            value = self._wgt_value

        if not got_reply or value is None:
            self.get_logger().warn("Timeout waiting for weight response")
            response.success = False
            response.message = "FAIL (timeout)"
            response.weight  = 0.0
        else:
            response.success = True
            response.message = "OK"
            response.weight  = value
            self.get_logger().info(f"Weight: {value:.3f}")

        return response

    # =======================================================================
    # CAN helpers
    # =======================================================================

    def _send_cmd(self, can_id: int, data: list):
        msg = Frame()
        msg.id  = can_id
        msg.dlc = len(data)
        msg.data = data + [0] * (8 - len(data))
        msg.is_extended = False
        msg.is_rtr      = False
        msg.is_error    = False
        self._pub.publish(msg)
        self.get_logger().info(
            f"TX CAN 0x{can_id:03X}  data={[f'0x{b:02X}' for b in data]}"
        )

    def _on_can_msg(self, msg: Frame):
        if msg.id == LID_RESP_ID and msg.dlc >= 2:
            self._handle_lid_response(msg)
        elif msg.id == WEIGHT_RESP_ID and msg.dlc >= 4:
            self._handle_weight_response(msg)

    def _handle_lid_response(self, msg: Frame):
        cmd    = msg.data[0]
        status = msg.data[1]
        ok     = status == 0x00

        self.get_logger().info(
            f"RX CAN 0x{LID_RESP_ID:03X}  cmd=0x{cmd:02X}  "
            f"status={'OK' if ok else 'FAIL'}"
        )

        with self._lid_lock:
            if self._lid_pending == cmd:
                self._lid_ok = ok
                self._lid_event.set()

    def _handle_weight_response(self, msg: Frame):
        # ESP32 надсилає float32 little-endian в байтах 0-3
        raw   = bytes(msg.data[:4])
        value = struct.unpack("<f", raw)[0]

        self.get_logger().info(
            f"RX CAN 0x{WEIGHT_RESP_ID:03X}  weight={value:.3f}"
        )

        with self._wgt_lock:
            if self._wgt_pending:
                self._wgt_value = value
                self._wgt_event.set()


# ===========================================================================
# main
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    # потрібно мінімум 3 threads: sub_cbg + action_cbg + weight_cbg
    executor = MultiThreadedExecutor(num_threads=4)
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
