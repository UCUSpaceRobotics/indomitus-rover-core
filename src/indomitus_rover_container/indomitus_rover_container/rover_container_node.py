#!/usr/bin/env python3
"""
ROS2 node - container control via CAN bus (ros2_socketcan).

Action:  /container/lid  (indomitus_interfaces/action/ContainerLid)
  goal.open = True  -> open lid   (CAN cmd_id, byte 0 = cmd_open)
  goal.open = False -> close lid  (CAN cmd_id, byte 0 = cmd_close)
  feedback.status   -> "opening" | "closing" | "done" | "error"
  result.success    -> True if ESP32 finished OK within timeout
  result.message    -> human-readable status

Service: /container/weight  (indomitus_interfaces/srv/GetWeight)
  request:  (empty)
  response: float32 weight, bool success, string message

CAN TX (PC -> ESP32)  ID cmd_id:
  byte 0 = cmd_open | cmd_close | cmd_poll_status | cmd_get_weight

CAN RX (ESP32 -> PC):
  ID lid_resp_id    byte 0 = echo cmd, byte 1 = status
    status: 0x00 ACK | 0x01 IN_PROGRESS | 0x02 DONE | 0x03 ERROR
  ID weight_resp_id bytes 0-3 = float32 weight little-endian
"""

import struct
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from can_msgs.msg import Frame
from indomitus_interfaces.action import ContainerLid
from indomitus_interfaces.srv import GetWeight


class ContainerCanNode(Node):
    def __init__(self):
        super().__init__("container_can_node")

        self._declare_parameters()
        self._load_parameters()

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

        # --- lid response ---
        self._lid_active = False
        self._lid_active_lock = threading.Lock()
        self._lid_lock   = threading.Lock()
        self._lid_event  = threading.Event()
        self._lid_status : int | None = None  # last received status

        # --- weight response ---
        self._wgt_lock    = threading.Lock()
        self._wgt_pending = False
        self._wgt_event   = threading.Event()
        self._wgt_value   : float | None = None

        self.get_logger().info(
            f"ContainerCanNode ready\n"
            f"  CAN TX id=0x{self._cmd_id:03X}  "
            f"open=0x{self._cmd_open:02X}  "
            f"close=0x{self._cmd_close:02X}  "
            f"poll=0x{self._cmd_poll_status:02X}  "
            f"weight=0x{self._cmd_get_weight:02X}\n"
            f"  CAN RX lid=0x{self._lid_resp_id:03X}  "
            f"weight=0x{self._weight_resp_id:03X}\n"
            f"  /container/lid    (Action)  - goal.open: true=open, false=close\n"
            f"  /container/weight (Service) - get weight sensor reading"
        )

    # =======================================================================
    # Parameters
    # =======================================================================
 
    def _declare_parameters(self):
        # CAN IDs
        self.declare_parameter("can.cmd_id",          0x200)
        self.declare_parameter("can.lid_resp_id",     0x201)
        self.declare_parameter("can.weight_resp_id",  0x202)
        # Commands
        self.declare_parameter("can.cmd_open",        0x01)
        self.declare_parameter("can.cmd_close",       0x02)
        self.declare_parameter("can.cmd_poll_status", 0x04)
        self.declare_parameter("can.cmd_get_weight",  0x10)
        # Status codes (ESP32 → PC, byte 1)
        self.declare_parameter("can.status_ack",         0x00)
        self.declare_parameter("can.status_in_progress", 0x01)
        self.declare_parameter("can.status_done",        0x02)
        self.declare_parameter("can.status_error",       0x03)
        # Timeouts / intervals
        self.declare_parameter("timeouts.lid_ack_s",           2.0)
        self.declare_parameter("timeouts.lid_done_s",          15.0)
        self.declare_parameter("timeouts.lid_poll_interval_s", 0.5)
        self.declare_parameter("timeouts.weight_s",            3.0)
 
    def _load_parameters(self):
        self._cmd_id          = self.get_parameter("can.cmd_id").value
        self._lid_resp_id     = self.get_parameter("can.lid_resp_id").value
        self._weight_resp_id  = self.get_parameter("can.weight_resp_id").value
        self._cmd_open        = self.get_parameter("can.cmd_open").value
        self._cmd_close       = self.get_parameter("can.cmd_close").value
        self._cmd_poll_status = self.get_parameter("can.cmd_poll_status").value
        self._cmd_get_weight  = self.get_parameter("can.cmd_get_weight").value
        self._status_ack         = self.get_parameter("can.status_ack").value
        self._status_in_progress = self.get_parameter("can.status_in_progress").value
        self._status_done        = self.get_parameter("can.status_done").value
        self._status_error       = self.get_parameter("can.status_error").value
        self._lid_ack_timeout    = self.get_parameter("timeouts.lid_ack_s").value
        self._lid_done_timeout   = self.get_parameter("timeouts.lid_done_s").value
        self._lid_poll_interval  = self.get_parameter("timeouts.lid_poll_interval_s").value
        self._weight_timeout     = self.get_parameter("timeouts.weight_s").value

    # =======================================================================
    # Action: lid
    # =======================================================================

    def _lid_goal_cb(self, goal_handle):
        with self._lid_active_lock:
            if self._lid_active:
                self.get_logger().warn("Lid action is already in progress, rejecting new goal")
                return GoalResponse.REJECT
            self._lid_active = True
        self.get_logger().info("Action goal received")
        return GoalResponse.ACCEPT

    def _lid_cancel_cb(self, goal_handle):
        self.get_logger().info("Action cancel requested")
        return CancelResponse.ACCEPT
    
    def _lid_execute_cb(self, goal_handle):
        try:
            return self._lid_execute_impl(goal_handle)
        finally:
            with self._lid_active_lock:
                self._lid_active = False

    def _lid_execute_impl(self, goal_handle):
        open_lid = goal_handle.request.open
        cmd      = self._cmd_open if open_lid else self._cmd_close
        label    = "opening" if open_lid else "closing"

        self.get_logger().info(f"Action execute: {label.upper()}")

        feedback = ContainerLid.Feedback()
        result   = ContainerLid.Result()

        # --- step 1: send command and wait for ACK ---
        feedback.status = label
        goal_handle.publish_feedback(feedback)

        with self._lid_lock:
            self._lid_status = None
            self._lid_event.clear()

        self._send_cmd(self._cmd_id, [cmd])

        got_ack = self._lid_event.wait(timeout=self._lid_ack_timeout)

        with self._lid_lock:
            ack_status = self._lid_status

        if not got_ack or ack_status not in (self._status_ack, self._status_in_progress):
            self.get_logger().warn(f"No ACK from ESP32 (cmd=0x{cmd:02X})")
            feedback.status = "error"
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            result.success = False
            result.message = f"{label.upper()} FAIL (no ACK)"
            return result

        self.get_logger().info(f"ACK received, polling every {self._lid_poll_interval}s")

        # --- step 2: polling until DONE or ERROR ---
        deadline = time.monotonic() + self._lid_done_timeout

        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"{label.upper()} CANCELED"
                return result

            time.sleep(self._lid_poll_interval)

            with self._lid_lock:
                self._lid_status = None
                self._lid_event.clear()

            self._send_cmd(self._cmd_id, [self._cmd_poll_status])

            got_reply = self._lid_event.wait(timeout=self._lid_ack_timeout)

            with self._lid_lock:
                current_status = self._lid_status

            if not got_reply or current_status is None:
                self.get_logger().warn("Poll timeout — no response from ESP32")
                continue

            if current_status == self._status_done:
                feedback.status = "done"
                goal_handle.publish_feedback(feedback)
                goal_handle.succeed()
                result.success = True
                result.message = f"{label.upper()} OK"
                return result

            if current_status == self._status_error:
                feedback.status = "error"
                goal_handle.publish_feedback(feedback)
                goal_handle.abort()
                result.success = False
                result.message = f"{label.upper()} FAIL (ESP32 error)"
                return result

            # STATUS_IN_PROGRESS — wait more
            self.get_logger().info(f"Still {label}...")

        # exceeded deadline
        self.get_logger().warn(f"Deadline exceeded waiting for {label.upper()} DONE")
        feedback.status = "error"
        goal_handle.publish_feedback(feedback)
        goal_handle.abort()
        result.success = False
        result.message = f"{label.upper()} FAIL (timeout)"
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

        self._send_cmd(self._cmd_id, [self._cmd_get_weight])

        got_reply = self._wgt_event.wait(timeout=self._weight_timeout)

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
        if msg.id == self._lid_resp_id and msg.dlc >= 2:
            self._handle_lid_response(msg)
        elif msg.id == self._weight_resp_id and msg.dlc >= 4:
            self._handle_weight_response(msg)

    def _handle_lid_response(self, msg: Frame):
        cmd    = msg.data[0]
        status = msg.data[1]

        self.get_logger().info(
            f"RX CAN 0x{self._lid_resp_id:03X}  "
            f"cmd=0x{cmd:02X}  status=0x{status:02X}"
        )

        with self._lid_lock:
            self._lid_status = status
            self._lid_event.set()

    def _handle_weight_response(self, msg: Frame):
        raw   = bytes(msg.data[:4])
        value = struct.unpack("<f", raw)[0]

        self.get_logger().info(
            f"RX CAN 0x{self._weight_resp_id:03X}  weight={value:.3f}"
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
