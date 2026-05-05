#!/usr/bin/env python3
"""
can_hw_bridge — temporary replacement for ros2_socketcan.

Subscribes to /to_can_bus (can_msgs/Frame) and forwards every frame
to the physical CAN bus via cansend.

Startup:  setMode (register 10) + enable for all motors
Shutdown: velocity=0, steer=0, wait 1.5 s, disable all motors
"""
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from can_msgs.msg import Frame


class CanHwBridgeNode(Node):

    def __init__(self):
        super().__init__('can_hw_bridge')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('steer_ids',     [11, 13, 15, 17])
        self.declare_parameter('drive_ids',     [10, 12, 14, 16])

        self._iface      = self.get_parameter('can_interface').value
        self._steer_ids  = list(self.get_parameter('steer_ids').value)
        self._drive_ids  = list(self.get_parameter('drive_ids').value)

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._sub = self.create_subscription(Frame, '/to_can_bus', self._on_frame, qos)

        # Hardware init after 500 ms (one-shot timer)
        self._init_timer = self.create_timer(0.5, self._init_once)

        self.get_logger().info(
            f'can_hw_bridge started on {self._iface} | '
            f'steer={self._steer_ids} drive={self._drive_ids}')

    # ── CAN forwarding ────────────────────────────────────────────────────────

    def _on_frame(self, msg: Frame):
        data = bytes(msg.data[:msg.dlc])
        cmd = f'{self._iface} {msg.id:03X}#{data.hex().upper()}'
        ret = subprocess.run(['cansend'] + cmd.split(),
                             capture_output=True, text=True)
        if ret.returncode == 0:
            self.get_logger().info(f'TX  {cmd}')
        else:
            self.get_logger().error(f'cansend FAIL: {cmd} — {ret.stderr.strip()}')

    # ── Hardware lifecycle ────────────────────────────────────────────────────

    def _init_once(self):
        self._init_timer.cancel()
        self.hardware_init()

    def hardware_init(self):
        """Set mode + enable all motors."""
        self.get_logger().info('Hardware init: setMode + enable')
        for esc_id in self._steer_ids:
            self._send(0x7FF, bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF,
                                     0x55, 0x0A, 2, 0, 0, 0]))   # mode 2 = PositionVelocity
        for esc_id in self._drive_ids:
            self._send(0x7FF, bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF,
                                     0x55, 0x0A, 3, 0, 0, 0]))   # mode 3 = Velocity
        time.sleep(0.05)
        for esc_id in self._steer_ids + self._drive_ids:
            self._send(esc_id, bytes([0xFF] * 7 + [0xFC]))        # enable
        self.get_logger().info('All motors enabled')

    def hardware_stop(self):
        """Zero commands → 1.5 s settle → disable all motors."""
        self.get_logger().info('Stopping: sending zero commands')
        zero_vel = struct.pack('<f', 0.0)
        zero_pos = struct.pack('<ff', 0.0, 5.0)   # pos=0 rad, vmax=5 rad/s

        for esc_id in self._drive_ids:
            self._send(0x200 + esc_id, zero_vel)
        for esc_id in self._steer_ids:
            self._send(0x100 + esc_id, zero_pos)

        self.get_logger().info('Waiting 1.5 s for motion to settle...')
        time.sleep(1.5)

        self.get_logger().info('Disabling all motors')
        for esc_id in self._steer_ids + self._drive_ids:
            self._send(esc_id, bytes([0xFF] * 7 + [0xFD]))        # disable
        self.get_logger().info('All motors disabled')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send(self, can_id: int, data: bytes) -> bool:
        cmd = f'{self._iface} {can_id:03X}#{data.hex().upper()}'
        ret = subprocess.run(['cansend'] + cmd.split(),
                             capture_output=True, text=True)
        ok = ret.returncode == 0
        level = self.get_logger().info if ok else self.get_logger().error
        level(f'cansend {"OK" if ok else "FAIL"}  {cmd}')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = CanHwBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hardware_stop()
        node.destroy_node()
        rclpy.shutdown()
