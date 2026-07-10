#!/usr/bin/env python3
"""
ROS2 node - power (voltage/current) monitoring via CAN bus (ros2_socketcan).

Listens on /from_can_bus for frames coming from the two power sensors and
republishes decoded voltage/current as sensor_msgs/msg/BatteryState.

CAN RX (ESP32 -> PC), per sensor (INA228, one frame per sensor):
  ID sensor_can_id   byte 0-3 = current (float32, little-endian, amps)
                      byte 4-7 = voltage (float32, little-endian, volts)
  Values are already physical units (ESP converts via INA228 LSB), no
  scaling needed on this side.

Publishes:
  <topic_prefix>/<sensor_name>   sensor_msgs/msg/BatteryState
    .voltage  -> volts
    .current  -> amps
    .location -> sensor_name (identifies which physical sensor this is)
"""

from dataclasses import dataclass

import math
import struct

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from can_msgs.msg import Frame
from sensor_msgs.msg import BatteryState


@dataclass
class SensorConfig:
    name: str
    can_id: int
    publisher: object = None


class PowerMonitorCanNode(Node):
    def __init__(self):
        super().__init__("power_monitor_can_node")

        self._declare_parameters()
        self._load_parameters()

        self._sub_cbg = MutuallyExclusiveCallbackGroup()

        self._can_sub = self.create_subscription(
            Frame, "/from_can_bus", self._on_can_msg, 10,
            callback_group=self._sub_cbg,
        )

        # --- one publisher per sensor, indexed by CAN id for O(1) lookup ---
        self._sensors_by_id = {}
        for cfg in self._sensor_configs:
            cfg.publisher = self.create_publisher(
                BatteryState,
                f"{self._topic_prefix}/{cfg.name}",
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self._sensors_by_id[cfg.can_id] = cfg

        sensors_desc = "\n".join(
            f"    0x{cfg.can_id:03X} -> {self._topic_prefix}/{cfg.name}"
            for cfg in self._sensor_configs
        )
        self.get_logger().info(
            f"PowerMonitorCanNode ready, listening on /from_can_bus\n"
            f"  sensors:\n{sensors_desc}"
        )


    def _declare_parameters(self):
        self.declare_parameter("topic_prefix", "power_monitor")
        self.declare_parameter("sensor_names", ["sensor_rover", "sensor_arm"])
        self.declare_parameter("sensor_can_ids", [0x302, 0x303])

    def _load_parameters(self):
        self._topic_prefix = self.get_parameter("topic_prefix").value
        names = self.get_parameter("sensor_names").value
        ids = self.get_parameter("sensor_can_ids").value

        if len(names) != len(ids):
            raise ValueError(
                f"sensor_names ({len(names)}) and sensor_can_ids ({len(ids)}) "
                f"must have the same length"
            )

        self._sensor_configs = [
            SensorConfig(name=n, can_id=i) for n, i in zip(names, ids)
        ]


    def _on_can_msg(self, msg: Frame):
        cfg = self._sensors_by_id.get(msg.id)
        if cfg is None:
            return  # not a frame we care about

        if msg.dlc < 8:
            self.get_logger().warn(
                f"RX CAN 0x{msg.id:03X} ({cfg.name}): dlc={msg.dlc} < 8, ignoring"
            )
            return

        current, voltage = struct.unpack_from("<ff", bytes(msg.data), 0)

        if not (math.isfinite(current) and math.isfinite(voltage)):
            self.get_logger().warn(
                f"RX CAN 0x{msg.id:03X} ({cfg.name}): non-finite value, ignoring"
            )
            return

        self.get_logger().debug(
            f"RX CAN 0x{msg.id:03X} ({cfg.name})  "
            f"voltage={voltage:.3f} V  current={current:.3f} A"
        )

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = float(voltage)
        state.current = float(current)
        state.location = cfg.name
        state.present = True
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

        cfg.publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    node = PowerMonitorCanNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
