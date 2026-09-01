#!/usr/bin/env python3
"""Lights the tower red for autonomous, green for manual drive.

Watches the same command topics twist_mux does (cmd_vel_joy, cmd_vel_gs,
cmd_vel_nav, cmd_vel_ext, cmd_vel_lora) and picks the same winner it would,
using the priorities/timeouts mirrored from rover_bringup/config/twist_mux.yaml
in DriveSourceWatchdog - see that module for why this reimplements the rule
rather than reading it back off twist_mux's /diagnostics.

Drives lights/traffic_light (indomitus_interfaces/srv/SetTrafficLight),
setting red/green and leaving blue at KEEP so this never disturbs
gs_link_lamp_node's blue-on-GS-connection lamp.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from indomitus_interfaces.srv import SetTrafficLight

from rover_teleop.drive_source_watchdog import DriveSourceWatchdog, Source
from rover_teleop.service_call import GuardedCall

# Mirrors rover_bringup/config/twist_mux.yaml. Keep both in sync by hand -
# there is nothing at runtime enforcing the match, same contract as the LoRa
# wire scale between lora_rover_node and lora_gateway_node.
DEFAULT_SOURCES = (
    Source('cmd_vel_joy', priority=100, timeout=0.5, autonomous=False),
    Source('cmd_vel_gs', priority=50, timeout=0.5, autonomous=False),
    Source('cmd_vel_nav', priority=20, timeout=1.0, autonomous=True),
    Source('cmd_vel_ext', priority=10, timeout=0.5, autonomous=False),
    Source('cmd_vel_lora', priority=5, timeout=1.0, autonomous=False),
)

KEEP = SetTrafficLight.Request.KEEP
OFF = SetTrafficLight.Request.OFF
ON = SetTrafficLight.Request.ON


class DriveSourceLampNode(Node):

    def __init__(self):
        super().__init__('drive_source_lamp_node')

        self.declare_parameter('poll_rate_hz', 5.0)

        self.watchdog = DriveSourceWatchdog(DEFAULT_SOURCES)
        # Unknown until the first service call succeeds, so startup always
        # issues at least one SetTrafficLight rather than trusting the lamp's
        # current hardware state.
        self._commanded = None
        self._call = GuardedCall(
            self.create_client(SetTrafficLight, 'lights/traffic_light'))

        for source in DEFAULT_SOURCES:
            self.create_subscription(
                Twist, source.name, self._make_on_msg(source.name), 10)

        poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.create_timer(1.0 / poll_rate_hz, self._tick)

        self.get_logger().info(
            f'drive_source_lamp_node: watching '
            f'{", ".join(s.name for s in DEFAULT_SOURCES)}')

    def _make_on_msg(self, name):
        def _on_msg(_msg):
            self.watchdog.on_message(name)
        return _on_msg

    def _tick(self):
        autonomous = self.watchdog.autonomous()
        desired = {
            True: (ON, OFF),           # red on,  green off
            False: (OFF, ON),          # red off, green on
            None: (OFF, OFF),          # nobody driving - both off
        }[autonomous]

        if desired == self._commanded or self._call.pending:
            return

        red, green = desired
        request = SetTrafficLight.Request(red=red, green=green, blue=KEEP)
        if self._call.call(request, lambda future: self._on_set_done(future, desired)):
            return
        # Not ready yet (rover_lighting_node not up, CAN not up) - retried on
        # the next tick.

    def _on_set_done(self, future, requested):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'lights/traffic_light call failed: {exc!r}')
            return
        if not response.success:
            self.get_logger().warn(f'lights/traffic_light refused: {response.message}')
            return
        self._commanded = requested


def main():
    rclpy.init()
    node = DriveSourceLampNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
