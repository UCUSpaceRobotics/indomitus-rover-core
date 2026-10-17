#!/usr/bin/env python3
"""Lights the tower red for autonomous, green for manual drive.

Watches the same command topics twist_mux does and picks the same winner it
would. Topics, priorities and timeouts are read from the twist_mux config
itself (parameter twist_mux_config, default this package's
config/twist_mux.yaml, the file twist_mux.launch.py loads); which of them count
as autonomous is the autonomous_topics parameter. See DriveSourceWatchdog for
why this reimplements the rule rather than reading it back off twist_mux's
/diagnostics.

Drives lights/traffic_light (indomitus_interfaces/srv/SetTrafficLight),
setting red/green and leaving blue at KEEP so this never disturbs
gs_link_lamp_node's blue-on-GS-connection lamp.
"""

import os

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from indomitus_interfaces.srv import SetTrafficLight

from rover_drive.drive_source_watchdog import DriveSourceWatchdog, sources_from_twist_mux
from rover_drive.service_call import GuardedCall

KEEP = SetTrafficLight.Request.KEEP
OFF = SetTrafficLight.Request.OFF
ON = SetTrafficLight.Request.ON


class DriveSourceLampNode(Node):

    def __init__(self):
        super().__init__('drive_source_lamp_node')

        self.declare_parameter('poll_rate_hz', 5.0)
        self.declare_parameter('twist_mux_config', os.path.join(
            get_package_share_directory('rover_drive'), 'config', 'twist_mux.yaml'))
        self.declare_parameter('autonomous_topics', ['cmd_vel_nav'])

        config_path = str(self.get_parameter('twist_mux_config').value)
        with open(config_path) as f:
            sources = sources_from_twist_mux(
                yaml.safe_load(f),
                self.get_parameter('autonomous_topics').value)

        self.watchdog = DriveSourceWatchdog(sources)
        # Unknown until the first service call succeeds, so startup always
        # issues at least one SetTrafficLight rather than trusting the lamp's
        # current hardware state.
        self._commanded = None
        self._call = GuardedCall(
            self.create_client(SetTrafficLight, 'lights/traffic_light'))

        for source in sources:
            self.create_subscription(
                Twist, source.name, self._make_on_msg(source.name), 10)

        poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.create_timer(1.0 / poll_rate_hz, self._tick)

        self.get_logger().info(
            f'drive_source_lamp_node: watching '
            f'{", ".join(s.name for s in sources)} (from {config_path})')

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
