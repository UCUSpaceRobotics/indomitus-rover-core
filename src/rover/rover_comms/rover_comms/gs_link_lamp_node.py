#!/usr/bin/env python3
"""Lights the tower's blue LED while the ground station is reachable.

Subscribes to /gs/link/state, which link_status_node on the ground station
(indomitus-ground-station, gs_comms) publishes at 2 Hz whenever gs_comms is
running - rover and ground station share a DDS domain (see docker-compose
ROS_DOMAIN_ID / fastdds_rover_link.xml on both sides), so no rosbridge or new
cross-repo channel is needed for this to reach the rover. See
gs_link_watchdog.py for the connected/disconnected rule.

Drives lights/traffic_blue (std_srvs/SetBool), the single-purpose per-lamp
service rover_lighting_node already exposes - no new lamp interface needed.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rover_comms.gs_link_watchdog import GsLinkWatchdog


class GsLinkLampNode(Node):

    def __init__(self):
        super().__init__('gs_link_lamp_node')

        self.declare_parameter('gs_link_state_topic', '/gs/link/state')
        # link_status_node publishes at 2 Hz; 4 missed samples rides out a
        # single dropped DDS packet without flickering the lamp, while still
        # catching "the console just closed" within a couple of seconds.
        self.declare_parameter('timeout_sec', 2.0)
        self.declare_parameter('poll_rate_hz', 2.0)

        self.watchdog = GsLinkWatchdog(
            timeout=float(self.get_parameter('timeout_sec').value))
        # Unknown until the first service call succeeds, so startup always
        # issues at least one SetBool rather than assuming the lamp's current
        # hardware state matches.
        self._lamp_on = None
        self._call_pending = False

        topic = self.get_parameter('gs_link_state_topic').value
        self.create_subscription(String, topic, self._on_state, 10)
        self.cli = self.create_client(SetBool, 'lights/traffic_blue')

        poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.create_timer(1.0 / poll_rate_hz, self._tick)

        self.get_logger().info(
            f'gs_link_lamp_node: watching {topic}, '
            f'timeout {self.watchdog.timeout}s')

    def _on_state(self, msg):
        self.watchdog.on_message(msg.data)

    def _tick(self):
        connected = self.watchdog.connected()
        if connected == self._lamp_on or self._call_pending:
            return
        if not self.cli.service_is_ready():
            # Retried on the next tick - rover_lighting_node coming up after
            # this node, or CAN not up yet, must not be fatal here.
            return
        self._call_pending = True
        self.cli.call_async(SetBool.Request(data=connected)).add_done_callback(
            lambda future: self._on_set_done(future, connected))

    def _on_set_done(self, future, requested):
        self._call_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'lights/traffic_blue call failed: {exc!r}')
            return
        if not response.success:
            self.get_logger().warn(
                f'lights/traffic_blue refused: {response.message}')
            return
        self._lamp_on = requested


def main():
    rclpy.init()
    node = GsLinkLampNode()
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
