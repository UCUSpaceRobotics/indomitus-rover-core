#!/usr/bin/env python3
"""
collision_link_reporter — logs WHICH links are in collision, by name.

move_group and moveit_servo are prebuilt binary ROS packages (not vendored
in this repo), so their own collision-check code can't be patched to print
link names directly. This node gets the same information a different way:
it watches /joint_states and polls move_group's own
/check_state_validity service (moveit_msgs/GetStateValidity) — the same
service /check_state_validity manual calls use — and logs the
contact_body_1/contact_body_2 pair whenever the current pose goes invalid.

Edge-triggered: logs once when a collision starts, once when it clears, not
every poll — so leaving the arm sitting in a bad pose doesn't spam the log.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

GROUP_NAME = 'indomitus_arm'
CHECK_PERIOD_SEC = 0.5


class CollisionLinkReporter(Node):
    def __init__(self):
        super().__init__('collision_link_reporter')
        self._latest_js = None
        self._in_collision = False
        self._client = self.create_client(GetStateValidity, 'check_state_validity')
        self.create_subscription(JointState, 'joint_states', self._on_js, 10)
        self.create_timer(CHECK_PERIOD_SEC, self._check)
        self.get_logger().info(
            f'collision_link_reporter ready — polling /check_state_validity '
            f'every {CHECK_PERIOD_SEC}s for group "{GROUP_NAME}".'
        )

    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _check(self):
        if self._latest_js is None or not self._client.service_is_ready():
            return
        req = GetStateValidity.Request()
        req.robot_state = RobotState(joint_state=self._latest_js)
        req.group_name = GROUP_NAME
        self._client.call_async(req).add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            resp = future.result()
        except Exception:
            return  # service call failed/dropped this cycle — just retry next tick

        if resp.valid:
            if self._in_collision:
                self._in_collision = False
                self.get_logger().info('Collision cleared.')
            return

        if not self._in_collision:
            self._in_collision = True
            pairs = {(c.contact_body_1, c.contact_body_2) for c in resp.contacts}
            if pairs:
                for a, b in sorted(pairs):
                    self.get_logger().error(f'COLLISION: {a} <-> {b}')
            else:
                # valid=False with no contacts happens for e.g. joint-limit
                # violations rather than link-link contact.
                self.get_logger().error(
                    'State invalid (no contact pair reported — check joint limits).'
                )


def main():
    rclpy.init()
    node = CollisionLinkReporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
