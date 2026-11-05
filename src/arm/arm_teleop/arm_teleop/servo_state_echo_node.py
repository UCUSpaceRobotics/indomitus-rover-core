#!/usr/bin/env python3
"""Feed moveit_servo its own last command as the arm's current state.

Workaround for teleop drift; the why and the rules are in command_state.py.

  in   joint_states                                       (real, from JSB)
  in   indomitus_arm_forward_position_controller/commands (what Servo sent)
  out  servo_node/joint_states_cmd                        (servo.yaml joint_topic
                                                           is pointed here by
                                                           arm.launch.py)

Publishes on every real joint_states message and right after every command,
so Servo's next cycle already sees the point it just sent. Polls
controller_manager/list_controllers to know whether the forward controller
owns the joints; if that service is unreachable, it falls back to passing the
real state through - exactly Servo's behaviour without this node.
"""

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from arm_teleop.command_state import CommandState

# Same order as indomitus_arm_forward_position_controller's `joints`
# (arm_moveit_config/config/ros2_controllers.yaml) - its command array is
# indexed by it.
DEFAULT_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]


class ServoStateEcho(Node):

    def __init__(self):
        super().__init__('servo_state_echo')
        self.declare_parameter('joints', DEFAULT_JOINTS)
        # Above the lag teleop normally runs with (slew limit + MIT sag, a few
        # hundredths of a rad), below anything that would be a visible jump
        # if Servo were re-based onto the real pose.
        self.declare_parameter('max_lead_rad', 0.15)
        self.declare_parameter('forward_controller', 'indomitus_arm_forward_position_controller')
        self.declare_parameter('controller_poll_hz', 2.0)

        joints = list(self.get_parameter('joints').value)
        self._forward = self.get_parameter('forward_controller').value
        self._state = CommandState(joints, self.get_parameter('max_lead_rad').value)
        self._last_js = None
        self._cm_missing_logged = False

        self._pub = self.create_publisher(JointState, 'servo_node/joint_states_cmd', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)
        self.create_subscription(
            Float64MultiArray, f'{self._forward}/commands', self._on_command, 10)

        self._list_client = self.create_client(ListControllers, 'controller_manager/list_controllers')
        self._poll_pending = False
        self.create_timer(1.0 / float(self.get_parameter('controller_poll_hz').value), self._poll)

        self.get_logger().info(
            f'servo_state_echo: Servo integrates from its last command on {len(joints)} joints '
            f'while {self._forward} is active (max lead {self._state.max_lead_rad:.2f} rad)')

    # -- controller state -----------------------------------------------------

    def _poll(self):
        if self._poll_pending:
            return
        if not self._list_client.service_is_ready():
            if not self._cm_missing_logged:
                # Normal for the first seconds after launch, before the
                # controller_manager is up; harmless either way.
                self.get_logger().info(
                    'controller_manager/list_controllers not available - passing the real '
                    'joint state to Servo until it is')
                self._cm_missing_logged = True
            self._state.set_forward_active(False)
            return
        self._cm_missing_logged = False
        self._poll_pending = True
        self._list_client.call_async(ListControllers.Request()).add_done_callback(self._on_list)

    def _on_list(self, future):
        self._poll_pending = False
        try:
            controllers = future.result().controller
        except Exception as exc:  # noqa: BLE001 - any failure means "don't trust the command"
            self.get_logger().warn(f'list_controllers failed: {exc}', throttle_duration_sec=5.0)
            self._state.set_forward_active(False)
            return
        active = any(c.name == self._forward and c.state == 'active' for c in controllers)
        if active != self._state.forward_active:
            self.get_logger().info(
                f'{self._forward} {"active" if active else "inactive"}: Servo now integrates from '
                f'{"its last command" if active else "the real joint state"}')
        self._state.set_forward_active(active)

    # -- data -----------------------------------------------------------------

    def _on_command(self, msg: Float64MultiArray):
        self._state.on_command(msg.data)
        self._publish()

    def _on_joint_states(self, msg: JointState):
        self._last_js = msg
        self._publish()

    def _publish(self):
        js = self._last_js
        if js is None:
            return
        was_tracking = self._state.tracking
        positions = self._state.resolve(dict(zip(js.name, js.position)))
        if was_tracking and not self._state.tracking and self._state.forward_active:
            self.get_logger().warn(
                f'{self._state.last_drop_joint}: command more than '
                f'{self._state.max_lead_rad:.2f} rad from the arm - re-basing Servo on the '
                'real joint state (blocked or overloaded joint?)', throttle_duration_sec=2.0)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = js.header.frame_id
        out.name = list(js.name)
        out.position = [positions[n] for n in js.name]
        out.velocity = list(js.velocity)
        out.effort = list(js.effort)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ServoStateEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
