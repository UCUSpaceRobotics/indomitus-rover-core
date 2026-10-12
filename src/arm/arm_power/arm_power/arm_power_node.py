#!/usr/bin/env python3
"""
Arm power owner — the arm's counterpart of rover_power's drive_power.

The real arm starts with its motors off (ArmSystem INACTIVE: positions read,
no torque). Turning them on and, more importantly, off again is hardware state
every operator shares — keyboard and gamepad teleop, the ground station — so it
lives here, once, instead of inside each teleop process.

Service:
  power    std_srvs/SetBool
    true:  ArmSystem -> active (ArmCanSystem ramps the stiffness up from the
           measured pose), then the arm trajectory controller and any gripper
           controllers -> active.
    false: Release. Refused in the reply unless the arm rests in the park pose
           (arm_teleop's Park lays it there) and is still. Then the arm's
           controllers -> inactive and ArmSystem -> inactive: ArmCanSystem
           ramps the stiffness down, shoulder and elbow first, and disables the
           motors — or, if a joint moves meanwhile, restores full stiffness and
           stays active (reported in last_error).
  The reply says whether the request was accepted; state says what happened.

Topic:
  state    indomitus_interfaces/ArmState
    Latched; republished on a heartbeat and immediately on a change. Kept true
    by polling controller_manager, since teleop switches the arm's controllers
    itself (trajectory <-> streaming).
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from indomitus_interfaces.msg import ArmState as ArmStateMsg

from arm_power.arm_power_state import (
    ArmPower,
    after_power_on,
    after_release,
    can_move,
    in_park_pose,
    observed,
    release_refusal,
    request_started,
    stiffness_ready,
)
from arm_power.service_call import GuardedCall


#: Latched, depth 1: a console that connects later gets the current state.
STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ARM_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]


class ArmPowerNode(Node):

    def __init__(self):
        super().__init__('arm_power')

        def declare_and_get(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self._hardware_name = str(declare_and_get('hardware_name', 'ArmSystem'))
        self._arm_controllers = list(declare_and_get(
            'arm_controllers',
            ['indomitus_arm_controller', 'indomitus_arm_forward_position_controller']))
        # Activated with the motors: the first arm controller (trajectory) and
        # every gripper controller that is loaded but inactive.
        self._gripper_controllers = list(declare_and_get(
            'gripper_controllers', ['gripper_right_controller', 'gripper_left_controller']))
        # ArmCanSystem's gain_ramp_secs on real hardware; 0 for mock / Gazebo.
        self._stiffness_ramp_secs = float(declare_and_get('stiffness_ramp_secs', 15.0))
        park = list(declare_and_get('park_pose', [0.0]))
        self._park = dict(zip(ARM_JOINTS, park)) if len(park) == len(ARM_JOINTS) else {}
        self._park_tolerance = float(declare_and_get('park_pose_tolerance', 0.08))
        # 12-bit velocity feedback quantizes to 0.022 rad/s.
        self._velocity_tolerance = float(declare_and_get('release_velocity_tolerance', 0.15))
        state_rate = float(declare_and_get('state_pub_rate', 2.0))

        self._state = ArmPower()
        self._positions = {}
        self._velocities = {}
        self._last_msg = None
        self._loaded = {}   # controller name -> state, from the last poll

        self._state_pub = self.create_publisher(ArmStateMsg, 'state', STATE_QOS)
        self.create_subscription(JointState, 'joint_states', self._on_joint_state, 10)

        self._set_hardware_client = self.create_client(
            SetHardwareComponentState, 'controller_manager/set_hardware_component_state')
        self._switch_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller')
        self._list_hardware_client = self.create_client(
            ListHardwareComponents, 'controller_manager/list_hardware_components')
        self._list_controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers')
        self._set_hardware_guard = GuardedCall(self._set_hardware_client)
        self._poll_guard = GuardedCall(self._list_hardware_client)

        self.create_service(SetBool, 'power', self._on_power)

        self.create_timer(1.0 / max(0.1, state_rate), self._heartbeat)

        if not self._park:
            self.get_logger().warn(
                'No park_pose (poses.json "park") — releasing the motors will be refused.')
        self.get_logger().info(
            f'ArmPower started — hardware={self._hardware_name}, '
            f'stiffness ramp {self._stiffness_ramp_secs:.0f} s\n'
            f'  power   (Service) - SetBool: true = motors on, false = release (park pose only)\n'
            f'  state   (Topic)   - latched ArmState')

    # =======================================================================
    # State topic
    # =======================================================================

    def _publish(self, force: bool = False):
        now = time.monotonic()
        state = self._state
        msg = ArmStateMsg()
        msg.motors_enabled = state.motors_enabled
        msg.controller_active = state.controller_active
        msg.stiffness_ready = stiffness_ready(state, now, self._stiffness_ramp_secs)
        msg.in_park_pose = in_park_pose(self._positions, self._park, self._park_tolerance)
        msg.busy = state.busy
        msg.can_move = can_move(state, now, self._stiffness_ramp_secs)
        msg.last_error = state.last_error
        if force or msg != self._last_msg:
            self._last_msg = msg
            self._state_pub.publish(msg)

    def _commit(self, state: ArmPower):
        self._state = state
        self._publish()

    def _heartbeat(self):
        """Republish (a console can tell a live state from a dead node) and
        re-read controller_manager: teleop switches the arm's controllers."""
        self._poll()
        self._publish(force=True)

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = float(pos)
        for name, vel in zip(msg.name, msg.velocity):
            self._velocities[name] = float(vel)
        self._publish()   # in_park_pose may have changed; unchanged -> not sent

    # =======================================================================
    # controller_manager poll
    # =======================================================================

    def _poll(self):
        if self._state.busy or not self._list_controllers_client.service_is_ready():
            return
        self._poll_guard.call(ListHardwareComponents.Request(), self._on_hardware_listed)

    def _on_hardware_listed(self, future):
        try:
            components = future.result().component
        except Exception as exc:
            self.get_logger().warn(f'list_hardware_components failed: {exc!r}')
            return
        motors_enabled = any(
            c.name == self._hardware_name and c.state.id == State.PRIMARY_STATE_ACTIVE
            for c in components)
        self._list_controllers_client.call_async(ListControllers.Request()).add_done_callback(
            lambda f: self._on_controllers_listed(f, motors_enabled))

    def _on_controllers_listed(self, future, motors_enabled: bool):
        try:
            controllers = future.result().controller
        except Exception as exc:
            self.get_logger().warn(f'list_controllers failed: {exc!r}')
            return
        if self._state.busy:
            return   # a request started meanwhile; its outcome is newer than this
        self._loaded = {c.name: c.state for c in controllers}
        controller_active = any(self._loaded.get(n) == 'active' for n in self._arm_controllers)
        self._commit(observed(self._state, motors_enabled, controller_active, time.monotonic()))

    # =======================================================================
    # Power
    # =======================================================================

    def _on_power(self, request, response):
        on = bool(request.data)
        if self._state.busy:
            response.success = False
            response.message = 'a power request is already in progress'
            return response
        if on:
            return self._power_on(response)
        return self._release(response)

    def _set_hardware(self, active: bool, on_done) -> bool:
        request = SetHardwareComponentState.Request()
        request.name = self._hardware_name
        request.target_state.id = (
            State.PRIMARY_STATE_ACTIVE if active else State.PRIMARY_STATE_INACTIVE)
        return self._set_hardware_guard.call(request, on_done)

    def _switch(self, activate, deactivate, on_done=None):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        # Already active / inactive and not-loaded controllers are skipped.
        request.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch_client.call_async(request)
        if on_done is not None:
            future.add_done_callback(on_done)

    def _power_on(self, response):
        if self._state.motors_enabled:
            # Already on: just make sure the joints follow a controller.
            self._activate_controllers()
            response.success = True
            response.message = 'motors already on'
            return response
        if not self._set_hardware(True, self._on_powered_on):
            response.success = False
            response.message = 'set_hardware_component_state busy or not available'
            return response
        self._commit(request_started(self._state))
        response.success = True
        response.message = 'turning the arm motors on'
        self.get_logger().info('Turning the arm motors on...')
        return response

    def _on_powered_on(self, future):
        try:
            ok = future.result().state.id == State.PRIMARY_STATE_ACTIVE
        except Exception as exc:
            self.get_logger().error(f'set_hardware_component_state failed: {exc!r}')
            ok = False
        self._commit(after_power_on(self._state, ok, time.monotonic(),
                                    error='activation failed — see ros2_control_node\'s log'))
        if not ok:
            self.get_logger().error(f'{self._hardware_name} did not become active.')
            return
        self.get_logger().info(
            f'Arm motors on; stiffness ramping up for {self._stiffness_ramp_secs:.0f} s.')
        self._activate_controllers()

    def _activate_controllers(self):
        grippers = [n for n in self._gripper_controllers if self._loaded.get(n) == 'inactive']
        self._switch([self._arm_controllers[0], *grippers], [],
                     on_done=lambda _: self._poll())

    def _release(self, response):
        refusal = release_refusal(self._state, self._positions, self._velocities,
                                  self._park, self._park_tolerance, self._velocity_tolerance)
        if refusal:
            response.success = False
            response.message = f'release refused: {refusal}'
            self.get_logger().warn(response.message)
            return response
        if not self._state.motors_enabled:
            response.success = True
            response.message = 'motors already off'
            return response
        if not (self._switch_client.service_is_ready()
                and self._set_hardware_client.service_is_ready()):
            response.success = False
            response.message = 'controller_manager not available'
            return response
        self._commit(request_started(self._state))
        self.get_logger().info('Releasing: arm controllers off, then the motors ramp down...')
        self._switch([], [*self._arm_controllers, *self._gripper_controllers],
                     on_done=self._on_controllers_off)
        response.success = True
        response.message = 'releasing the arm (motors ramp down, then off)'
        return response

    def _on_controllers_off(self, _future):
        if not self._set_hardware(False, self._on_released):
            self._commit(after_release(self._state, False, 'set_hardware_component_state busy'))
            self._activate_controllers()

    def _on_released(self, future):
        try:
            ok = future.result().state.id == State.PRIMARY_STATE_INACTIVE
        except Exception as exc:
            self.get_logger().error(f'set_hardware_component_state failed: {exc!r}')
            ok = False
        if ok:
            self._commit(after_release(self._state, True))
            self.get_logger().info('Released: arm motors are off.')
            return
        error = ('release aborted: the arm moved while relaxing (not resting on its '
                 'support?) — motors back at full stiffness')
        self._commit(after_release(self._state, False, error))
        self.get_logger().error(error)
        self._activate_controllers()


def main(args=None):
    rclpy.init(args=args)
    node = ArmPowerNode()
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
