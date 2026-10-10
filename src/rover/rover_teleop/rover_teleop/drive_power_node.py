#!/usr/bin/env python3
"""
Drive power owner.

Two operators command the same drive: the onboard joystick and the ground
station. Everything they can ask for that is *hardware* state rather than a
choice about their own output lives here, so there is exactly one copy of it.

Before this node existed, all of it lived inside joystick_interpreter, which
meant the swerve controller could only ever be activated by pressing a button
on a physical gamepad plugged into the rover — the ground station could hold
full priority in twist_mux and still not move a wheel.

Services (absolute forms for the ground station's latching switches, /toggle
forms for the joystick's momentary buttons):

  drive/power           std_srvs/SetBool   motors + controller, together
  drive/power/toggle    std_srvs/Trigger
  drive/compact         std_srvs/SetBool
  drive/compact/toggle  std_srvs/Trigger
  drive/clear_errors    std_srvs/Trigger   clear latched drive faults

Every one of them is fire-and-forget: the reply says whether the request was
accepted, and drive/state says what actually happened. Blocking a teleop
service on controller_manager is not worth the tidier return value.

Topic:
  drive/state           indomitus_interfaces/DriveState
    Latched. Republished on a 2 Hz heartbeat so a console can tell live state
    from a dead publisher, and pushed out immediately on a real change so the
    light bar does not lag the button — see _heartbeat and _publish_change.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State

from indomitus_interfaces.msg import DriveState as DriveStateMsg

from rover_teleop.drive_power_state import (
    DrivePower,
    after_compact_result,
    after_controller_result,
    after_errors_cleared,
    after_power_result,
    seeded,
)
from rover_teleop.service_call import GuardedCall
from rover_teleop.teleop_state import GenerationGuard


#: Latched, depth 1: a ground station UI that connects after the fact gets the
#: current state immediately rather than waiting for somebody to touch a switch.
STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class DrivePowerNode(Node):

    def __init__(self):
        super().__init__('drive_power')

        def declare_and_get(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self._controller_name = str(
            declare_and_get('controller_name', 'swerve_controller_test'))
        self._hardware_name = str(declare_and_get('hardware_name', 'RoverHardware'))
        # Capped low on purpose: drive/state crosses the Wi-Fi link to the
        # ground station, and nothing downstream needs it faster than the
        # joystick repaints its light bar.
        self._state_pub_rate = float(declare_and_get('state_pub_rate', 2.0))
        self._state_period = 1.0 / max(0.1, self._state_pub_rate)
        # Floor between out-of-turn publishes. A real change goes out at once
        # rather than waiting for the heartbeat — that wait is what an operator
        # reads as the light bar lagging the button. This only stops a burst of
        # changes from turning into a burst of packets.
        self._state_min_period = float(declare_and_get('state_min_period', 0.05))
        self._last_state_publish = 0.0

        self._state = DrivePower()
        self._joystick_active = False
        self._state_dirty = True

        # Startup state is read from controller_manager rather than assumed —
        # see _seed_from_controller_manager. _operator_touched makes sure a
        # slow seed can never overwrite a command somebody has already given.
        self._seeded = False
        self._operator_touched = False

        # Newest switch_controller request wins. Replies can land out of order,
        # and an old one must not be allowed to write back a stale
        # controller_active — that is how an operator ends up being told the
        # rover is drivable with the controller actually inactive.
        self._switch_guard = GenerationGuard()

        self._state_pub = self.create_publisher(DriveStateMsg, 'drive/state', STATE_QOS)
        self._state_timer = self.create_timer(self._state_period, self._publish_state)

        # joystick_interpreter outranks the ground station in twist_mux, so the
        # ground station operator needs to see when it is holding cmd_vel_joy.
        # Latched on both ends; no joystick running at all leaves this False,
        # which is the correct answer.
        self._joystick_active_sub = self.create_subscription(
            Bool, 'teleop/joystick_active', self._on_joystick_active, STATE_QOS)

        self._power_client = self.create_client(
            SetHardwareComponentState,
            'controller_manager/set_hardware_component_state')
        self._switch_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller')
        self._compact_client = self.create_client(
            SetBool, f'{self._controller_name}/set_compact_mode')
        self._clear_errors_client = self.create_client(
            Trigger,
            str(declare_and_get('clear_errors_service',
                                'rover_hardware_node/clear_motor_errors')))
        self._list_hardware_client = self.create_client(
            ListHardwareComponents, 'controller_manager/list_hardware_components')
        self._list_controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers')

        self._power_guard = GuardedCall(self._power_client)
        self._compact_guard = GuardedCall(self._compact_client)
        self._clear_errors_guard = GuardedCall(self._clear_errors_client)

        self.create_service(SetBool, 'drive/power', self._on_power)
        self.create_service(Trigger, 'drive/power/toggle', self._on_power_toggle)
        self.create_service(SetBool, 'drive/compact', self._on_compact)
        self.create_service(Trigger, 'drive/compact/toggle', self._on_compact_toggle)
        self.create_service(Trigger, 'drive/clear_errors', self._on_clear_errors)

        # controller_manager is usually not up yet when this node starts, so
        # poll for it rather than giving up on the first miss.
        self._seed_timer = self.create_timer(1.0, self._seed_from_controller_manager)

        self._publish_state()

        self.get_logger().info(
            f'DrivePower started — controller={self._controller_name}, '
            f'hardware={self._hardware_name}\n'
            f'  drive/power           (Service) - SetBool, absolute\n'
            f'  drive/power/toggle    (Service) - Trigger, invert\n'
            f'  drive/compact         (Service) - SetBool, absolute\n'
            f'  drive/compact/toggle  (Service) - Trigger, invert\n'
            f'  drive/clear_errors    (Service) - Trigger\n'
            f'  drive/state           (Topic)   - latched, {self._state_pub_rate} Hz + on change'
        )

    # =======================================================================
    # State topic
    # =======================================================================

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _publish_state(self):
        """Publish the current state. Also the heartbeat, on a timer.

        Republishing unchanged state is not redundancy: it is the only thing
        that tells a console the value on screen is still live. Without it a
        dead drive_power_node leaves the last latched state up forever,
        indistinguishable from a rover deliberately sitting still. It also
        makes the topic usable from a VOLATILE subscriber — the ground station
        UI reaches it through rosbridge, which need not match TRANSIENT_LOCAL.
        """
        state = self._state
        self._state_dirty = False
        self._last_state_publish = self._now_seconds()

        msg = DriveStateMsg()
        msg.motors_enabled = state.motors_enabled
        msg.controller_active = state.controller_active
        msg.motors_inhibited = state.motors_inhibited
        msg.compact_mode = state.compact_mode
        msg.can_drive = state.can_drive
        msg.joystick_active = self._joystick_active
        msg.controller_name = self._controller_name
        self._state_pub.publish(msg)

    def _publish_change(self):
        """Push a real change out now instead of waiting for the heartbeat.

        Waiting costs up to a full period, and that delay is visible: the
        operator presses the motor button and the light bar follows half a
        second later. Skipped only when the last publish was very recent, so a
        run of changes still cannot flood the link — the heartbeat carries it.
        """
        if self._now_seconds() - self._last_state_publish < self._state_min_period:
            return
        self._publish_state()

    def _commit(self, state: DrivePower):
        if state == self._state:
            return
        self._state = state
        self._state_dirty = True
        self._publish_change()

    def _on_joystick_active(self, msg: Bool):
        if msg.data == self._joystick_active:
            return
        self._joystick_active = msg.data
        self._state_dirty = True
        self._publish_change()

    # =======================================================================
    # Startup: adopt whatever is already true
    # =======================================================================

    def _seed_from_controller_manager(self):
        """Read the drive state back instead of assuming it is off.

        On hardware bringup spawns the swerve controller inactive, so the
        assumption happens to hold. In simulation both the hardware component
        and the controller come up active, and assuming otherwise would paint
        the light bar red on a rover that drives perfectly well and make the
        operator's first button press ask for a state that already applies.
        """
        if self._seeded or self._operator_touched:
            self._seed_timer.cancel()
            return
        if not (self._list_hardware_client.service_is_ready()
                and self._list_controllers_client.service_is_ready()):
            return

        self._seeded = True
        self._seed_timer.cancel()
        self._list_hardware_client.call_async(
            ListHardwareComponents.Request()).add_done_callback(self._on_hardware_listed)

    def _on_hardware_listed(self, future):
        try:
            components = future.result().component
        except Exception as exc:
            self.get_logger().warn(
                f'could not read hardware state at startup: {exc!r} — '
                f'assuming the drive is off')
            return

        motors_enabled = any(
            component.name == self._hardware_name
            and component.state.id == State.PRIMARY_STATE_ACTIVE
            for component in components)

        # Chained rather than run in parallel: two replies to merge is two more
        # orderings to get wrong, and this runs once at startup.
        self._list_controllers_client.call_async(
            ListControllers.Request()).add_done_callback(
                lambda f: self._on_controllers_listed(f, motors_enabled))

    def _on_controllers_listed(self, future, motors_enabled: bool):
        try:
            controllers = future.result().controller
        except Exception as exc:
            self.get_logger().warn(
                f'could not read controller state at startup: {exc!r} — '
                f'assuming the drive is off')
            return

        controller_active = any(
            controller.name == self._controller_name and controller.state == 'active'
            for controller in controllers)

        if self._operator_touched:
            # Somebody gave a command while these replies were in flight.
            # Their intent is newer than this snapshot.
            self.get_logger().debug('startup state discarded — a command arrived first')
            return

        self._commit(seeded(self._state, motors_enabled, controller_active))
        self.get_logger().info(
            f'Startup state read from controller_manager: '
            f'{self._hardware_name} {"active" if motors_enabled else "inactive"}, '
            f'{self._controller_name} {"active" if controller_active else "inactive"}')

    # =======================================================================
    # Power (motors + controller, as one operation)
    # =======================================================================

    def _on_power(self, request, response):
        return self._request_power(bool(request.data), response)

    def _on_power_toggle(self, request, response):
        return self._request_power(not self._state.motors_enabled, response)

    def _request_power(self, desired: bool, response):
        self._operator_touched = True
        target = 'ON' if desired else 'OFF'

        req = SetHardwareComponentState.Request()
        req.name = self._hardware_name
        req.target_state.id = (
            State.PRIMARY_STATE_ACTIVE if desired else State.PRIMARY_STATE_INACTIVE
        )

        started = self._power_guard.call(
            req, lambda future: self._on_power_result(future, desired))

        response.success = started
        response.message = (
            f'drive power {target} requested'
            if started else
            'set_hardware_component_state busy or not available'
        )
        if not started:
            self.get_logger().warn(response.message)
        return response

    def _on_power_result(self, future, desired: bool):
        try:
            ok = future.result().ok
        except Exception as exc:
            self.get_logger().error(f'drive power call failed: {exc!r}')
            return

        self._commit(after_power_result(self._state, ok, desired))

        if not ok:
            self.get_logger().error(
                f'controller_manager refused to make {self._hardware_name} '
                f'{"active" if desired else "inactive"}')
            return

        self.get_logger().info(f'Motors {"ENABLED" if desired else "DISABLED"}')
        # The controller follows power: an active controller with the hardware
        # inactive spins a loop that can never reach a wheel, and an active
        # hardware with no controller is a rover that ignores /cmd_vel.
        self._switch_controller(desired)

    def _switch_controller(self, activate: bool):
        if not self._switch_client.service_is_ready():
            self.get_logger().warn('switch_controller service not available')
            return

        req = SwitchController.Request()
        if activate:
            req.activate_controllers = [self._controller_name]
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = [self._controller_name]
        req.strictness = SwitchController.Request.BEST_EFFORT

        generation = self._switch_guard.start()
        self._switch_client.call_async(req).add_done_callback(
            lambda future: self._on_switch_result(future, activate, generation))

    def _on_switch_result(self, future, activate: bool, generation: int):
        target = 'active' if activate else 'inactive'

        if not self._switch_guard.is_current(generation):
            # A newer request has already been sent; this reply describes a
            # superseded intent. Acting on it would write back a stale
            # controller_active and mislead every operator watching drive/state.
            self.get_logger().debug(
                f'ignoring stale switch_controller reply (-> {target})')
            return

        try:
            ok = future.result().ok
        except Exception as exc:
            self.get_logger().error(
                f'switch_controller call failed: {exc!r} — '
                f'{self._controller_name} is not {target}')
            return

        self._commit(after_controller_result(self._state, ok, activate))

        if ok:
            self.get_logger().info(f'{self._controller_name} -> {target}')
        else:
            self.get_logger().error(
                f'controller_manager refused to make {self._controller_name} {target}')

    # =======================================================================
    # Compact mode
    # =======================================================================

    def _on_compact(self, request, response):
        return self._request_compact(bool(request.data), response)

    def _on_compact_toggle(self, request, response):
        return self._request_compact(not self._state.compact_mode, response)

    def _request_compact(self, desired: bool, response):
        self._operator_touched = True
        req = SetBool.Request()
        req.data = desired

        started = self._compact_guard.call(
            req, lambda future: self._on_compact_result(future, desired))

        response.success = started
        response.message = (
            f'compact mode {"ON" if desired else "OFF"} requested'
            if started else
            f'/{self._controller_name}/set_compact_mode busy or not available'
        )
        if not started:
            self.get_logger().warn(response.message)
        return response

    def _on_compact_result(self, future, desired: bool):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Compact mode call failed: {exc!r}')
            return

        self._commit(after_compact_result(self._state, result.success, desired))
        self.get_logger().info(
            f'Compact mode {"ENABLED" if self._state.compact_mode else "DISABLED"}: '
            f'{result.message}')

    # =======================================================================
    # Clear motor errors
    # =======================================================================

    def _on_clear_errors(self, request, response):
        self._operator_touched = True
        started = self._clear_errors_guard.call(
            Trigger.Request(), self._on_clear_errors_result)

        response.success = started
        response.message = (
            'clear_motor_errors requested'
            if started else
            'clear_motor_errors busy or not available'
        )
        if not started:
            self.get_logger().warn(response.message)
        return response

    def _on_clear_errors_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'clear_motor_errors call failed: {exc!r}')
            return

        if not result.success:
            self.get_logger().error(f'Motor errors not cleared: {result.message}')
            return

        self._commit(after_errors_cleared(self._state))
        self.get_logger().info(
            f'Motor errors cleared: {result.message} — '
            f'cycle drive power to re-enable')


def main(args=None):
    rclpy.init(args=args)
    node = DrivePowerNode()
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
