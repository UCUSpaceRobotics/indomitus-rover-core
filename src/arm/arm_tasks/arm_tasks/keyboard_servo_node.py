#!/usr/bin/env python3
"""
Keyboard control node for MoveIt Servo.

Reads keyboard input and sends Cartesian velocity commands to MoveIt Servo.
Designed to be modular — core logic is in ServoController class,
so switching to gamepad (joy package) requires only replacing the input source.

Controls:
    Translation (relative to camera_link):
        w / s  — forward / backward   (X axis)
        a / d  — left / right         (Y axis)
        q / e  — up / down            (Z axis)

    Rotation (relative to camera_link):
        i / k  — roll CW / CCW
        u / o  — pitch up / down
        j / l  — yaw left / right

    t / y  — close / open gripper
    r      — move to safe pose + start servo
    ESC/x  — exit

Gamepad controls (via ros2 joy joy_node, e.g. Stadia controller):
    Right stick      — forward/back, left/right  (X / Y axis)
    Left stick       — yaw / pitch
    L2 / R2          — roll
    L2 / R2, Y held  — up / down
    L1 / R1          — close / open gripper
    A                — move to safe pose + start servo
    X                — exit

Usage:
    Real hardware / RViz mock-hardware demo (wall clock, conservative speeds):
        ros2 run arm_tasks keyboard_servo_node

    Gazebo sim (sim clock + faster speeds, see arm_sim/config/keyboard_servo_sim.yaml):
        ros2 run arm_tasks keyboard_servo_node --ros-args \\
            --params-file $(ros2 pkg prefix arm_sim)/share/arm_sim/config/keyboard_servo_sim.yaml

    Gamepad, any target (start the joystick driver first, then this node):
        ros2 run joy joy_node
        ros2 run arm_tasks gamepad_servo_node
"""

import sys
import threading
import termios
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float64MultiArray
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import evdev
from evdev import ecodes


DEFAULT_LINEAR_SPEED  = 0.1
DEFAULT_ANGULAR_SPEED = 0.3
# GRIPPER_STROKE is only 0.012m, so using linear_speed here (meant for a
# much bigger Cartesian workspace) would open/close it in ~0.1s — a jump,
# not a jog. This gives a ~2s full stroke instead.
DEFAULT_GRIPPER_SPEED = 0.006
DEFAULT_PUBLISH_RATE  = 50.0
DEFAULT_COMMAND_FRAME = 'arm_camera_link'
DEFAULT_SAFE_POSE     = [0.0, 1.2, -1.0, 0.8, 0.5, 0.0]
DEFAULT_KEYBOARD_DEVICE_PATH = '/dev/input/event3'
DEFAULT_SAFE_POSE_TIMEOUT = 60.0

SAFE_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]

ROLL_JOINT_NAME = 'arm_wrist_2_end_effector_joint'

# Unlike ROLL_JOINT_NAME, this isn't in the indomitus_arm planning group,
# so Servo silently ignores JointJog commands for it ("Ignoring joint
# arm_jaw_gripper_finger_right_joint") — it needs its own ros2_control
# controller (gripper_controller) commanded directly, bypassing Servo
# entirely. 0 is closed, GRIPPER_STROKE is open (matches the joint's
# URDF limit), so held keys integrate into an absolute position setpoint
# rather than a velocity, since the controller only takes positions.
GRIPPER_JOINT_NAME = 'arm_jaw_gripper_finger_right_joint'
GRIPPER_STROKE = 0.012

SERVO_STATUS_INVALID                              = -1
SERVO_STATUS_OK                                    = 0
SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY = 1
SERVO_STATUS_HALT_FOR_SINGULARITY                  = 2
SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY    = 3
SERVO_STATUS_DECELERATE_FOR_COLLISION              = 4
SERVO_STATUS_HALT_FOR_COLLISION                    = 5
SERVO_STATUS_JOINT_BOUND                           = 6

SERVO_STATUS_NAMES = {
    SERVO_STATUS_INVALID: 'INVALID',
    SERVO_STATUS_OK: 'NO_WARNING',
    SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
    SERVO_STATUS_HALT_FOR_SINGULARITY: 'HALT_FOR_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY: 'DECELERATE_FOR_LEAVING_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_COLLISION: 'DECELERATE_FOR_COLLISION',
    SERVO_STATUS_HALT_FOR_COLLISION: 'HALT_FOR_COLLISION',
    SERVO_STATUS_JOINT_BOUND: 'JOINT_BOUND',
}


class ServoController(Node):
    """ROS2 node that turns Cartesian velocity commands into MoveIt Servo messages.

    Declares and reads ROS parameters for speed, publish rate, command frame,
    safe pose and keyboard device path, publishes ``TwistStamped`` messages on
    a timer, and exposes helper methods to start/stop Servo and to drive the
    arm to a predefined safe pose via a ``FollowJointTrajectory`` action.
    """

    def __init__(self):
        """Initialize the node, declare parameters, and set up pub/sub/clients.

        Declares all ROS parameters (with defaults), reads their resolved
        values into instance attributes, zeroes the internal velocity state,
        creates the twist publisher, the Servo start/stop service clients,
        the trajectory action client, the publish timer, and the servo
        status subscription.
        """

        super().__init__('keyboard_servo_node')

        self.declare_parameter('linear_speed',  DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('gripper_speed', DEFAULT_GRIPPER_SPEED)
        self.declare_parameter('publish_rate',  DEFAULT_PUBLISH_RATE)
        self.declare_parameter('command_frame', DEFAULT_COMMAND_FRAME)
        self.declare_parameter('safe_pose',     DEFAULT_SAFE_POSE)
        self.declare_parameter('keyboard_device_path', DEFAULT_KEYBOARD_DEVICE_PATH)
        self.declare_parameter('safe_pose_timeout', DEFAULT_SAFE_POSE_TIMEOUT)

        self._linear_speed  = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._gripper_speed = self.get_parameter('gripper_speed').value
        self._publish_rate  = self.get_parameter('publish_rate').value
        self._command_frame = self.get_parameter('command_frame').value
        self._safe_pose     = list(self.get_parameter('safe_pose').value)
        self._keyboard_device_path = self.get_parameter('keyboard_device_path').value
        self._safe_pose_timeout    = self.get_parameter('safe_pose_timeout').value

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.gripper_vel = 0.0
        self._gripper_position = 0.0
        self._roll_was_active = False

        self._pub = self.create_publisher(TwistStamped, 'servo_node/delta_twist_cmds', 10)
        self._joint_jog_pub = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 10)
        self._gripper_pub = self.create_publisher(Float64MultiArray, 'gripper_controller/commands', 10)
        self._start_client = self.create_client(Trigger, 'servo_node/start_servo')
        self._stop_client  = self.create_client(Trigger, 'servo_node/stop_servo')
        self._traj_client  = ActionClient(
            self,
            FollowJointTrajectory,
            'indomitus_arm_controller/follow_joint_trajectory'
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

        self._servo_status = SERVO_STATUS_OK
        self._status_sub = self.create_subscription(
            Int8,
            'servo_node/status',
            self._on_servo_status,
            10
        )

        self.get_logger().info(
            f'ServoController ready — '
            f'linear_speed={self._linear_speed}, '
            f'angular_speed={self._angular_speed}, '
            f'gripper_speed={self._gripper_speed}, '
            f'command_frame={self._command_frame}'
        )

    @property
    def linear_speed(self) -> float:
        """Return the configured linear speed scale, in meters per second."""
        return self._linear_speed

    @property
    def angular_speed(self) -> float:
        """Return the configured angular speed scale, in radians per second."""
        return self._angular_speed

    @property
    def gripper_speed(self) -> float:
        """Return the configured gripper speed scale, in meters per second."""
        return self._gripper_speed

    @property
    def keyboard_device_path(self) -> str:
        """Return the filesystem path of the keyboard input device (evdev)."""
        return self._keyboard_device_path

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0, gripper_vel=0.0):
        """Set the current Cartesian velocity command.

        Args:
            vx: Linear velocity along the X axis, in meters per second.
            vy: Linear velocity along the Y axis, in meters per second.
            vz: Linear velocity along the Z axis, in meters per second.
            wx: Angular velocity about the X axis, in radians per second.
            wy: Angular velocity about the Y axis, in radians per second.
            wz: Angular velocity about the Z axis, in radians per second.
            gripper_vel: Velocity for GRIPPER_JOINT_NAME, in meters per
                second (positive opens, negative closes).

        Notes:
            The stored values are published on the next timer tick by
            ``_publish``; this method does not publish immediately.
        """
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.gripper_vel = gripper_vel

    def stop(self):
        """Zero out all velocity components, halting Cartesian motion.

        Equivalent to calling ``set_velocity()`` with no arguments.
        """
        self.set_velocity()

    def stop_servo(self) -> bool:
        """Call the Servo ``stop_servo`` service and wait for confirmation.

        Waits up to 2 seconds for the service to become available and up to
        3 seconds for the asynchronous call to complete.

        Returns:
            bool: True if the service was available and the call completed
            within the timeout; False if the service was unavailable or the
            call timed out.
        """
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo stop service not available')
            return False

        done_event = threading.Event()

        def _cb(future):
            """Mark the stop-servo call as complete (service response callback)."""
            done_event.set()

        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=3.0):
            self.get_logger().warn('Servo stop timed out')
            return False
        return True

    def move_to_safe_pose(self):
        """Stop motion and drive the arm to the configured safe pose.

        Halts current velocity commands, confirms Servo has stopped, then
        sends a ``FollowJointTrajectory`` goal to move all joints to the
        ``safe_pose`` parameter values over 3 seconds (of controller time,
        i.e. sim time under Gazebo) and blocks until the controller reports
        the goal finished, up to ``safe_pose_timeout`` wall-clock seconds
        (<= 0 waits forever; the default is generous because under a slow
        sim the trajectory legitimately takes longer in wall time). This
        method runs on a dedicated thread, so waiting does not stall
        keyboard handling.

        Returns:
            bool: True if the controller reported the goal SUCCEEDED with
            error code SUCCESSFUL; False if Servo could not be confirmed
            stopped, the action server was unavailable, the goal was
            rejected, the trajectory was aborted/canceled or finished with
            a controller error, or no result arrived within the timeout.
        """
        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting safe pose move.'
            )
            return False

        if not self._traj_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        traj = JointTrajectory()
        traj.joint_names = SAFE_POSE_JOINTS

        point = JointTrajectoryPoint()
        point.positions = self._safe_pose
        point.velocities = [0.0] * len(self._safe_pose)
        point.time_from_start = Duration(sec=3)
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info('Moving to safe pose...')

        done_event = threading.Event()
        outcome = {'success': False, 'error': ''}

        def result_cb(future):
            """Record the trajectory result's status and error code."""
            try:
                wrapped = future.result()
                result = wrapped.result
                if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                    outcome['success'] = True
                else:
                    outcome['error'] = (
                        f'goal status {wrapped.status}, '
                        f'controller error code {result.error_code}'
                        + (f' ({result.error_string})' if result.error_string else '')
                    )
            except Exception as e:
                outcome['error'] = f'failed to read result: {e!r}'
            finally:
                done_event.set()

        def goal_response_cb(future):
            """Handle the trajectory action's goal-acceptance response.

            Args:
                future: Future resolving to the goal handle returned by
                    ``send_goal_async``.
            """
            try:
                goal_handle = future.result()
            except Exception as e:
                goal_handle = None
                outcome['error'] = f'goal request failed: {e!r}'
            if not goal_handle or not goal_handle.accepted:
                outcome['error'] = outcome['error'] or 'goal rejected'
                done_event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_cb)

        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        timeout = self._safe_pose_timeout if self._safe_pose_timeout > 0.0 else None
        if not done_event.wait(timeout=timeout):
            self.get_logger().warn(
                f'No trajectory result within {self._safe_pose_timeout:.1f}s — '
                'controller may be unresponsive '
                '(raise the safe_pose_timeout parameter if the sim is just slow).'
            )
            return False
        if not outcome['success']:
            self.get_logger().error(f'Safe pose move failed: {outcome["error"]}')
            return False
        self.get_logger().info('Safe pose reached!')
        return True

    def start_servo(self):
        """Asynchronously call the Servo ``start_servo`` service.

        Waits up to 2 seconds for the service to become available, then
        issues an asynchronous call whose result is handled by
        ``_on_start_result``. Does not block for the call's completion.
        """
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servo start service not available')
            return
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_result)

    def _on_start_result(self, future):
        """Log the outcome of an asynchronous ``start_servo`` service call.

        Args:
            future: Future resolving to the ``Trigger.Response`` returned by
                the ``start_servo`` service.
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Servo started successfully')
            else:
                self.get_logger().warn(f'Servo start failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Servo start error: {e}')

    def _on_servo_status(self, msg: Int8):
        """Handle incoming Servo status updates.

        Automatically restarts Servo only when the status transitions into
        SERVO_STATUS_HALT_FOR_SINGULARITY, since that halt is typically
        recoverable by re-issuing start_servo (Servo re-attempts the motion
        away from the singular configuration). Other halt states — e.g.
        SERVO_STATUS_HALT_FOR_COLLISION — are intentionally NOT
        auto-recovered: they represent conditions where continuing motion
        could be unsafe, so they are left for the operator to resolve
        manually (e.g. by moving the arm clear via keyboard input, or a
        workspace/collision object review) rather than being silently
        retried.
        """
        code = msg.data
        if code != self._servo_status:
            if code == SERVO_STATUS_HALT_FOR_SINGULARITY:
                self.start_servo()
        self._servo_status = code

    def _publish(self):
        """Publish the current velocity state, routing roll/gripper independently.

        Called periodically by the internal timer at ``publish_rate`` Hz.

        The gripper has its own ros2_control controller and is commanded
        on every tick ``gripper_vel`` is nonzero, integrating it into an
        absolute position setpoint (the controller only accepts
        positions, and the joint isn't in Servo's planning group so it
        can't go through JointJog) — independent of the twist/roll
        branch below, so it can be combined with either.

        MoveIt Servo acts on whichever command type (Cartesian twist or
        joint jog) arrived most recently, so those two can't be combined
        within one cycle: while ``wx`` (roll) is nonzero, only a
        ``JointJog`` for ``ROLL_JOINT_NAME`` is published and the other
        five Cartesian axes are held for that tick; otherwise a
        ``TwistStamped`` carries ``vx``..``vz``/``wy``/``wz`` as before
        (``wx`` is always 0 there, since roll never travels this path).
        """
        if self.gripper_vel != 0.0:
            self._gripper_position += self.gripper_vel / self._publish_rate
            self._gripper_position = max(0.0, min(GRIPPER_STROKE, self._gripper_position))
            gripper_msg = Float64MultiArray()
            gripper_msg.data = [self._gripper_position]
            self._gripper_pub.publish(gripper_msg)

        if self.wx != 0.0:
            self._roll_was_active = True
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = [ROLL_JOINT_NAME]
            msg.velocities = [self.wx]
            msg.duration = 1.0 / self._publish_rate
            self._joint_jog_pub.publish(msg)
            return

        if self._roll_was_active:
            # Roll just stopped: Servo acts on whichever command type
            # arrived last, so a zero Twist alone might not halt a joint
            # that was being moved via JointJog — send one explicit zero
            # jog so it doesn't coast until Servo's own command timeout.
            self._roll_was_active = False
            halt = JointJog()
            halt.header.stamp = self.get_clock().now().to_msg()
            halt.joint_names = [ROLL_JOINT_NAME]
            halt.velocities = [0.0]
            halt.duration = 1.0 / self._publish_rate
            self._joint_jog_pub.publish(halt)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._command_frame
        msg.twist.linear.x  = self.vx
        msg.twist.linear.y  = self.vy
        msg.twist.linear.z  = self.vz
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = self.wy
        msg.twist.angular.z = self.wz
        self._pub.publish(msg)


HELP = """
╔══════════════════════════════════════════════╗
║     Keyboard Servo Control (camera frame)    ║
╠══════════════════════════════════════════════╣
║  Translation:                                ║
║    w / s  — forward / backward  (X)          ║
║    a / d  — left / right        (Y)          ║
║    q / e  — up / down           (Z)          ║
║  Rotation:                                   ║
║    i / k  — roll CW / CCW                    ║
║    u / o  — pitch up / down                  ║
║    j / l  — yaw left / right                 ║
║  Other:                                      ║
║    t / y  — close / open gripper             ║
║    r      — move to safe pose + start servo  ║
║    ESC/x  — exit                             ║
╚══════════════════════════════════════════════╝
"""


class KeyboardInputLoop:
    """Reads raw keyboard events via evdev and drives a ``ServoController``.

    Maintains the set of currently pressed direction keys, recomputes the
    combined Cartesian velocity whenever the set changes, and handles the
    special "safe pose" and "exit" key bindings.
    """

    _DIRECTIONS = {
        ecodes.KEY_W: ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_S: (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_A: ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_D: ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_Q: ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0),
        ecodes.KEY_E: ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
        ecodes.KEY_I: ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0),
        ecodes.KEY_K: ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0),
        ecodes.KEY_U: ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0),
        ecodes.KEY_O: ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0),
        ecodes.KEY_J: ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0),
        ecodes.KEY_L: ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0),
    }

    # Gripper isn't a Cartesian axis, so it's tracked separately from
    # _DIRECTIONS' 6-tuples rather than forced into that shape.
    _GRIPPER_KEYS = {
        ecodes.KEY_T: -1.0,  # close
        ecodes.KEY_Y:  1.0,  # open
    }

    _KEYSTATE_UP = 0
    _KEYSTATE_DOWN = 1
    _KEYSTATE_REPEAT = 2

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and initialize input state.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from keyboard input.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._gripper_speed = controller.gripper_speed
        self._device_path = controller.keyboard_device_path
        self._lock = threading.Lock()
        self._pressed = set()
        self._exit_event = threading.Event()
        self._device = None
        self._read_thread = None
        self._servo_started = False

    def _open_device(self) -> bool:
        """Open the evdev keyboard device at ``self._device_path``.

        Logs a descriptive error (including a snippet to list available
        input devices) if the device cannot be opened.

        Returns:
            bool: True if the device was opened successfully, False
            otherwise.
        """
        try:
            self._device = evdev.InputDevice(self._device_path)
        except (FileNotFoundError, PermissionError, OSError) as e:
            self._controller.get_logger().error(
                f'Could not open keyboard device {self._device_path!r}: {e!r}. '
                f'Run `python3 -c "import evdev; [print(p, evdev.InputDevice(p).name) '
                f'for p in evdev.list_devices()]"` to list available devices, and set '
                f'the keyboard_device_path ROS parameter accordingly.'
            )
            return False

        self._controller.get_logger().info(
            f'Reading keyboard from {self._device_path} ({self._device.name})'
        )
        return True

    def _recompute_velocity(self):
        """Recompute and apply the combined velocity from all pressed keys.

        Sums the per-axis direction vectors of every currently pressed key
        in ``self._pressed``, scales the result by the controller's linear
        and angular speed settings, and forwards it to
        ``ServoController.set_velocity``.
        """
        vx = vy = vz = wx = wy = wz = gripper = 0.0
        with self._lock:
            active = list(self._pressed)
        for code in active:
            d = self._DIRECTIONS.get(code)
            if d is not None:
                vx += d[0]
                vy += d[1]
                vz += d[2]
                wx += d[3]
                wy += d[4]
                wz += d[5]
                continue
            g = self._GRIPPER_KEYS.get(code)
            if g is not None:
                gripper += g
        self._controller.set_velocity(
            vx * self._linear_speed, vy * self._linear_speed, vz * self._linear_speed,
            wx * self._angular_speed, wy * self._angular_speed, wz * self._angular_speed,
            gripper_vel=gripper * self._gripper_speed,
        )

    def _handle_safe_pose(self):
        """Clear pressed keys, stop motion, and move to the safe pose.

        Servo is only started if the safe-pose move actually succeeded;
        starting it after a rejected/aborted/timed-out move would let the
        operator resume Cartesian teleop from a pose that never reached the
        intended safe configuration.

        Intended to run in its own thread (spawned from ``_read_loop``) so
        that the blocking safe-pose and servo-start calls do not stall
        keyboard event processing.
        """
        with self._lock:
            self._pressed.clear()
        self._controller.stop()
        print('Moving to safe pose...')
        if self._controller.move_to_safe_pose():
            print('Starting servo...')
            self._controller.start_servo()
            self._servo_started = True
        else:
            print('Safe pose failed — Servo not started.')

    def _read_loop(self):
        """Continuously read raw key events and update velocity/state.

        Runs until ESC/X is pressed, the exit event is set, or the device
        read loop raises an ``OSError``. Recognized key events:

        * ESC / X (key down) — signal exit and stop reading.
        * R (key down) — spawn a thread to run ``_handle_safe_pose``.
        * Any mapped direction key (key down/up) — update ``self._pressed``
          and recompute velocity.

        Intended to run in a dedicated daemon thread started by ``run``.
        """
        try:
            for event in self._device.read_loop():
                if self._exit_event.is_set():
                    break
                if event.type != ecodes.EV_KEY:
                    continue

                code, value = event.code, event.value

                if code in (ecodes.KEY_ESC, ecodes.KEY_X) and value == self._KEYSTATE_DOWN:
                    self._exit_event.set()
                    break

                if code == ecodes.KEY_R and value == self._KEYSTATE_DOWN:
                    threading.Thread(target=self._handle_safe_pose, daemon=True).start()
                    continue

                if code not in self._DIRECTIONS and code not in self._GRIPPER_KEYS:
                    continue

                # Direction keys are ignored entirely until Servo has
                # started — _handle_safe_pose clears _pressed anyway, so
                # tracking presses before that point would just be
                # discarded, and set_velocity()'d twists Servo isn't
                # listening to yet would have nothing to show for it.
                if not self._servo_started:
                    continue

                if value == self._KEYSTATE_DOWN:
                    with self._lock:
                        already_pressed = code in self._pressed
                        self._pressed.add(code)
                    if not already_pressed:
                        self._recompute_velocity()
                        key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                        print(f'{key_name} vx={self._controller.vx:.2f} vy={self._controller.vy:.2f} '
                              f'vz={self._controller.vz:.2f} wx={self._controller.wx:.2f} '
                              f'wy={self._controller.wy:.2f} wz={self._controller.wz:.2f} '
                              f'gripper={self._controller.gripper_vel:.2f}')
                elif value == self._KEYSTATE_UP:
                    with self._lock:
                        self._pressed.discard(code)
                    self._recompute_velocity()

        except OSError as e:
            self._controller.get_logger().error(f'Keyboard read loop failed: {e!r}')
        finally:
            self._exit_event.set()

    def run(self):
        """Open the keyboard device and run the input loop until exit.

        Opens the evdev device (returning early if this fails), prints the
        help banner, disables terminal echo where possible, starts the
        event-reading loop in a daemon thread, and blocks until the exit
        event is set. On exit, stops the controller's motion and restores
        the original terminal settings.
        """
        if not self._open_device():
            return
        print(HELP)

        stdin_fd = sys.stdin.fileno()
        old_term_settings = None
        try:
            old_term_settings = termios.tcgetattr(stdin_fd)
            new_term_settings = termios.tcgetattr(stdin_fd)
            new_term_settings[3] &= ~termios.ECHO
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, new_term_settings)
        except termios.error:
            old_term_settings = None

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()
            if old_term_settings is not None:
                termios.tcflush(stdin_fd, termios.TCIFLUSH)
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)


GAMEPAD_HELP = """
╔══════════════════════════════════════════════╗
║      Gamepad Servo Control (camera frame)    ║
╠══════════════════════════════════════════════╣
║  Right stick      — forward / back (X)       ║
║                     left / right   (Y)       ║
║  Left stick       — yaw                      ║
║                     pitch                    ║
║  L2 / R2          — roll                     ║
║  L2 / R2, Y held  — up / down      (Z)       ║
║  L1 / R1          — close / open gripper     ║
║  A                — safe pose + servo        ║
║  X                — exit                     ║
╚══════════════════════════════════════════════╝
"""


class GamepadInputLoop:
    """Reads sensor_msgs/Joy messages and drives a ``ServoController``.

    Replaces the raw-keyboard evdev input of ``KeyboardInputLoop`` with a
    subscription to the ``joy`` package's ``/joy`` topic (published by
    ``ros2 run joy joy_node``) — as this module's docstring already
    promises, ``ServoController`` itself needs no changes.

    Axis/button indices and rest values below were established by
    watching `ros2 topic echo /joy` live with this controller over
    Bluetooth:

    * Axes 0/1 (left stick) and 2/3 (right stick) rest at 0.0, X left =
      +1.0, X right = -1.0, Y forward = +1.0, Y back = -1.0.
    * Axes 4 and 5 (L2 / R2) rest at **+1.0** (released) and go to
      **-1.0** at full press — the opposite convention from the sticks.
      ``_trigger_amount`` below converts that to the same "0 at rest"
      shape as everything else in this class expects.
    * Buttons 0/2/3 are A/X/Y.
    """

    AXIS_LEFT_X = 0     # yaw
    AXIS_LEFT_Y = 1     # pitch
    AXIS_RIGHT_X = 2    # left / right
    AXIS_RIGHT_Y = 3    # forward / back
    AXIS_L2 = 4         # roll (normal) / up (Y held)
    AXIS_R2 = 5         # roll (normal) / down (Y held)

    BUTTON_SAFE_POSE = 0   # 'A' — move to safe pose + start servo
    BUTTON_Y = 3           # 'Y' — held to shift L2 / R2 from roll to up / down
    BUTTON_EXIT = 2        # 'X' — exit
    BUTTON_GRIPPER_CLOSE = 4  # L1
    BUTTON_GRIPPER_OPEN  = 5  # R1

    _DEADZONE = 0.2
    _JOY_TIMEOUT_SEC = 0.2
    _WATCHDOG_PERIOD_SEC = 0.1

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and subscribe to ``/joy``.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from gamepad input. Its own
                ``create_subscription`` is reused for the ``/joy`` topic
                so the callback runs on the node's existing executor —
                no separate thread is needed, unlike the keyboard's
                blocking evdev read loop.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._gripper_speed = controller.gripper_speed
        self._exit_event = threading.Event()
        # None means "no trustworthy baseline yet" — the first message
        # after startup or a /joy dropout only seeds this, it never
        # fires a rising-edge action (a gamepad can report a stale
        # "pressed" button on that first message, which was causing
        # spurious exits).
        self._prev_buttons = None
        self._shift_armed = {self.BUTTON_Y: False}
        self._safe_pose_running = threading.Lock()
        self._safe_pose_active = False
        self._prev_active = (False,) * 7

        # Teleop (axes -> velocity) is locked out until the first
        # safe-pose move (A) completes — Servo hasn't been started yet
        # at that point, so nothing would move anyway, and the arm's
        # real-world pose is unknown to this process until then. This
        # is a one-time latch: once armed it stays armed, including
        # across /joy dropouts (e.g. a Bluetooth reconnect) — there is
        # no re-arm button and no trip back to the fixed safe pose.
        self._last_joy_time = None
        self._joy_silent = False
        self._teleop_locked = True

        # A dropout does not re-lock teleop, but it does set this: the
        # first /joy message(s) after a dropout can report stale values
        # (observed live: full deflection with nothing touched) — likely
        # a freshly re-created Bluetooth HID device reporting a default
        # before its first real report arrives. So we force zero
        # velocity and wait, with no timeout, until one message reports
        # everything centered/released, then resume from wherever the
        # arm already is.
        self._joy_settling = False

        self._sub = controller.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._watchdog_timer = controller.create_timer(
            self._WATCHDOG_PERIOD_SEC, self._check_joy_timeout
        )

    @classmethod
    def _deadzone(cls, value: float) -> float:
        """Zero out small stick values so resting drift doesn't creep the arm."""
        return 0.0 if abs(value) < cls._DEADZONE else value

    def _axis(self, axes, index: int) -> float:
        """Return ``axes[index]`` with deadzone applied, or 0.0 if out of range."""
        if index >= len(axes):
            return 0.0
        return self._deadzone(axes[index])

    def _trigger_amount(self, axes, index: int) -> float:
        """Return how far a trigger (L2/R2) is pressed: 0.0 (released) .. 1.0 (full press).

        ``joy_node`` reports these axes resting at +1.0 and going to
        -1.0 at full press — inverted and offset from every other axis
        in this class, which rests at 0.0. Remapping it here means the
        deadzone, the settle-guard's "must be centered" check, and
        ``_route``'s arming logic can all keep treating 0.0 as "at
        rest" uniformly, without special-casing these two axes.
        """
        if index >= len(axes):
            return 0.0
        amount = (1.0 - axes[index]) / 2.0
        return 0.0 if amount < self._DEADZONE else amount

    def _button_pressed(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` is currently held down."""
        return index < len(buttons) and buttons[index] == 1

    def _button_rising_edge(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` was just pressed this message.

        Requires a real previous reading — see ``_prev_buttons`` in
        ``__init__`` for why ``None`` (no baseline yet) always reports
        no edge rather than comparing against an assumed all-zero state.
        """
        if self._prev_buttons is None:
            return False
        was_pressed = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        return self._button_pressed(buttons, index) and not was_pressed

    def _route(self, shift_button: int, held: bool, raw_value: float):
        """Route one combined trigger value to a (normal, shifted) pair.

        While ``shift_button`` is not held, ``raw_value`` is returned as
        ``(raw_value, 0.0)``. While held, it's routed to the second slot
        instead — but only once ``raw_value`` has passed back through the
        deadzone since the button was pressed (i.e. L2/R2 have been let
        go back to neutral at least once since Y was pressed), returning
        ``(0.0, 0.0)`` until then. This is what prevents an already-held
        trigger from producing a velocity jump the instant Y is pressed.
        Disarmed again as soon as ``shift_button`` is released.
        """
        if not held:
            self._shift_armed[shift_button] = False
            return raw_value, 0.0
        if not self._shift_armed[shift_button]:
            if raw_value == 0.0:
                self._shift_armed[shift_button] = True
            return 0.0, 0.0
        return 0.0, raw_value

    @staticmethod
    def _active_label(vx, vy, vz, wx, wy, wz, gripper_vel, y_held: bool) -> str:
        """Describe which physical control(s) are driving a nonzero command.

        Mirrors KeyboardInputLoop's per-key name in the feedback line,
        generalized to gamepad axes (several of which can be active at
        once, e.g. a diagonally-pushed stick).
        """
        parts = []
        if vx or vy:
            parts.append('right stick')
        if wy or wz:
            parts.append('left stick')
        if wx:
            parts.append('L2/R2')
        if vz:
            parts.append('Y+L2/R2')
        if gripper_vel:
            parts.append('L1/R1')
        return '+'.join(parts) if parts else ('Y' if y_held else 'idle')

    def _check_joy_timeout(self):
        """Stop the arm if no ``/joy`` message has arrived recently.

        Runs on the node's own timer, independent of message arrival, so
        a disconnected controller or a dead ``joy_node`` can't
        leave the last commanded velocity republishing forever — Servo's
        own command timeout does not help here because ServoController
        keeps re-publishing that last twist every tick regardless of
        whether new input has arrived. This only zeroes the current
        command; it does not lock teleop out, so control resumes on its
        own the moment ``/joy`` messages start arriving again (e.g. a
        Bluetooth reconnect) — no separate re-arm step.
        """
        if self._last_joy_time is None:
            return
        elapsed = (self._controller.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if elapsed > self._JOY_TIMEOUT_SEC:
            if not self._joy_silent:
                self._joy_silent = True
                self._joy_settling = True
                self._prev_buttons = None
                self._controller.get_logger().warn(
                    f'/joy timed out after {elapsed:.2f}s — stopping arm.'
                )
            self._controller.stop()

    def _on_joy(self, msg: Joy):
        """Translate one Joy snapshot into a velocity command and edge-triggered actions."""
        axes = msg.axes
        buttons = msg.buttons

        if self._joy_silent:
            self._joy_silent = False
            self._controller.get_logger().info('/joy resumed.')
        self._last_joy_time = self._controller.get_clock().now()

        # Button edges are always processed, even while teleop is locked
        # out — buttons are digital (no analog drift), and the safe-pose
        # button is the only way to clear the lock, so it must keep
        # working while locked or the arm could never be armed at all.
        safe_pose_pressed = self._button_rising_edge(buttons, self.BUTTON_SAFE_POSE)
        exit_pressed = self._button_rising_edge(buttons, self.BUTTON_EXIT)
        self._prev_buttons = list(buttons)

        if exit_pressed:
            self._exit_event.set()

        if safe_pose_pressed:
            threading.Thread(target=self._handle_safe_pose, daemon=True).start()

        if self._teleop_locked or self._safe_pose_active:
            self._controller.stop()
            return

        if self._joy_settling:
            centered = (
                all(
                    self._axis(axes, i) == 0.0
                    for i in (self.AXIS_LEFT_X, self.AXIS_LEFT_Y,
                              self.AXIS_RIGHT_X, self.AXIS_RIGHT_Y)
                )
                and self._trigger_amount(axes, self.AXIS_L2) == 0.0
                and self._trigger_amount(axes, self.AXIS_R2) == 0.0
            )
            self._controller.stop()
            if centered:
                self._joy_settling = False
                self._controller.get_logger().info('Sticks centered — resuming control.')
            return

        vy = self._axis(axes, self.AXIS_RIGHT_X) * self._linear_speed
        vx = self._axis(axes, self.AXIS_RIGHT_Y) * self._linear_speed

        wz = self._axis(axes, self.AXIS_LEFT_X) * self._angular_speed
        wy = self._axis(axes, self.AXIS_LEFT_Y) * self._angular_speed

        trigger_diff = self._trigger_amount(axes, self.AXIS_R2) - self._trigger_amount(axes, self.AXIS_L2)
        y_held = self._button_pressed(buttons, self.BUTTON_Y)
        roll, updown = self._route(self.BUTTON_Y, y_held, trigger_diff)
        wx = roll * self._angular_speed
        vz = updown * self._linear_speed

        gripper_open = self._button_pressed(buttons, self.BUTTON_GRIPPER_OPEN)
        gripper_close = self._button_pressed(buttons, self.BUTTON_GRIPPER_CLOSE)
        gripper_vel = (
            (1.0 if gripper_open else 0.0) - (1.0 if gripper_close else 0.0)
        ) * self._gripper_speed

        self._controller.set_velocity(vx, vy, vz, wx, wy, wz, gripper_vel=gripper_vel)

        active = (vx != 0.0, vy != 0.0, vz != 0.0, wx != 0.0, wy != 0.0, wz != 0.0, gripper_vel != 0.0)
        if active != self._prev_active and any(active):
            label = self._active_label(vx, vy, vz, wx, wy, wz, gripper_vel, y_held)
            print(f'{label} vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} '
                  f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f} gripper={gripper_vel:.4f}')
        self._prev_active = active

    def _handle_safe_pose(self):
        """Stop motion and move to the safe pose (mirrors KeyboardInputLoop's 'r').

        Guarded by a non-blocking lock so a second button press while a
        move is already in progress is ignored instead of racing a
        redundant safe-pose goal against the first. Also sets
        ``_safe_pose_active`` for the duration so ``_on_joy`` ignores
        stick/trigger input while it's set — otherwise a stick held
        during the move would resume Cartesian motion the instant
        ``start_servo()`` re-enables Servo, before the operator has a
        chance to let go.

        This is also the only place ``_teleop_locked`` is cleared (see
        its declaration in ``__init__``).
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        self._safe_pose_active = True
        try:
            self._controller.stop()
            print('Moving to safe pose...')
            if self._controller.move_to_safe_pose():
                print('Starting servo...')
                self._controller.start_servo()
                self._teleop_locked = False
                self._controller.get_logger().info('Teleop enabled.')
            else:
                print('Safe pose failed — Servo not started.')
        finally:
            self._safe_pose_active = False
            self._safe_pose_running.release()

    def run(self):
        """Print the help banner and block until the exit button is pressed."""
        print(GAMEPAD_HELP)
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()


def _run_teleop(controller: 'ServoController', input_loop) -> None:
    """Spin ``controller`` in a background thread and run ``input_loop`` until exit.

    Shared by ``main`` (keyboard) and ``main_gamepad`` (gamepad): both
    input loops expose the same ``run()`` contract (block until an exit
    condition, leave the controller stopped), so the ROS lifecycle
    around them doesn't need to be duplicated per input source.
    """
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        input_loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        if not controller.stop_servo():
            controller.get_logger().warn(
                'Could not confirm Servo stopped before shutdown — '
                'it may still be active.'
            )
        controller.destroy_node()
        rclpy.shutdown()


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, KeyboardInputLoop(controller))


def main_gamepad():
    """Entry point: initialize ROS2, run the gamepad input loop, and clean up.

    Requires a running ``joy`` publisher (``ros2 run joy joy_node``).
    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, GamepadInputLoop(controller))


if __name__ == '__main__':
    main()