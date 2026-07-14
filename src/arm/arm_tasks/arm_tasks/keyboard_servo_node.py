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

    r      — move to safe pose + start servo
    ESC/x  — exit
"""

import sys
import threading
import termios
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import evdev
from evdev import ecodes


DEFAULT_LINEAR_SPEED  = 0.25
DEFAULT_ANGULAR_SPEED = 0.75
DEFAULT_PUBLISH_RATE  = 50.0
DEFAULT_COMMAND_FRAME = 'arm_camera_link'
DEFAULT_SAFE_POSE     = [0.0, 1.2, -1.0, 0.8, 0.5, 0.0]
DEFAULT_KEYBOARD_DEVICE_PATH = '/dev/input/event3'

SAFE_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]

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
        self.declare_parameter('publish_rate',  DEFAULT_PUBLISH_RATE)
        self.declare_parameter('command_frame', DEFAULT_COMMAND_FRAME)
        self.declare_parameter('safe_pose',     DEFAULT_SAFE_POSE)
        self.declare_parameter('keyboard_device_path', DEFAULT_KEYBOARD_DEVICE_PATH)

        self._linear_speed  = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._publish_rate  = self.get_parameter('publish_rate').value
        self._command_frame = self.get_parameter('command_frame').value
        self._safe_pose     = list(self.get_parameter('safe_pose').value)
        self._keyboard_device_path = self.get_parameter('keyboard_device_path').value

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0

        self._pub = self.create_publisher(TwistStamped, 'servo_node/delta_twist_cmds', 10)
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
    def keyboard_device_path(self) -> str:
        """Return the filesystem path of the keyboard input device (evdev)."""
        return self._keyboard_device_path

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0):
        """Set the current Cartesian velocity command.

        Args:
            vx: Linear velocity along the X axis, in meters per second.
            vy: Linear velocity along the Y axis, in meters per second.
            vz: Linear velocity along the Z axis, in meters per second.
            wx: Angular velocity about the X axis, in radians per second.
            wy: Angular velocity about the Y axis, in radians per second.
            wz: Angular velocity about the Z axis, in radians per second.

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
        the goal finished. There is deliberately no wall-clock timeout: under
        Gazebo with a low real-time factor the trajectory can legitimately
        take much longer in wall time, and this method runs on a dedicated
        thread, so waiting does not stall keyboard handling.

        Returns:
            bool: True once the trajectory result was received; False if
            Servo could not be confirmed stopped, the trajectory action
            server was unavailable, or the goal was rejected.
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
        goal_rejected = threading.Event()

        def goal_response_cb(future):
            """Handle the trajectory action's goal-acceptance response.

            Args:
                future: Future resolving to the goal handle returned by
                    ``send_goal_async``.
            """
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                goal_rejected.set()
                done_event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: done_event.set())

        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        done_event.wait()
        if goal_rejected.is_set():
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
        """Publish the current velocity state as a stamped Twist message.

        Called periodically by the internal timer at ``publish_rate`` Hz;
        builds a ``TwistStamped`` message from the current ``vx``..``wz``
        attributes, stamps it with the current time and ``command_frame``,
        and publishes it.
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._command_frame
        msg.twist.linear.x  = self.vx
        msg.twist.linear.y  = self.vy
        msg.twist.linear.z  = self.vz
        msg.twist.angular.x = self.wx
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
        self._device_path = controller.keyboard_device_path
        self._lock = threading.Lock()
        self._pressed = set()
        self._exit_event = threading.Event()
        self._device = None
        self._read_thread = None

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
        vx = vy = vz = wx = wy = wz = 0.0
        with self._lock:
            active = list(self._pressed)
        for code in active:
            d = self._DIRECTIONS.get(code)
            if d is None:
                continue
            vx += d[0]
            vy += d[1]
            vz += d[2]
            wx += d[3]
            wy += d[4]
            wz += d[5]
        self._controller.set_velocity(
            vx * self._linear_speed, vy * self._linear_speed, vz * self._linear_speed,
            wx * self._angular_speed, wy * self._angular_speed, wz * self._angular_speed,
        )

    def _handle_safe_pose(self):
        """Clear pressed keys, stop motion, and move to the safe pose.

        Intended to run in its own thread (spawned from ``_read_loop``) so
        that the blocking safe-pose and servo-start calls do not stall
        keyboard event processing.
        """
        with self._lock:
            self._pressed.clear()
        self._controller.stop()
        print('Moving to safe pose...')
        self._controller.move_to_safe_pose()
        print('Starting servo...')
        self._controller.start_servo()

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

                if code not in self._DIRECTIONS:
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
                              f'wy={self._controller.wy:.2f} wz={self._controller.wz:.2f}')
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


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    Initializes rclpy, creates the ``ServoController`` node, spins it in a
    background daemon thread, then runs the blocking ``KeyboardInputLoop``
    on the main thread. On exit (normal, via ESC/X, or ``KeyboardInterrupt``),
    stops any motion, confirms Servo has stopped, destroys the node, and
    shuts down rclpy.
    """
    rclpy.init()
    controller = ServoController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    input_loop = KeyboardInputLoop(controller)

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


if __name__ == '__main__':
    main()