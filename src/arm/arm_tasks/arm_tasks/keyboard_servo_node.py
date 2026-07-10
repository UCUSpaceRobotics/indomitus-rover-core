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
from pynput import keyboard as pynput_keyboard


DEFAULT_LINEAR_SPEED  = 0.1
DEFAULT_ANGULAR_SPEED = 0.3
DEFAULT_PUBLISH_RATE  = 50.0
DEFAULT_COMMAND_FRAME = 'arm_camera_link'
DEFAULT_SAFE_POSE     = [0.0, 1.2, -1.0, 0.8, 0.5, 0.0]

SAFE_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]

# Mirrors StatusCode from moveit_servo/utils/datatype.hpp
# (same values moveit_msgs/ServoStatus uses in its `code` field,
#  but here they arrive as a plain std_msgs/Int8 on Humble).
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
    """
    Core Servo controller.
    Publishes TwistStamped commands to MoveIt Servo.
    Input source (keyboard, gamepad) is injected from outside.
    To switch to gamepad — replace keyboard_loop with gamepad_loop,
    ServoController stays unchanged.
    """

    def __init__(self):
        super().__init__('keyboard_servo_node')

        self.declare_parameter('linear_speed',  DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('publish_rate',  DEFAULT_PUBLISH_RATE)
        self.declare_parameter('command_frame', DEFAULT_COMMAND_FRAME)
        self.declare_parameter('safe_pose',     DEFAULT_SAFE_POSE)

        self._linear_speed  = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._publish_rate  = self.get_parameter('publish_rate').value
        self._command_frame = self.get_parameter('command_frame').value
        self._safe_pose     = list(self.get_parameter('safe_pose').value)

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
        return self._linear_speed

    @property
    def angular_speed(self) -> float:
        return self._angular_speed

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx
        self.wy = wy
        self.wz = wz

    def stop(self):
        """
        Zero out commanded velocity.

        NOTE: this only publishes a zero TwistStamped — it does NOT pause/disable
        MoveIt Servo itself. Servo keeps running and will keep processing whatever
        is published on delta_twist_cmds. Use stop_servo() to actually pause Servo
        via its service.
        """
        self.set_velocity()

    def stop_servo(self) -> bool:
        """Stop MoveIt Servo and wait for confirmation."""
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo stop service not available')
            return False

        done_event = threading.Event()

        def _cb(future):
            done_event.set()

        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=3.0):
            self.get_logger().warn('Servo stop timed out')
            return False
        return True

    def move_to_safe_pose(self):
        """Stop Servo and move arm to safe pose."""
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

        def goal_response_cb(future):
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                done_event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: done_event.set())

        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        done = done_event.wait(timeout=8.0)
        if done:
            self.get_logger().info('Safe pose reached!')
        else:
            self.get_logger().warn('Safe pose timeout!')
        return done

    def start_servo(self):
        """Start MoveIt Servo."""
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servo start service not available')
            return
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_result)

    def _on_start_result(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Servo started successfully')
            else:
                self.get_logger().warn(f'Servo start failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Servo start error: {e}')

    def _on_servo_status(self, msg: Int8):
        """
        Handle Servo status updates.

        Only HALT_FOR_SINGULARITY is auto-recovered: it's a transient, expected
        condition during normal teleop and safe to just restart Servo for.
        All other halt states (HALT_FOR_COLLISION, JOINT_BOUND, etc.) are left
        for the operator to resolve via safe pose (r key) — auto-restarting those
        could mean continuing to command motion into a collision or a joint limit.
        No logging is done here — status is tracked silently.

        Note: on this Humble build, moveit_servo publishes status as a plain
        std_msgs/Int8 (msg.data), not moveit_msgs/ServoStatus (msg.code).
        """
        code = msg.data
        if code != self._servo_status:
            if code == SERVO_STATUS_HALT_FOR_SINGULARITY:
                self.start_servo()
        self._servo_status = code

    def _publish(self):
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


def build_key_map(linear_speed: float, angular_speed: float) -> dict:
    """
    Build key → velocity mapping from speed parameters.

    Kept for backward compatibility / reference only — the running input
    loop now uses KeyboardInputLoop._DIRECTIONS instead, since it needs
    per-key unit directions (not pre-scaled full vectors) to combine
    multiple simultaneously held keys.
    """
    return {
        'w': ( linear_speed,  0.0,          0.0,           0.0,           0.0,           0.0),
        's': (-linear_speed,  0.0,          0.0,           0.0,           0.0,           0.0),
        'a': ( 0.0,           linear_speed, 0.0,           0.0,           0.0,           0.0),
        'd': ( 0.0,          -linear_speed, 0.0,           0.0,           0.0,           0.0),
        'q': ( 0.0,           0.0,          linear_speed,  0.0,           0.0,           0.0),
        'e': ( 0.0,           0.0,         -linear_speed,  0.0,           0.0,           0.0),
        'i': ( 0.0,           0.0,          0.0,           angular_speed, 0.0,           0.0),
        'k': ( 0.0,           0.0,          0.0,          -angular_speed, 0.0,           0.0),
        'u': ( 0.0,           0.0,          0.0,           0.0,           angular_speed, 0.0),
        'o': ( 0.0,           0.0,          0.0,           0.0,          -angular_speed, 0.0),
        'j': ( 0.0,           0.0,          0.0,           0.0,           0.0,           angular_speed),
        'l': ( 0.0,           0.0,          0.0,           0.0,           0.0,          -angular_speed),
    }


class KeyboardInputLoop:
    """
    Reads keyboard input using explicit press/release events (pynput),
    instead of relying on terminal key auto-repeat.

    Movement is tied directly to physical key state: velocity is recomputed
    from the current set of held-down keys every time a key goes down or up.
    This also means multiple keys can be held at once for combined motion
    (e.g. w + q moves forward and up together), which was not reliably
    possible with the old auto-repeat/timeout approach.

    NOTE: pynput's keyboard listener needs either an X11 display (default
    backend) or access to /dev/input (evdev backend, usually needs the user
    to be in the 'input' group or run as root). Plain headless SSH sessions
    without a display and without input-device permissions will not receive
    key events — run this on the robot's desktop session, or set up evdev
    access, if you hit that.
    """

    # Maps a character key to (vx, vy, vz, wx, wy, wz) *unit* direction.
    # Actual speed is applied when combining active keys.
    _DIRECTIONS = {
        'w': ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        's': (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        'a': ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
        'd': ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0),
        'q': ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0),
        'e': ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
        'i': ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0),
        'k': ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0),
        'u': ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0),
        'o': ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0),
        'j': ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0),
        'l': ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0),
    }

    def __init__(self, controller: 'ServoController'):
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._lock = threading.Lock()
        self._pressed = set()
        self._exit_event = threading.Event()
        self._listener = None

    @staticmethod
    def _key_to_char(key):
        """Normalize a pynput key event to a lowercase character, or None."""
        try:
            return key.char.lower()
        except AttributeError:
            return None

    def _recompute_velocity(self):
        """Sum unit directions of all currently held movement keys and apply speed."""
        vx = vy = vz = wx = wy = wz = 0.0
        with self._lock:
            active = list(self._pressed)
        for k in active:
            d = self._DIRECTIONS.get(k)
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

    def _on_press(self, key):
        if key in (pynput_keyboard.Key.esc,):
            self._exit_event.set()
            return False  # stop listener

        char = self._key_to_char(key)
        if char is None:
            return

        if char == 'x':
            self._exit_event.set()
            return False

        if char == 'r':
            # Handle safe-pose in a separate thread so we don't block the
            # listener callback (and therefore other key events) while it runs.
            threading.Thread(target=self._handle_safe_pose, daemon=True).start()
            return

        if char in self._DIRECTIONS:
            with self._lock:
                already_pressed = char in self._pressed
                self._pressed.add(char)
            if not already_pressed:
                self._recompute_velocity()
                print(f'[{char} down] vx={self._controller.vx:.2f} vy={self._controller.vy:.2f} '
                      f'vz={self._controller.vz:.2f} wx={self._controller.wx:.2f} '
                      f'wy={self._controller.wy:.2f} wz={self._controller.wz:.2f}')

    def _on_release(self, key):
        char = self._key_to_char(key)
        if char is None or char not in self._DIRECTIONS:
            return
        with self._lock:
            self._pressed.discard(char)
        self._recompute_velocity()

    def _handle_safe_pose(self):
        with self._lock:
            self._pressed.clear()
        self._controller.stop()
        print('Moving to safe pose...')
        self._controller.move_to_safe_pose()
        print('Starting servo...')
        self._controller.start_servo()

    def run(self):
        print(HELP)
        self._listener = pynput_keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()
            if self._listener.running:
                self._listener.stop()

def main():
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