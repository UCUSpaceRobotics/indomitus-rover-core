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
        u / o  — pitch up / down      (not available: 5 DOF only)
        j / l  — yaw left / right

    r      — move to safe pose + start servo
    ESC/x  — exit
"""

import sys
import threading
import tty
import termios
import select
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import ServoStatus


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

SERVO_STATUS_HALT_FOR_SINGULARITY = 2
SERVO_STATUS_OK = 0


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
            ServoStatus,
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
        self.stop_servo()

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

    def _on_servo_status(self, msg: ServoStatus):
        """
        Handle Servo status updates.
        Automatically restarts Servo on singularity halt.
        Other halt states (joint limits, collisions) are logged but not
        auto-recovered — they require operator action via safe pose (r key).
        """
        code = msg.code
        if code != self._servo_status:
            if code == SERVO_STATUS_HALT_FOR_SINGULARITY:
                self.get_logger().warn(
                    'Servo halted: near singularity — restarting. '
                    'Press r to move to safe pose if arm is stuck.'
                )
                self.start_servo()
            elif code != SERVO_STATUS_OK:
                self.get_logger().warn(
                    f'Servo halted with status code {code}. '
                    'Press r to move to safe pose and restart.'
                )
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
║    u / o  — pitch (not available)            ║
║    j / l  — yaw left / right                 ║
║  Other:                                      ║
║    r      — move to safe pose + start servo  ║
║    ESC/x  — exit                             ║
╚══════════════════════════════════════════════╝
"""


def build_key_map(linear_speed: float, angular_speed: float) -> dict:
    """Build key → velocity mapping from speed parameters."""
    return {
        'w': ( linear_speed,  0.0,          0.0,           0.0,           0.0,           0.0),
        's': (-linear_speed,  0.0,          0.0,           0.0,           0.0,           0.0),
        'a': ( 0.0,           linear_speed, 0.0,           0.0,           0.0,           0.0),
        'd': ( 0.0,          -linear_speed, 0.0,           0.0,           0.0,           0.0),
        'q': ( 0.0,           0.0,          linear_speed,  0.0,           0.0,           0.0),
        'e': ( 0.0,           0.0,         -linear_speed,  0.0,           0.0,           0.0),
        'i': ( 0.0,           0.0,          0.0,           0.0,           angular_speed, 0.0),
        'k': ( 0.0,           0.0,          0.0,           0.0,          -angular_speed, 0.0),
        'j': ( 0.0,           0.0,          0.0,           0.0,           0.0,           angular_speed),
        'l': ( 0.0,           0.0,          0.0,           0.0,           0.0,          -angular_speed),
    }


def get_key(timeout: float = 0.05):
    """
    Read one key from stdin with a timeout.
    Returns None if no key was pressed within timeout.
    Note: hold-to-move relies on terminal key auto-repeat.
    For explicit press/release events, a library like pynput is needed.
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_loop(controller: ServoController):
    """
    Main keyboard input loop.
    Moves while key is held, stops on release (via auto-repeat timeout).
    """
    key_map = build_key_map(controller.linear_speed, controller.angular_speed)
    print(HELP)
    current_key = None

    while rclpy.ok():
        key = get_key(timeout=0.05)

        if key is None:
            if current_key is not None:
                controller.stop()
                current_key = None
            continue

        if key in ('\x1b', 'x'):
            controller.stop()
            print('\nExiting...')
            break

        if key == 'r':
            current_key = None
            controller.stop()
            print('Moving to safe pose...')
            controller.move_to_safe_pose()
            print('Starting servo...')
            controller.start_servo()
            continue

        if key in key_map:
            if key != current_key:
                vx, vy, vz, wx, wy, wz = key_map[key]
                controller.set_velocity(vx, vy, vz, wx, wy, wz)
                print(f'[{key}] vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} '
                      f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f}')
                current_key = key
        else:
            controller.stop()
            current_key = None
            print(f'Unknown key: {repr(key)}')

def main():
    rclpy.init()
    controller = ServoController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        keyboard_loop(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.stop_servo()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()