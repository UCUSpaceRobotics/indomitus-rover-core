#!/usr/bin/env python3
"""
Keyboard control node for MoveIt Servo.

Reads keyboard input and sends Cartesian velocity commands to MoveIt Servo.
Designed to be modular — core logic is in ServoController class,
so switching to gamepad (joy package) requires only replacing the input source.

Controls:
    Translation:
        w / s  — forward / backward  (X)
        a / d  — up / down           (Y)
        q / e  — left / right        (Z)
    Rotation:
        u / o  — roll CW / CCW
        i / k  — pitch up / down
        j / l  — yaw left / right
    Other:
        r      — move to safe pose + start servo
        SPACE  — stop
        ESC/x  — exit
"""

import sys
import threading
import tty
import termios

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int8

# ── Constants ─────────────────────────────────────────────────────────────────

SERVO_TOPIC = '/servo_node/delta_twist_cmds'
SERVO_START_SERVICE = '/servo_node/start_servo'
SERVO_STOP_SERVICE = '/servo_node/stop_servo'
COMMAND_FRAME = 'arm_camera_link'
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.3
PUBLISH_RATE = 50.0

SAFE_POSE = [0.0, 1.2, -1.0, 0.8, 0.5, 0.0]
SAFE_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]

# ── ServoController ───────────────────────────────────────────────────────────

class ServoController(Node):

    def __init__(self):
        super().__init__('keyboard_servo_node')

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0

        self._pub = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)
        self._start_client = self.create_client(Trigger, SERVO_START_SERVICE)
        self._stop_client = self.create_client(Trigger, SERVO_STOP_SERVICE)
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/indomitus_arm_controller/follow_joint_trajectory'
        )
        self._timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish)

        self._servo_status = 0
        self._status_sub = self.create_subscription(
            Int8,
            '/servo_node/status',
            self._on_servo_status,
            10
        )

        self.get_logger().info('ServoController ready.')

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

    def stop_servo(self):
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo stop service not available')
            return
        self._stop_client.call_async(Trigger.Request())

    def move_to_safe_pose(self):
        self.stop()
        self.stop_servo()

        import time
        time.sleep(0.5)

        if not self._traj_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        traj = JointTrajectory()
        traj.joint_names = SAFE_POSE_JOINTS

        point = JointTrajectoryPoint()
        point.positions = SAFE_POSE
        point.velocities = [0.0] * len(SAFE_POSE)
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

    def _on_servo_status(self, msg):
        if msg.data == 2 and self._servo_status != 2:
            self.get_logger().warn('Servo error detected, restarting...')
            self.start_servo()
        self._servo_status = msg.data

    def _publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = COMMAND_FRAME
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz
        msg.twist.angular.x = self.wx
        msg.twist.angular.y = self.wy
        msg.twist.angular.z = self.wz
        self._pub.publish(msg)

# ── KeyboardInput ─────────────────────────────────────────────────────────────

KEY_MAP = {
    'w': ( LINEAR_SPEED,  0.0,          0.0,          0.0,           0.0,           0.0),
    's': (-LINEAR_SPEED,  0.0,          0.0,          0.0,           0.0,           0.0),
    'a': ( 0.0,           LINEAR_SPEED, 0.0,          0.0,           0.0,           0.0),
    'd': ( 0.0,          -LINEAR_SPEED, 0.0,          0.0,           0.0,           0.0),
    'q': ( 0.0,           0.0,          LINEAR_SPEED, 0.0,           0.0,           0.0),
    'e': ( 0.0,           0.0,         -LINEAR_SPEED, 0.0,           0.0,           0.0),
    'u': ( 0.0,           0.0,          0.0,          ANGULAR_SPEED, 0.0,           0.0),
    'o': ( 0.0,           0.0,          0.0,         -ANGULAR_SPEED, 0.0,           0.0),
    'i': ( 0.0,           0.0,          0.0,          0.0,           ANGULAR_SPEED, 0.0),
    'k': ( 0.0,           0.0,          0.0,          0.0,          -ANGULAR_SPEED, 0.0),
    'j': ( 0.0,           0.0,          0.0,          0.0,           0.0,           ANGULAR_SPEED),
    'l': ( 0.0,           0.0,          0.0,          0.0,           0.0,          -ANGULAR_SPEED),
}

HELP = """
╔══════════════════════════════════════════════╗
║     Keyboard Servo Control (camera frame)    ║
╠══════════════════════════════════════════════╣
║  Translation:                                ║
║    w / s  — forward / backward  (X)          ║
║    a / d  — up / down           (Y)          ║
║    q / e  — left / right        (Z)          ║
║  Rotation:                                   ║
║    u / o  — roll CW / CCW                    ║
║    i / k  — pitch up / down                  ║
║    j / l  — yaw left / right                 ║
║  Other:                                      ║
║    r      — move to safe pose + start servo  ║
║    SPACE  — stop                             ║
║    ESC/x  — exit                             ║
╚══════════════════════════════════════════════╝
"""


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_loop(controller: ServoController):
    print(HELP)

    while rclpy.ok():
        key = get_key()

        if key in ('\x1b', 'x'):
            controller.stop()
            print('\nExiting...')
            break

        if key == 'r':
            print('Moving to safe pose...')
            controller.move_to_safe_pose()
            print('Starting servo...')
            controller.start_servo()
            continue

        if key == ' ':
            controller.stop()
            print('STOP')
            continue

        if key in KEY_MAP:
            vx, vy, vz, wx, wy, wz = KEY_MAP[key]
            controller.set_velocity(vx, vy, vz, wx, wy, wz)
            print(f'[{key}] vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} '
                  f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f}')
        else:
            print(f'Unknown key: {repr(key)}')

# ── Main ──────────────────────────────────────────────────────────────────────

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
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()