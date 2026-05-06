#!/usr/bin/env python3
"""
Rover Controller Node.

Converts cmd_vel (Twist) → wheel angles + speeds for 4-wheel steering rover.

Steering geometry:
  - Front and rear wheels steer MIRRORED:
    if front-left = +θ, then rear-left = -θ
  - Ackermann-like: inner wheel turns sharper than outer

Wheel layout (top view):
        front
   FL -------- FR
   |            |
   |   (center) |
   |            |
   RL -------- RR
        rear

Published message (indomitus_msgs/WheelTargets):
  fl_angle, fr_angle, rl_angle, rr_angle  — radians
  fl_speed, fr_speed, rl_speed, rr_speed  — rad/s
"""

import math

from geometry_msgs.msg import Twist
from indomitus_msgs.msg import WheelTargets

import rclpy
from rclpy.node import Node


class RoverController(Node):

    def __init__(self):
        super().__init__('rover_controller')

        self.declare_parameter('wheelbase', 1.20)
        self.declare_parameter('track_width', 0.80)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('max_steer_deg', 90.0)

        self._read_params()

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(
            WheelTargets,
            '/wheel_targets',
            10
        )

        self.get_logger().info('RoverController started — listening on /cmd_vel')

    def _read_params(self):
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steer = math.radians(self.get_parameter('max_steer_deg').value)

        self.L2 = self.wheelbase / 2.0
        self.W2 = self.track_width / 2.0

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        angles, speeds = self.compute_wheel_commands(vx, vy, wz)

        out = WheelTargets()
        out.fl_angle = angles[0]
        out.fr_angle = angles[1]
        out.rl_angle = angles[2]
        out.rr_angle = angles[3]
        out.fl_speed = speeds[0]
        out.fr_speed = speeds[1]
        out.rl_speed = speeds[2]
        out.rr_speed = speeds[3]

        self.pub.publish(out)

        self.get_logger().debug(
            f'FL={math.degrees(angles[0]):+.1f}° FR={math.degrees(angles[1]):+.1f}° '
            f'RL={math.degrees(angles[2]):+.1f}° RR={math.degrees(angles[3]):+.1f}° | '
            f'FL={speeds[0]:+.2f} FR={speeds[1]:+.2f} '
            f'RL={speeds[2]:+.2f} RR={speeds[3]:+.2f} rad/s'
        )

    def compute_wheel_commands(self, vx: float, vy: float, wz: float):
        """
        Compute wheel angles and speeds from every wheel.

        Returns:
            angles : [FL, FR, RL, RR]  in radians
            speeds : [FL, FR, RL, RR]  in rad/s
        """
        # Case 1: pure spin on the spot (vx≈0, vy≈0, wz≠0)
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            if abs(wz) < 1e-4:
                return [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

            spin_angle = math.radians(45.0)

            r = math.hypot(self.L2, self.W2)
            wheel_speed = abs(wz) * r / self.wheel_radius
            sign = math.copysign(1.0, wz)

            return [-spin_angle, spin_angle, spin_angle, -spin_angle], \
                   [-wheel_speed*sign, wheel_speed*sign, -wheel_speed*sign, wheel_speed*sign]

        # Case 2: straight line
        if abs(wz) < 1e-4:
            wheel_speed = vx / self.wheel_radius
            return [0.0, 0.0, 0.0, 0.0], [wheel_speed] * 4

        # Case 3: Ackermann / general motion
        icr_x = abs(vy) / wz
        icr_y = abs(vx) / wz

        print(f'ICR: x={icr_x:.2f} m, y={icr_y:.2f} m')

        wheel_pos = {
            'FL': (self.L2, self.W2),
            'FR': (self.L2, -self.W2),
            'RL': (-self.L2, self.W2),
            'RR': (-self.L2, -self.W2),
        }

        angles = []
        speeds = []

        for _, (wx, wy) in wheel_pos.items():
            vx_w = vx - wz * wy
            vy_w = vy + wz * wx

            angle = math.atan2(vy_w, vx_w)
            speed = math.hypot(vx_w, vy_w) / self.wheel_radius

            angles.append(angle)
            # speeds.append(0.0)
            speeds.append(speed)

        fl_a, fr_a, rl_a, rr_a = angles

        angles = [fl_a, fr_a, rl_a, rr_a]
        angles = [self._clamp(a, -self.max_steer, self.max_steer) for a in angles]

        return angles, speeds

    # Helpers

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))


def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
