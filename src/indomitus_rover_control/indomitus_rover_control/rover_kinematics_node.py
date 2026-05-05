#!/usr/bin/env python3
"""
Rover Controller Node
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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from indomitus_msgs.msg import WheelTargets


# Rover geometry
WHEELBASE    = 1.20  # meters, distance between front and rear axle
TRACK_WIDTH  = 0.80  # meters, distance between left and right wheels
WHEEL_RADIUS = 0.15

MAX_STEER_ANGLE = math.radians(45.0)

L2 = WHEELBASE   / 2.0
W2 = TRACK_WIDTH / 2.0


class RoverController(Node):

    def __init__(self):
        super().__init__('rover_controller')

        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(
            WheelTargets,
            'wheel_targets',
            10
        )

        self.get_logger().info('RoverController started — listening on /cmd_vel')

  
    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        angles, speeds = self.compute_wheel_commands(vx, vy, wz)

        out = WheelTargets()
        out.fl_angle = float(angles[0])
        out.fr_angle = float(angles[1])
        out.rl_angle = float(angles[2])
        out.rr_angle = float(angles[3])
        out.fl_speed = float(speeds[0])
        out.fr_speed = float(speeds[1])
        out.rl_speed = float(speeds[2])
        out.rr_speed = float(speeds[3])

        self.pub.publish(out)

        self.get_logger().debug(
            f'FL={math.degrees(angles[0]):+.1f}° FR={math.degrees(angles[1]):+.1f}° '
            f'RL={math.degrees(angles[2]):+.1f}° RR={math.degrees(angles[3]):+.1f}° | '
            f'FL={speeds[0]:+.2f} FR={speeds[1]:+.2f} '
            f'RL={speeds[2]:+.2f} RR={speeds[3]:+.2f} rad/s'
        )


    def compute_wheel_commands(self, vx: float, vy: float, wz: float):
        """
        Returns:
            angles : [FL, FR, RL, RR]  in radians
            speeds : [FL, FR, RL, RR]  in rad/s
        """

        # Case 1: pure spin on the spot (vx≈0, vy≈0, wz≠0)
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            if abs(wz) < 1e-4:
                return [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

            spin_angle = math.radians(45.0)
            fl_angle =  spin_angle
            fr_angle = -spin_angle
            rl_angle = -spin_angle
            rr_angle =  spin_angle

            r = math.hypot(L2, W2)
            wheel_speed = abs(wz) * r / WHEEL_RADIUS
            sign = math.copysign(1.0, wz)

            return [fl_angle, fr_angle, rl_angle, rr_angle], [sign * wheel_speed] * 4

        # Case 2: straight line
        if abs(wz) < 1e-4:
            wheel_speed = vx / WHEEL_RADIUS
            return [0.0, 0.0, 0.0, 0.0], [wheel_speed] * 4

        # Case 3: Ackermann / general motion
        icr_x = vy / wz
        icr_y = vx / wz

        wheel_pos = {
            'FL': ( L2,  W2),
            'FR': ( L2, -W2),
            'RL': (-L2,  W2),
            'RR': (-L2, -W2),
        }

        angles = []
        speeds = []

        for _, (wx, wy) in wheel_pos.items():
            dx = wx - icr_x
            dy = wy - icr_y

            angle = math.atan2(dx, -dy)
            r_wheel = math.hypot(dx, dy)
            wheel_speed = wz * r_wheel / WHEEL_RADIUS

            angles.append(angle)
            speeds.append(wheel_speed)

        fl_a, fr_a, rl_a, rr_a = angles

        # Rear mirrors front
        rl_a = -fl_a
        rr_a = -fr_a

        angles = [fl_a, fr_a, rl_a, rr_a]
        angles = [self._clamp(a, -MAX_STEER_ANGLE, MAX_STEER_ANGLE) for a in angles]

        return angles, speeds

    # Helpers

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))


# Entry point

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
