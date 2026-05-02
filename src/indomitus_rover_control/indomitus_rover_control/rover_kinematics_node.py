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

Published message (Float32MultiArray), 8 values in order:
  [FL_angle, FR_angle, RL_angle, RR_angle,
   FL_speed, FR_speed, RL_speed, RR_speed]

  angles in radians, speeds in rad/s
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


# ── Rover geometry ──────────────────────────────────────────────────────────
WHEELBASE   = 1.20   # meters, distance between front and rear axle
TRACK_WIDTH = 0.80   # meters, distance between left and right wheels
WHEEL_RADIUS = 0.10  # meters — adjust to your actual wheel radius

MAX_STEER_ANGLE = math.radians(45.0)  # ±45°

# Half-dimensions (used often)
L2 = WHEELBASE   / 2.0
W2 = TRACK_WIDTH / 2.0


class RoverController(Node):

    def __init__(self):
        super().__init__('rover_controller')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_targets',
            10
        )

        self.get_logger().info('RoverController started — listening on /cmd_vel')

    # ── Main callback ────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        vx  = msg.linear.x   # forward speed  (m/s)
        vy  = msg.linear.y   # lateral speed  (m/s)  — for future crab mode
        wz  = msg.angular.z  # yaw rate       (rad/s)

        angles, speeds = self.compute_wheel_commands(vx, vy, wz)

        out = Float32MultiArray()
        out.data = [float(v) for v in angles + speeds]
        self.pub.publish(out)

        self.get_logger().debug(
            f'FL={math.degrees(angles[0]):+.1f}° FR={math.degrees(angles[1]):+.1f}° '
            f'RL={math.degrees(angles[2]):+.1f}° RR={math.degrees(angles[3]):+.1f}° | '
            f'FL={speeds[0]:+.2f} FR={speeds[1]:+.2f} '
            f'RL={speeds[2]:+.2f} RR={speeds[3]:+.2f} rad/s'
        )

    # ── Geometry ─────────────────────────────────────────────────────────────

    def compute_wheel_commands(self, vx: float, vy: float, wz: float):
        """
        Returns:
            angles : [FL, FR, RL, RR]  in radians
            speeds : [FL, FR, RL, RR]  in rad/s
        """

        # ── Case 1: pure spin on the spot (vx≈0, vy≈0, wz≠0) ───────────────
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            if abs(wz) < 1e-4:
                return [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

            # Wheels point tangentially around rover center
            #   FL, RR → +45°   (one diagonal)
            #   FR, RL → -45°   (other diagonal)
            spin_angle = math.radians(45.0)

            fl_angle =  spin_angle
            fr_angle = -spin_angle
            rl_angle = -spin_angle   # mirrored from FL
            rr_angle =  spin_angle   # mirrored from FR

            # Each wheel center is at distance r from rover center
            r = math.hypot(L2, W2)
            wheel_v = abs(wz) * r          # linear speed at wheel center (m/s)
            wheel_speed = wheel_v / WHEEL_RADIUS

            # Sign: positive wz = counter-clockwise (ROS convention)
            sign = math.copysign(1.0, wz)
            speeds = [sign * wheel_speed] * 4

            angles = [fl_angle, fr_angle, rl_angle, rr_angle]
            return angles, speeds

        # ── Case 2: Ackermann / general motion ──────────────────────────────
        #
        # Instantaneous Centre of Curvature (ICC) from Twist:
        #   For pure Ackermann:  R = vx / wz
        #   General ICR location relative to rover centre:
        #     icr_x = -vy / wz   (longitudinal, if lateral speed present)
        #     icr_y =  vx / wz   (lateral)
        #
        # We use the full formula so that future crab-like commands work too.

        if abs(wz) < 1e-4:
            # Straight line — all wheels point forward, same speed
            wheel_speed = vx / WHEEL_RADIUS
            return [0.0, 0.0, 0.0, 0.0], [wheel_speed] * 4

        # ICR in rover frame (rover centre = origin, x forward, y left)
        icr_x =  vy / wz   # how far ahead/behind the ICR is
        icr_y =  vx / wz   # how far to the left the ICR is (+ = left turn)

        # Wheel positions relative to rover centre [x, y]
        wheel_pos = {
            'FL': ( L2,  W2),
            'FR': ( L2, -W2),
            'RL': (-L2,  W2),
            'RR': (-L2, -W2),
        }

        angles = []
        speeds = []

        for name, (wx, wy) in wheel_pos.items():
            dx = wx - icr_x
            dy = wy - icr_y

            # Angle the wheel must point (tangent to its circle around ICR)
            angle = math.atan2(dx, -dy)   # atan2(forward_component, right_component)

            # Distance from this wheel to ICR
            r_wheel = math.hypot(dx, dy)

            # Linear speed of this wheel centre = |wz| * r_wheel
            # Sign preserved from wz direction
            wheel_v = wz * r_wheel        # signed
            wheel_speed = wheel_v / WHEEL_RADIUS

            angles.append(angle)
            speeds.append(wheel_speed)

        fl_a, fr_a, rl_a, rr_a = angles

        # ── Enforce mirrored steering: rear mirrors front ────────────────────
        # Front angles are "ground truth"; rear gets negated
        rl_a = -fl_a
        rr_a = -fr_a

        angles = [fl_a, fr_a, rl_a, rr_a]

        # ── Clamp to mechanical limit ────────────────────────────────────────
        angles = [self._clamp(a, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
                  for a in angles]

        return angles, speeds

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))


# ── Entry point ──────────────────────────────────────────────────────────────

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


# if __name__ == '__main__':
#     main()