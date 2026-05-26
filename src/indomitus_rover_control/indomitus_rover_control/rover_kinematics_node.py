#!/usr/bin/env python3
"""
Rover Controller Node.

Converts cmd_vel (Twist) → wheel angles + speeds for 4-wheel swerve-drive rover.

Steering geometry:
  - Each wheel steers independently (true swerve / holonomic drive)
  - Angles are kept in (-π/2, π/2); if target exceeds that range the wheel
    flips 180° and the drive direction is reversed instead

Wheel layout (top view):
        front
   FL -------- FR
   |            |
   |   (center) |
   |            |
   RL -------- RR
        rear

Control architecture:
  cmd_vel_callback  →  stores target_vx / target_vy / target_wz
                               ↓
  control_loop (timer, default 20 Hz)
      1. Rate-limit velocities         (acc_lim smoothing)
      2. Compute target angles/speeds  (kinematics)
      3. Step current angles → target  (steer rate limiter)
      4. Global speed scaling          (cos of worst-case angle error)
      5. Publish WheelTargets

Speed scaling uses the MAXIMUM angle error across all four wheels so every
wheel is slowed by the same factor — preserving kinematics and avoiding
uncoordinated deceleration / wheel slip.

Published message (indomitus_interfaces/WheelTargets):
  fl_angle, fr_angle, rl_angle, rr_angle  — radians
  fl_speed, fr_speed, rl_speed, rr_speed  — rad/s
"""

import math

from geometry_msgs.msg import Twist
from indomitus_interfaces.msg import WheelTargets
from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node


_COMPACT_OFFSETS = {'FL': -math.pi, 'FR': +math.pi,
                   'RL': +math.pi, 'RR': -math.pi}
_ZERO_OFFSETS  = {name: 0.0 for name in _COMPACT_OFFSETS}

class RoverController(Node):

    def __init__(self):
        super().__init__('rover_controller')

        # --- Physical parameters (rover_description/config/rover_geometry.yaml) ---
        self.declare_parameter('wheelbase',   1.20)   # m  — front-to-rear axle distance
        self.declare_parameter('track_width', 0.80)   # m  — left-to-right wheel distance
        self.declare_parameter('wheel_radius', 0.15)  # m

        # --- Steering limits (rover_description/config/rover_motors.yaml) ---
        self.declare_parameter('max_steer_deg',      10.0)   # max steering angle
        self.declare_parameter('max_steer_rate_deg', 10.0)   # deg/s — physical steer motor speed

        # --- Velocity limits (rover_bringup/config/rover_controller.yaml) ---
        self.declare_parameter('max_linear_speed',  0.10)   # m/s
        self.declare_parameter('max_angular_speed', 0.1)   # rad/s
        self.declare_parameter('max_accel',         0.1)   # m/s²  — linear acceleration limit
        self.declare_parameter('max_decel',  0.3)
        self.declare_parameter('control_frequency', 20.0)  # Hz
        self.declare_parameter('cmd_vel_timeout_s', 0.5)   # s

        self._read_params()

        self.current_angles = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # Raw targets from cmd_vel
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        self._current_scale = 0.0

        self.wheel_names = ['FL', 'FR', 'RL', 'RR']
        self.wheel_pos = {
            'FL': ( self.L2,  self.W2),
            'FR': ( self.L2, -self.W2),
            'RL': (-self.L2,  self.W2),
            'RR': (-self.L2, -self.W2),
        }

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(WheelTargets, 'wheel_targets', 10)

        # Compact mode state
        self._compact_mode = False
        self._offset_angles = _ZERO_OFFSETS.copy()

        self._compact_srv = self.create_service(
            SetBool, 'set_compact_mode', self._on_set_compact_mode)

        dt = 1.0 / self.control_freq
        self.timer = self.create_timer(dt, self.control_loop)

        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_s').value

        self.get_logger().info('RoverController started — listening on cmd_vel')

    # ──────────────────────────────────────────────────────────────────────────
    # Initialisation helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _read_params(self):
        self.wheelbase      = self.get_parameter('wheelbase').value
        self.track_width    = self.get_parameter('track_width').value
        self.wheel_radius   = self.get_parameter('wheel_radius').value
        self.max_steer      = math.radians(self.get_parameter('max_steer_deg').value)
        self.max_steer_rate = math.radians(self.get_parameter('max_steer_rate_deg').value)
        self.max_linear     = self.get_parameter('max_linear_speed').value
        self.max_angular    = self.get_parameter('max_angular_speed').value
        self.max_accel      = self.get_parameter('max_accel').value
        self.max_decel      = self.get_parameter('max_decel').value
        
        self.control_freq   = self.get_parameter('control_frequency').value

        self.L2 = self.wheelbase / 2.0
        self.W2 = self.track_width / 2.0

    # ──────────────────────────────────────────────────────────────────────────
    # Compact mode service
    # ──────────────────────────────────────────────────────────────────────────

    def _on_set_compact_mode(self, request: SetBool.Request, response: SetBool.Response):
        if request.data == self._compact_mode:
            response.success = True
            response.message = 'Already in requested mode'
            return response
        self._compact_mode = request.data
        if self._compact_mode:
            self.get_logger().info('Compact mode ENABLED')
            self._offset_angles = _COMPACT_OFFSETS.copy()
        else:
            self.get_logger().info('Compact mode DISABLED')
            self._offset_angles = _ZERO_OFFSETS.copy()
        
        response.success = True
        response.message = 'Compact mode ' + ('enabled' if self._compact_mode else 'disabled')
        return response

    # ──────────────────────────────────────────────────────────────────────────
    # cmd_vel subscriber — only stores the clamped target; no kinematics here
    # ──────────────────────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        self._last_cmd_vel_time = self.get_clock().now()

        self.target_vx = self._clamp(msg.linear.x,  -self.max_linear,  self.max_linear)
        self.target_vy = self._clamp(msg.linear.y,  -self.max_linear,  self.max_linear)
        self.target_wz = self._clamp(msg.angular.z, -self.max_angular, self.max_angular)

    # ──────────────────────────────────────────────────────────────────────────
    # Main control loop (runs at control_frequency Hz)
    # ──────────────────────────────────────────────────────────────────────────

    def control_loop(self):
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
        if elapsed > self._cmd_vel_timeout:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        dt = 1.0 / self.control_freq

        # Step 1 — smooth velocities with a linear rate limiter (acceleration cap)
        self.current_vx = self._rate_limit(self.current_vx, self.target_vx, self.max_accel, self.max_decel, dt)
        self.current_vy = self._rate_limit(self.current_vy, self.target_vy, self.max_accel, self.max_decel, dt)
        self.current_wz = self._rate_limit(self.current_wz, self.target_wz, self.max_accel, self.max_decel, dt)

        # Step 2 — kinematics: desired wheel angles and drive speeds
        target_angles, target_speeds = self.compute_wheel_commands(
            self.current_vx, self.current_vy, self.current_wz)

        if self._compact_mode:
            target_angles = [
                target_angles[i] + self._offset_angles[name]
                for i, name in enumerate(self.wheel_names)
            ]
            target_speeds = [-s for s in target_speeds]

        # Step 3 — advance each wheel angle toward its target at max_steer_rate
        for name, t_angle in zip(self.wheel_names, target_angles):
            self.current_angles[name] = self._step_angle(
                self.current_angles[name], t_angle, dt
            )

        # Step 4 — global speed scale: determined by the wheel with the largest
        #           angle error so all wheels slow down by the same factor,
        #           preserving kinematics and preventing wheel slip.
        max_angle_error = max(
            abs(target_angles[i] - self.current_angles[name])
            for i, name in enumerate(self.wheel_names)
        )

        raw_scale = max(0.0, math.cos(max_angle_error))

        # Hard cap: if any wheel is more than 20° off, limit to 30 % speed
        if max_angle_error > math.radians(20):
            speed_scale = min(speed_scale, 0.2)

        scale_up_rate   = 0.5 
        scale_down_rate = 5.0
        dt = 1.0 / self.control_freq

        if raw_scale < self._current_scale:
            self._current_scale = max(raw_scale, self._current_scale - scale_down_rate * dt)
        else:
            self._current_scale = min(raw_scale, self._current_scale + scale_up_rate * dt)

        speed_scale = self._current_scale

        # Step 5 — publish
        out = WheelTargets()
        result_angles = [self.current_angles[n] for n in self.wheel_names]
        result_speeds = [s * speed_scale for s in target_speeds]

        out.fl_angle, out.fr_angle, out.rl_angle, out.rr_angle = result_angles
        out.fl_speed, out.fr_speed, out.rl_speed, out.rr_speed = result_speeds
        self.pub.publish(out)

        self.get_logger().debug(
            # f'FL={math.degrees(result_angles[0]):+.1f}° '
            # f'FR={math.degrees(result_angles[1]):+.1f}° '
            # f'RL={math.degrees(result_angles[2]):+.1f}° '
            # f'RR={math.degrees(result_angles[3]):+.1f}° | '
            # f'speeds: {[f"{s:+.2f}" for s in result_speeds]} rad/s | '
            f'scale={speed_scale:.2f}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Kinematics
    # ──────────────────────────────────────────────────────────────────────────

    def compute_wheel_commands(self, vx: float, vy: float, wz: float):
        """
        Compute target wheel angles and drive speeds from body-frame velocity.

        Returns:
            angles : [FL, FR, RL, RR]  in radians, clamped to ±max_steer
            speeds : [FL, FR, RL, RR]  in rad/s
        """
        # Case 1: pure rotation on the spot (vx ≈ 0, vy ≈ 0)
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            if abs(wz) < 1e-4:
                return [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

            # Wheels point tangentially around the chassis centre
            angle = math.atan2(self.L2, self.W2)
            r = math.hypot(self.L2, self.W2)
            wheel_speed = abs(wz) * r / self.wheel_radius
            sign = math.copysign(1.0, wz)

            return (
                [-angle,  angle,  angle, -angle],
                [-wheel_speed * sign,  wheel_speed * sign,
                 -wheel_speed * sign,  wheel_speed * sign],
            )

        # Case 2: straight-line motion (no rotation)
        if abs(wz) < 1e-4:
            speed = math.hypot(vx, vy) / self.wheel_radius
            angle = math.atan2(vy, vx)
            angle, speed = self._normalize_wheel_angle(angle, speed)
            return [angle] * 4, [speed] * 4

        # Case 3: general motion (Ackermann / holonomic)
        angles = []
        speeds = []

        for _, (wx, wy) in self.wheel_pos.items():
            # Velocity of this wheel's contact point in body frame
            vx_w = vx - wz * wy
            vy_w = vy + wz * wx

            angle = math.atan2(vy_w, vx_w)
            speed = math.hypot(vx_w, vy_w) / self.wheel_radius
            angle, speed = self._normalize_wheel_angle(angle, speed)

            angles.append(angle)
            speeds.append(speed)

        angles = [self._clamp(a, -self.max_steer, self.max_steer) for a in angles]
        return angles, speeds

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _rate_limit(self, current, target, max_accel, max_decel, dt):
        """Linear rate limiter — limits how fast a value can change per second."""
        diff = target - current
        if abs(target) < abs(current) or (current * target < 0):
            rate = max_decel
        else:
            rate = max_accel
        step = self._clamp(diff, -rate * dt, rate * dt)
        return current + step

    def _step_angle(self, current: float, target: float, dt: float) -> float:
        """
        Advance a steering angle toward target at max_steer_rate (rad/s).
        Difference is normalised to (-π, π) to take the shortest path.
        """
        diff = target - current
        # diff = (diff + math.pi) % (2 * math.pi) - math.pi  # shortest-path wrap
        step = self._clamp(diff, -self.max_steer_rate * dt, self.max_steer_rate * dt)

        # Steering clamp is intentionally omitted here.
        # Each wheel has asymmetric limits ([-π/2, 3π/2] or [-3π/2, π/2]
        # depending on side) which are enforced by the chassis driver.
        # Adding a symmetric clamp here would incorrectly restrict valid angles.
        # return self._clamp(current + step, -self.max_steer, self.max_steer)
        return current + step

    def _normalize_wheel_angle(self, angle: float, speed: float):
        """
        Keep steering angle in (-π/2, π/2).
        If the target is in the rear half-plane, flip by 180° and reverse drive.
        """
        if angle > math.pi / 2:
            angle -= math.pi
            speed = -speed
        elif angle < -math.pi / 2:
            angle += math.pi
            speed = -speed
        return angle, speed

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
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
        if rclpy.ok():
            rclpy.shutdown()