#!/usr/bin/env python3
"""
Rover Controller Node.

Converts cmd_vel (Twist) → wheel angles + speeds for 4-wheel swerve-drive rover.

Wheel layout (top view):
        front
   FL -------- FR
   |            |
   |   (center) |
   |            |
   RL -------- RR
        rear

State machine:
    NORMAL  — full operation, IK angles + traction
    ROTATE  — pure rotate-in-place, fixed tangential angles + traction
    TRANSIT — traction held at 0, wheels moving to target angles for next state

Transitions:
    NORMAL → TRANSIT → ROTATE   when |vx|<eps and |vy|<eps and |wz|>eps
    ROTATE → TRANSIT → NORMAL   when |vx|>eps or |vy|>eps

TRANSIT entry:
    - Does NOT instantly zero speed_scale.
    - speed_scale decelerates at scale_down_rate until it reaches 0.
    - Only then wheels begin rotating to new target angles.
    - When all wheels aligned → enter destination state, scale ramps back up.

Compact mode:
    NOT a state. A flag that adds ±180° offset to all IK output angles.
    NORMAL / ROTATE / TRANSIT all work identically in compact mode.
    Toggling compact triggers a TRANSIT to realign wheels with new offsets.
    Drive works normally once realigned.

Nav2 compatibility:
    Nav2 publishes standard cmd_vel (vx, vy, wz).
    No Nav2 configuration needed beyond motion_model: Omni.
"""

import math

from geometry_msgs.msg import Twist
from indomitus_interfaces.msg import WheelTargets
from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node


_COMPACT_OFFSETS = {
    'FL': -math.pi,
    'FR': +math.pi,
    'RL': +math.pi,
    'RR': -math.pi,
}
_ZERO_OFFSETS = {name: 0.0 for name in _COMPACT_OFFSETS}

_VXY_EPS = 1e-3   # m/s  — below this vx/vy is considered zero
_WZ_EPS  = 1e-3   # rad/s — below this wz is considered zero


class RoverController(Node):

    def __init__(self):
        super().__init__('rover_controller')

        # --- Physical parameters ---
        self.declare_parameter('wheelbase',    1.20)
        self.declare_parameter('track_width',  0.80)
        self.declare_parameter('wheel_radius', 0.15)

        # --- Steering limits (placeholders, real values from yaml) ---
        self.declare_parameter('max_steer_deg',      10.0)
        self.declare_parameter('max_steer_rate_deg', 10.0)

        # --- Velocity limits ---
        self.declare_parameter('max_linear_speed',  0.10)
        self.declare_parameter('max_angular_speed', 0.10)
        self.declare_parameter('max_accel',         0.10)
        self.declare_parameter('max_decel',         0.30)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('cmd_vel_timeout_s', 2.0)

        # --- Transition parameters ---
        self.declare_parameter('align_threshold_deg', 5.0)
        self.declare_parameter('scale_up_rate',   0.5)
        self.declare_parameter('scale_down_rate', 1.0)

        self._read_params()

        # Estimated wheel angles (open loop — no encoders yet)
        self.current_angles = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}

        # Smoothed body-frame velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # Raw targets from cmd_vel
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # Speed scale (0.0–1.0, smoothed)
        self._current_scale = 0.0

        # State machine — states: 'NORMAL', 'ROTATE', 'TRANSIT'
        self._state = 'NORMAL'
        self._transit_dest   = 'NORMAL'
        self._transit_target = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        # True while scale is still ramping down before wheels start moving
        self._transit_stopping = False
        # Last real IK speeds — held during stopping so scale ramps down smoothly
        self._last_work_speeds = [0.0, 0.0, 0.0, 0.0]

        self.wheel_names = ['FL', 'FR', 'RL', 'RR']
        self.wheel_pos = {
            'FL': (+self.L2, +self.W2),
            'FR': (+self.L2, -self.W2),
            'RL': (-self.L2, +self.W2),
            'RR': (-self.L2, -self.W2),
        }

        # Compact mode — flag only, not a state
        self._compact_mode  = False
        self._offset_angles = _ZERO_OFFSETS.copy()

        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(
            WheelTargets, 'wheel_targets', 10)
        self._compact_srv = self.create_service(
            SetBool, 'set_compact_mode', self._on_set_compact_mode)

        dt = 1.0 / self.control_freq
        self.timer = self.create_timer(dt, self.control_loop)

        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_timeout   = self.get_parameter('cmd_vel_timeout_s').value

        self.get_logger().info('RoverController started')

    # ──────────────────────────────────────────────────────────────────────────
    # Init
    # ──────────────────────────────────────────────────────────────────────────

    def _read_params(self):
        self.wheelbase       = self.get_parameter('wheelbase').value
        self.track_width     = self.get_parameter('track_width').value
        self.wheel_radius    = self.get_parameter('wheel_radius').value
        self.max_steer       = math.radians(self.get_parameter('max_steer_deg').value)
        self.max_steer_rate  = math.radians(self.get_parameter('max_steer_rate_deg').value)
        self.max_linear      = self.get_parameter('max_linear_speed').value
        self.max_angular     = self.get_parameter('max_angular_speed').value
        self.max_accel       = self.get_parameter('max_accel').value
        self.max_decel       = self.get_parameter('max_decel').value
        self.control_freq    = self.get_parameter('control_frequency').value
        self.align_threshold = math.radians(
            self.get_parameter('align_threshold_deg').value)
        self.scale_up_rate   = self.get_parameter('scale_up_rate').value
        self.scale_down_rate = self.get_parameter('scale_down_rate').value
        self.L2 = self.wheelbase / 2.0
        self.W2 = self.track_width / 2.0

    # ──────────────────────────────────────────────────────────────────────────
    # Compact mode service
    # ──────────────────────────────────────────────────────────────────────────

    def _on_set_compact_mode(self,
                             request: SetBool.Request,
                             response: SetBool.Response):
        if request.data == self._compact_mode:
            response.success = True
            response.message = 'Already in requested mode'
            return response

        self._compact_mode  = request.data
        self._offset_angles = (
            _COMPACT_OFFSETS.copy() if self._compact_mode else _ZERO_OFFSETS.copy()
        )
        self.get_logger().info(
            'Compact mode ' + ('ENABLED' if self._compact_mode else 'DISABLED'))

        # Compute transit target: current IK angles WITH new compact offset.
        # _ik_full already applies self._offset_angles, which was just updated,
        # so calling it now gives the correct post-toggle angles.
        # Special case: if robot is fully stopped (idle), _ik_full returns zeros
        # and we must explicitly use the offset as the target instead.
        vx, vy, wz = self.current_vx, self.current_vy, self.current_wz
        is_idle = (abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3)

        if is_idle:
            # Target is simply the offset position for each wheel
            if self._compact_mode:
                # Compact on: go to offset angles (current_angle + offset delta)
                # Use the raw offset value as absolute target since at idle
                # wheels should be at exactly the offset position
                target = {
                    name: _COMPACT_OFFSETS[name]
                    for name in self.wheel_names
                }
            else:
                # Compact off: return to 0° (straight)
                target = {name: 0.0 for name in self.wheel_names}
        else:
            ik_angles, _ = self._ik_full(vx, vy, wz)
            target = {name: ik_angles[i] for i, name in enumerate(self.wheel_names)}

        self._enter_transit(target, self._state)

        response.success = True
        response.message = 'Compact mode ' + (
            'enabled' if self._compact_mode else 'disabled')
        return response

    # ──────────────────────────────────────────────────────────────────────────
    # cmd_vel
    # ──────────────────────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        self._last_cmd_vel_time = self.get_clock().now()
        vx = self._clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vy = self._clamp(msg.linear.y, -self.max_linear, self.max_linear)
        wz = self._clamp(msg.angular.z, -self.max_angular, self.max_angular)
        wz = self._clamp_wz(vx, vy, wz)
        self.target_vx = vx
        self.target_vy = vy
        self.target_wz = wz

    # ──────────────────────────────────────────────────────────────────────────
    # Transit helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _enter_transit(self, target_angles: dict, dest: str):
        """
        Begin transition to dest state.
        Does NOT reset _current_scale — deceleration is smooth.
        Wheels only start moving once _transit_stopping phase ends (scale ≈ 0).
        """
        self._state            = 'TRANSIT'
        self._transit_dest     = dest
        self._transit_target   = dict(target_angles)
        self._transit_stopping = True   # phase 1: decelerate
        self.get_logger().info(f'TRANSIT → {dest}')

    def _transit_complete(self) -> bool:
        return all(
            abs(self._transit_target[name] - self.current_angles[name])
            < self.align_threshold
            for name in self.wheel_names
        )

    def _should_rotate(self, vx, vy, wz) -> bool:
        return (
            abs(vx) < _VXY_EPS and
            abs(vy) < _VXY_EPS and
            abs(wz) > _WZ_EPS
        )

    def _should_translate(self, vx, vy) -> bool:
        return abs(vx) > _VXY_EPS or abs(vy) > _VXY_EPS

    # ──────────────────────────────────────────────────────────────────────────
    # Main control loop
    # ──────────────────────────────────────────────────────────────────────────

    def control_loop(self):
        elapsed = (
            self.get_clock().now() - self._last_cmd_vel_time
        ).nanoseconds * 1e-9
        if elapsed > self._cmd_vel_timeout:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        dt = 1.0 / self.control_freq

        # Step 1 — smooth velocities
        self.current_vx = self._rate_limit(
            self.current_vx, self.target_vx, self.max_accel, self.max_decel, dt)
        self.current_vy = self._rate_limit(
            self.current_vy, self.target_vy, self.max_accel, self.max_decel, dt)
        self.current_wz = self._rate_limit(
            self.current_wz, self.target_wz, self.max_accel, self.max_decel, dt)
        self.current_wz = self._clamp_wz(
            self.current_vx, self.current_vy, self.current_wz)

        vx = self.current_vx
        vy = self.current_vy
        wz = self.current_wz

        # Step 2 — determine desired destination state from current cmd_vel
        # This runs every tick regardless of current state so that a changing
        # cmd_vel always updates the transit target, even mid-transition.
        if self._should_rotate(vx, vy, wz):
            desired_dest = 'ROTATE'
            desired_angles, _ = self._ik_rotate(wz)
        elif self._should_translate(vx, vy):
            desired_dest = 'NORMAL'
            desired_angles, _ = self._ik_full(vx, vy, wz)
        else:
            # Fully stopped — stay in current non-transit state
            desired_dest = self._transit_dest if self._state == 'TRANSIT' else self._state
            desired_angles_list, _ = self._ik_full(vx, vy, wz)
            desired_angles = desired_angles_list

        desired_target = {name: desired_angles[i]
                          for i, name in enumerate(self.wheel_names)}

        # Trigger or update transit whenever the destination or target changes
        # significantly vs what we're currently tracking.
        if self._state != 'TRANSIT':
            # Only enter transit on a real mode change
            current_dest = self._state
            if desired_dest != current_dest:
                self._enter_transit(desired_target, desired_dest)
        else:
            # Already in transit — update target to latest IK so that a
            # mid-transit cmd_vel change is reflected immediately.
            # Only update during stopping phase (wheels not yet moving);
            # once aligning, keep target stable so wheels don't chase a
            # moving goal.
            self._transit_target = desired_target
            self._transit_dest   = desired_dest

            # Check completion
            if not self._transit_stopping and self._transit_complete():
                self._state = self._transit_dest
                self.get_logger().info(f'Aligned → {self._state}')

        # Step 3 — compute work angles and speeds
        if self._state == 'NORMAL':
            work_angles, work_speeds = self._ik_full(vx, vy, wz)

        elif self._state == 'ROTATE':
            work_angles, work_speeds = self._ik_rotate(wz)

        else:  # TRANSIT
            if self._transit_stopping:
                # Phase 1: keep current angles, let scale ramp down smoothly.
                # Use last real IK speeds — speed_scale * speeds gives smooth
                # decel, NOT instant zero.
                work_angles = [self.current_angles[n] for n in self.wheel_names]
                work_speeds = self._last_work_speeds
            else:
                # Phase 2: scale already 0, steer toward target
                work_angles = [self._transit_target[n] for n in self.wheel_names]
                work_speeds = [0.0] * 4

        # Save work_speeds for use in next tick's stopping phase
        if self._state != 'TRANSIT':
            self._last_work_speeds = list(work_speeds)

        # Step 4 — speed scale
        if self._state == 'TRANSIT':
            if self._transit_stopping:
                # Smooth deceleration — do not hard-zero
                self._current_scale = max(
                    0.0,
                    self._current_scale - self.scale_down_rate * dt)
                # Once slow enough, switch to steering phase
                if self._current_scale < 0.02:
                    self._current_scale    = 0.0
                    self._transit_stopping = False
                    self.get_logger().debug('TRANSIT: stopped, now aligning wheels')
            else:
                self._current_scale = 0.0
            speed_scale = self._current_scale

        else:
            # NORMAL or ROTATE — scale by worst-case angle error
            max_err = max(
                abs(work_angles[i] - self.current_angles[name])
                for i, name in enumerate(self.wheel_names)
            )
            raw_scale = max(0.0, math.cos(max_err))
            if max_err > math.radians(20):
                raw_scale = min(raw_scale, 0.3)

            if raw_scale < self._current_scale:
                self._current_scale = max(
                    raw_scale,
                    self._current_scale - self.scale_down_rate * dt)
            else:
                self._current_scale = min(
                    raw_scale,
                    self._current_scale + self.scale_up_rate * dt)

            speed_scale = self._current_scale

        # Step 5 — advance wheel angles
        for name, t_angle in zip(self.wheel_names, work_angles):
            self.current_angles[name] = self._step_angle(
                self.current_angles[name], t_angle, dt)

        # Step 6 — publish
        out = WheelTargets()
        result_angles = [self.current_angles[n] for n in self.wheel_names]
        result_speeds = [s * speed_scale for s in work_speeds]

        out.fl_angle, out.fr_angle, out.rl_angle, out.rr_angle = result_angles
        out.fl_speed, out.fr_speed, out.rl_speed, out.rr_speed = result_speeds
        self.pub.publish(out)

        self.get_logger().debug(
            f'{self._state}{"(stop)" if self._transit_stopping else ""} | '
            f'compact={self._compact_mode} | '
            f'scale={speed_scale:.2f} | '
            f'angles=[{",".join(f"{math.degrees(self.current_angles[n]):+.1f}" for n in self.wheel_names)}]'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Kinematics
    # ──────────────────────────────────────────────────────────────────────────

    def _apply_compact_offset(self, angles: list) -> list:
        """Add compact offset to each wheel angle if compact mode is active."""
        if not self._compact_mode:
            return angles
        return [
            angles[i] + self._offset_angles[name]
            for i, name in enumerate(self.wheel_names)
        ]

    def _ik_full(self, vx: float, vy: float, wz: float):
        """Full IK for NORMAL mode. Compact offset applied here."""
        if abs(vx) < _VXY_EPS and abs(vy) < _VXY_EPS:
            if abs(wz) < _WZ_EPS:
                angles = self._apply_compact_offset([0.0] * 4)
                return angles, [0.0] * 4
            return self._ik_rotate(wz)

        if abs(wz) < _WZ_EPS:
            speed = math.hypot(vx, vy) / self.wheel_radius
            angle = math.atan2(vy, vx)
            angle, speed = self._normalize_wheel_angle(angle, speed)
            angles = self._apply_compact_offset([angle] * 4)
            speeds = [speed] * 4
            if self._compact_mode:
                speeds = [-s for s in speeds]
            return angles, speeds

        angles, speeds = self._ik_general(vx, vy, wz)
        angles = self._apply_compact_offset(angles)
        if self._compact_mode:
            speeds = [-s for s in speeds]
        return angles, speeds

    def _ik_rotate(self, wz: float):
        """IK for pure rotate-in-place. Compact offset applied here."""
        angles, speeds = [], []
        for _, (wx, wy) in self.wheel_pos.items():
            vx_w = -wz * wy
            vy_w = +wz * wx
            angle = math.atan2(vy_w, vx_w)
            speed = math.hypot(vx_w, vy_w) / self.wheel_radius
            angle, speed = self._normalize_wheel_angle(angle, speed)
            angles.append(angle)
            speeds.append(speed)
        angles = self._apply_compact_offset(angles)
        if self._compact_mode:
            speeds = [-s for s in speeds]
        return angles, speeds

    def _ik_general(self, vx: float, vy: float, wz: float):
        """Raw holonomic IK — no compact offset (applied by callers)."""
        angles, speeds = [], []
        for _, (wx, wy) in self.wheel_pos.items():
            vx_w = vx - wz * wy
            vy_w = vy + wz * wx
            angle = math.atan2(vy_w, vx_w)
            speed = math.hypot(vx_w, vy_w) / self.wheel_radius
            angle, speed = self._normalize_wheel_angle(angle, speed)
            angles.append(angle)
            speeds.append(speed)
        angles = [self._clamp(a, -self.max_steer, self.max_steer) for a in angles]
        return angles, speeds

    def _clamp_wz(self, vx: float, vy: float, wz: float) -> float:
        if abs(wz) < 1e-6:
            return wz
        sign   = math.copysign(1.0, wz)
        margin = 1.1
        max_wz = self.max_angular
        if abs(vy) > _VXY_EPS:
            max_wz = min(max_wz, abs(vy) / (self.L2 * margin))
        if abs(vx) > _VXY_EPS:
            max_wz = min(max_wz, abs(vx) / (self.W2 * margin))
        return sign * min(abs(wz), max_wz)

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _rate_limit(self, current, target, max_accel, max_decel, dt):
        diff = target - current
        rate = max_decel if (
            abs(target) < abs(current) or current * target < 0
        ) else max_accel
        return current + self._clamp(diff, -rate * dt, rate * dt)

    def _step_angle(self, current: float, target: float, dt: float) -> float:
        """Advance steering angle toward target. No wrap — cable safe."""
        diff = target - current
        step = self._clamp(diff, -self.max_steer_rate * dt, self.max_steer_rate * dt)
        return current + step

    def _normalize_wheel_angle(self, angle: float, speed: float):
        """Keep steering in (-π/2, π/2); flip and reverse drive if outside."""
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