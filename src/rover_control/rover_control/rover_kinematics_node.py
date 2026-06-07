#!/usr/bin/env python3
"""
Rover Controller Node.

Converts cmd_vel (Twist) → wheel angles + speeds for 4-wheel swerve-drive rover.
"""

import math
from dataclasses import dataclass
import copy
from enum import Enum

from geometry_msgs.msg import Twist
from indomitus_interfaces.msg import WheelTargets
from std_srvs.srv import SetBool

import rclpy
from rclpy.node import Node


class RoverState(str, Enum):
    NORMAL = "NORMAL"
    ROTATE = "ROTATE"
    TRANSIT = "TRANSIT"


@dataclass(frozen=True)
class WheelData:
    fl: float | tuple[float, float]
    fr: float | tuple[float, float]
    rl: float | tuple[float, float]
    rr: float | tuple[float, float]

    def __iter__(self):
        return iter((self.fl, self.fr, self.rl, self.rr))

    @classmethod
    def filled(cls, value):
        return cls(value, value, value, value)


_COMPACT_OFFSETS = WheelData(
    fl=-math.pi, fr=+math.pi,
    rl=+math.pi, rr=-math.pi
)
_ZERO_OFFSETS = WheelData(
    fl=0.0, fr=0.0,
    rl=0.0, rr=0.0
)

_VXY_EPS = 1e-3   # m/s  — below this vx/vy is considered zero
_WZ_EPS  = 1e-3   # rad/s — below this wz is considered zero


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SlewRateLimiter:
    """Class to limit acceleration and deceleration rates for a single variable."""
    
    def __init__(self, max_accel: float, max_decel: float, initial_value: float = 0.0):
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.current = initial_value

    def update(self, target: float, dt: float) -> float:
        diff = target - self.current
        # Determine if we are braking or accelerating
        is_braking = (abs(target) < abs(self.current)) or (self.current * target < 0)
        rate = self.max_decel if is_braking else self.max_accel
        
        step = clamp(diff, -rate * dt, rate * dt)
        self.current += step
        return self.current


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

        self.limiters = {
            'vx': SlewRateLimiter(self.max_accel, self.max_decel),
            'vy': SlewRateLimiter(self.max_accel, self.max_decel),
            'wz': SlewRateLimiter(self.max_accel, self.max_decel),
        }

        # Estimated wheel angles (open loop — no encoders yet)
        self.current_angles = WheelData(fl=0.0, fr=0.0, rl=0.0, rr=0.0)

        # Smoothed body-frame velocities
        self.vx_smoothed = 0.0
        self.vy_smoothed = 0.0
        self.wz_smoothed = 0.0

        # Raw targets from cmd_vel
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # Speed scale (0.0–1.0, smoothed)
        self._current_scale = 0.0

        # State machine — states: 'NORMAL', 'ROTATE', 'TRANSIT'
        self._state = RoverState.NORMAL
        self._transit_dest   = RoverState.NORMAL
        self._transit_target = WheelData(fl=0.0, fr=0.0, rl=0.0, rr=0.0)
        # True while scale is still ramping down before wheels start moving
        self._transit_stopping = False
        # Last real IK speeds — held during stopping so scale ramps down smoothly
        self._last_work_speeds = WheelData(fl=0.0, fr=0.0, rl=0.0, rr=0.0)

        self.wheel_pos = WheelData(
            fl=(+self.L2, +self.W2),
            fr=(+self.L2, -self.W2),
            rl=(-self.L2, +self.W2),
            rr=(-self.L2, -self.W2)
        )

        # Compact mode — flag only, not a state
        self._compact_mode  = False
        self._offset_angles = copy.copy(_ZERO_OFFSETS)

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
        self.align_threshold = math.radians(self.get_parameter('align_threshold_deg').value)
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
        self._offset_angles = _COMPACT_OFFSETS.copy() if self._compact_mode else _ZERO_OFFSETS
        self.get_logger().info(
            f'Compact mode {"ENABLED" if self._compact_mode else "DISABLED"}')

        vx, vy, wz = self.vx_smoothed, self.vy_smoothed, self.wz_smoothed
        is_idle = (abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3)

        if is_idle:
            target = _COMPACT_OFFSETS if self._compact_mode else _ZERO_OFFSETS
        else:
            target, _ = self._ik_full(vx, vy, wz)

        self._enter_transit(target, self._state)

        response.success = True
        response.message = f'Compact mode {"enabled" if self._compact_mode else "disabled"}'
        return response

    # ──────────────────────────────────────────────────────────────────────────
    # cmd_vel
    # ──────────────────────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        self._last_cmd_vel_time = self.get_clock().now()
        vx = clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vy = clamp(msg.linear.y, -self.max_linear, self.max_linear)
        wz = clamp(msg.angular.z, -self.max_angular, self.max_angular)
        wz = self._clamp_wz(vx, vy, wz)
        self.target_vx = vx
        self.target_vy = vy
        self.target_wz = wz

    # ──────────────────────────────────────────────────────────────────────────
    # Transit helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _enter_transit(self, target_angles: WheelData, dest: RoverState):
        """
        Begin transition to dest state.
        Does NOT reset _current_scale — deceleration is smooth.
        Wheels only start moving once _transit_stopping phase ends (scale ≈ 0).
        """
        self._state            = RoverState.TRANSIT
        self._transit_dest     = dest
        self._transit_target   = target_angles
        self._transit_stopping = True   # phase 1: decelerate
        self.get_logger().info(f'TRANSIT → {dest}')

    def _transit_complete(self) -> bool:
        return all(
            abs(t_angle - c_angle) < self.align_threshold
            for t_angle, c_angle in zip(self._transit_target, self.current_angles)
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
        # Check topic timeout
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
        if elapsed > self._cmd_vel_timeout:
            self.target_vx, self.target_vy, self.target_wz = 0.0, 0.0, 0.0

        dt = 1.0 / self.control_freq

        # Step 1 — smooth velocities
        self.vx_smoothed = self.limiters['vx'].update(self.target_vx, dt)
        self.vy_smoothed = self.limiters['vy'].update(self.target_vy, dt)
        self.wz_smoothed = self.limiters['wz'].update(self.target_wz, dt)
        
        self.wz_smoothed = self._clamp_wz(
            self.vx_smoothed, self.vy_smoothed, self.wz_smoothed)

        vx = self.vx_smoothed
        vy = self.vy_smoothed
        wz = self.wz_smoothed

        if self._should_rotate(vx, vy, wz):
            desired_dest = RoverState.ROTATE
            desired_angles, _ = self._ik_rotate(wz)
        elif self._should_translate(vx, vy):
            desired_dest = RoverState.NORMAL
            desired_angles, _ = self._ik_full(vx, vy, wz)
        else:
            # Fully stopped — stay in current non-transit state
            desired_dest = self._transit_dest if self._state == RoverState.TRANSIT else self._state
            desired_angles, _ = self._ik_full(vx, vy, wz)

        if self._state != RoverState.TRANSIT:
            current_dest = self._state
            if desired_dest != current_dest:
                self._enter_transit(desired_angles, desired_dest)
        else:
            self._transit_target = desired_angles
            self._transit_dest   = desired_dest

            # Check completion
            if not self._transit_stopping and self._transit_complete():
                self._state = self._transit_dest
                self.get_logger().info(f'Aligned → {self._state}')

        # Step 3 — compute work angles and speeds
        if self._state == RoverState.NORMAL:
            work_angles, work_speeds = self._ik_full(vx, vy, wz)

        elif self._state == RoverState.ROTATE:
            work_angles, work_speeds = self._ik_rotate(wz)

        elif self._state == RoverState.TRANSIT:
            if self._transit_stopping:
                work_angles = self.current_angles
                work_speeds = self._last_work_speeds
            else:
                # Phase 2: scale already 0, steer toward target
                work_angles = self._transit_target
                work_speeds = WheelData.filled(0.0)

        if self._state != RoverState.TRANSIT:
            self._last_work_speeds = work_speeds

        # Step 4 — speed scale
        if self._state == RoverState.TRANSIT:
            if self._transit_stopping:
                self._current_scale = max(0.0,
                    self._current_scale - self.scale_down_rate * dt)
                if self._current_scale < 0.02:
                    self._current_scale    = 0.0
                    self._transit_stopping = False
                    self.get_logger().debug('TRANSIT: stopped, now aligning wheels')
            else:
                self._current_scale = 0.0
            speed_scale = self._current_scale

        else:
            # NORMAL or ROTATE — scale by worst-case angle error
            max_err = max(abs(w_a - c_a) for w_a, c_a in zip(work_angles, self.current_angles))
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
        self.current_angles = WheelData(
            fl=self._step_angle(self.current_angles.fl, work_angles.fl, dt),
            fr=self._step_angle(self.current_angles.fr, work_angles.fr, dt),
            rl=self._step_angle(self.current_angles.rl, work_angles.rl, dt),
            rr=self._step_angle(self.current_angles.rr, work_angles.rr, dt)
        )

        # Step 6 — publish
        out = WheelTargets()

        out.fl_angle = self.current_angles.fl
        out.fr_angle = self.current_angles.fr
        out.rl_angle = self.current_angles.rl
        out.rr_angle = self.current_angles.rr

        out.fl_speed = work_speeds.fl * speed_scale
        out.fr_speed = work_speeds.fr * speed_scale
        out.rl_speed = work_speeds.rl * speed_scale
        out.rr_speed = work_speeds.rr * speed_scale

        self.pub.publish(out)

    # ──────────────────────────────────────────────────────────────────────────
    # Kinematics
    # ──────────────────────────────────────────────────────────────────────────

    def _apply_compact_offset(self, angles: WheelData) -> WheelData:
        """Add compact offset to each wheel angle if compact mode is active."""
        if not self._compact_mode:
            return angles
        return WheelData(
            fl=angles.fl + self._offset_angles.fl,
            fr=angles.fr + self._offset_angles.fr,
            rl=angles.rl + self._offset_angles.rl,
            rr=angles.rr + self._offset_angles.rr
        )

    def _ik_full(self, vx: float, vy: float, wz: float) -> tuple[WheelData, WheelData]:
        """Full IK for NORMAL mode. Compact offset applied here."""
        if abs(vx) < _VXY_EPS and abs(vy) < _VXY_EPS:
            if abs(wz) < _WZ_EPS:
                angles = self._apply_compact_offset(WheelData.filled(0.0))
                return angles, WheelData.filled(0.0)
            return self._ik_rotate(wz)

        if abs(wz) < _WZ_EPS:
            speed = math.hypot(vx, vy) / self.wheel_radius
            angle = math.atan2(vy, vx)
            angle, speed = self._normalize_wheel_angle(angle, speed)
            angles = self._apply_compact_offset(WheelData.filled(angle))
            speeds = WheelData.filled(speed)
            if self._compact_mode:
                speeds = WheelData(fl=-speeds.fl, fr=-speeds.fr, rl=-speeds.rl, rr=-speeds.rr)
            return angles, speeds

        angles, speeds = self._ik_general(vx, vy, wz)
        angles = self._apply_compact_offset(angles)
        if self._compact_mode:
            speeds = WheelData(fl=-speeds.fl, fr=-speeds.fr, rl=-speeds.rl, rr=-speeds.rr)
        return angles, speeds

    def _ik_rotate(self, wz: float) -> tuple[WheelData, WheelData]:
        """IK for pure rotate-in-place. Compact offset applied here."""
        fl_a, fl_s = self._compute_wheel_ik(-wz * self.W2, +wz * self.L2)
        fr_a, fr_s = self._compute_wheel_ik(+wz * self.W2, +wz * self.L2)
        rl_a, rl_s = self._compute_wheel_ik(-wz * self.W2, -wz * self.L2)
        rr_a, rr_s = self._compute_wheel_ik(+wz * self.W2, -wz * self.L2)

        angles = WheelData(fl=fl_a, fr=fr_a, rl=rl_a, rr=rr_a)
        angles = self._apply_compact_offset(angles)

        if self._compact_mode:
            speeds = WheelData(fl=-fl_s, fr=-fr_s, rl=-rl_s, rr=-rr_s)
        else:
            speeds = WheelData(fl=fl_s, fr=fr_s, rl=rl_s, rr=rr_s)

        return angles, speeds

    def _ik_general(self, vx: float, vy: float, wz: float) -> tuple[WheelData, WheelData]:
        """Raw holonomic IK — no compact offset (applied by callers)."""
        fl_a_raw, fl_s_raw = self._compute_wheel_ik(vx - wz * self.W2, vy + wz * self.L2)
        fr_a_raw, fr_s_raw = self._compute_wheel_ik(vx + wz * self.W2, vy + wz * self.L2)
        rl_a_raw, rl_s_raw = self._compute_wheel_ik(vx - wz * self.W2, vy - wz * self.L2)
        rr_a_raw, rr_s_raw = self._compute_wheel_ik(vx + wz * self.W2, vy - wz * self.L2)

        fl_a = clamp(fl_a_raw, -self.max_steer, self.max_steer)
        fr_a = clamp(fr_a_raw, -self.max_steer, self.max_steer)
        rl_a = clamp(rl_a_raw, -self.max_steer, self.max_steer)
        rr_a = clamp(rr_a_raw, -self.max_steer, self.max_steer)

        fl_s = fl_s_raw * math.cos(fl_a_raw - fl_a)
        fr_s = fr_s_raw * math.cos(fr_a_raw - fr_a)
        rl_s = rl_s_raw * math.cos(rl_a_raw - rl_a)
        rr_s = rr_s_raw * math.cos(rr_a_raw - rr_a)

        angles = WheelData(fl=fl_a, fr=fr_a, rl=rl_a, rr=rr_a)
        speeds = WheelData(fl=fl_s, fr=fr_s, rl=rl_s, rr=rr_s)

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

    def _compute_wheel_ik(self, vx_w: float, vy_w: float) -> tuple[float, float]:
        """Compute steering angle and speed for a single wheel."""
        angle = math.atan2(vy_w, vx_w)
        speed = math.hypot(vx_w, vy_w) / self.wheel_radius
        return self._normalize_wheel_angle(angle, speed)

    def _step_angle(self, current: float, target: float, dt: float) -> float:
        """Advance steering angle toward target. No wrap — cable safe."""
        diff = target - current
        step = clamp(diff, -self.max_steer_rate * dt, self.max_steer_rate * dt)
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
