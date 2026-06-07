#!/usr/bin/env python3
"""
Rover Controller Node.

Converts cmd_vel (Twist) → wheel angles + speeds for 4-wheel swerve-drive rover.
"""

import math
from dataclasses import dataclass
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
    fl: float
    fr: float
    rl: float
    rr: float

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


class SwerveKinematics:
    """Kinematics for 4-wheel swerve drive, with support for compact mode offsets."""
    def __init__(self, wheelbase: float, track_width: float, wheel_radius: float, max_steer: float):
        self.wheel_radius = wheel_radius
        self.max_steer = max_steer
        self.L2 = wheelbase / 2.0
        self.W2 = track_width / 2.0

        self.compact_mode = False
        self._offset_angles = _ZERO_OFFSETS

    def set_compact_mode(self, enabled: bool):
        self.compact_mode = enabled
        self._offset_angles = _COMPACT_OFFSETS if enabled else _ZERO_OFFSETS

    def get_offset_angles(self) -> WheelData:
        return self._offset_angles

    def ik_full(self, vx: float, vy: float, wz: float) -> tuple[WheelData, WheelData]:
        """Full IK for NORMAL mode. Compact offset applied here."""
        if abs(vx) < _VXY_EPS and abs(vy) < _VXY_EPS:
            if abs(wz) < _WZ_EPS:
                angles = self._apply_compact_offset(WheelData.filled(0.0))
                return angles, WheelData.filled(0.0)
            return self.ik_rotate(wz)

        if abs(wz) < _WZ_EPS:
            speed = math.hypot(vx, vy) / self.wheel_radius
            angle = math.atan2(vy, vx)
            angle, speed = self._normalize_wheel_angle(angle, speed)
            angles = self._apply_compact_offset(WheelData.filled(angle))
            speeds = WheelData.filled(speed)
            if self.compact_mode:
                speeds = WheelData(fl=-speeds.fl, fr=-speeds.fr, rl=-speeds.rl, rr=-speeds.rr)
            return angles, speeds

        angles, speeds = self._ik_general(vx, vy, wz)
        angles = self._apply_compact_offset(angles)
        if self.compact_mode:
            speeds = WheelData(fl=-speeds.fl, fr=-speeds.fr, rl=-speeds.rl, rr=-speeds.rr)
        return angles, speeds

    def ik_rotate(self, wz: float) -> tuple[WheelData, WheelData]:
        """IK for pure rotate-in-place. Compact offset applied here."""
        fl_a, fl_s = self._compute_wheel_ik(-wz * self.W2, +wz * self.L2)
        fr_a, fr_s = self._compute_wheel_ik(+wz * self.W2, +wz * self.L2)
        rl_a, rl_s = self._compute_wheel_ik(-wz * self.W2, -wz * self.L2)
        rr_a, rr_s = self._compute_wheel_ik(+wz * self.W2, -wz * self.L2)

        angles = WheelData(fl=fl_a, fr=fr_a, rl=rl_a, rr=rr_a)
        angles = self._apply_compact_offset(angles)

        if self.compact_mode:
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

    def _apply_compact_offset(self, angles: WheelData) -> WheelData:
        """Apply compact mode offset to angles."""
        if not self.compact_mode:
            return angles
        return WheelData(
            fl=angles.fl + self._offset_angles.fl,
            fr=angles.fr + self._offset_angles.fr,
            rl=angles.rl + self._offset_angles.rl,
            rr=angles.rr + self._offset_angles.rr
        )

    def _compute_wheel_ik(self, vx_w: float, vy_w: float) -> tuple[float, float]:
        """Compute steering angle and speed for a single wheel."""
        angle = math.atan2(vy_w, vx_w)
        speed = math.hypot(vx_w, vy_w) / self.wheel_radius
        return self._normalize_wheel_angle(angle, speed)

    def _normalize_wheel_angle(self, angle: float, speed: float):
        if angle > math.pi / 2:
            angle -= math.pi
            speed = -speed
        elif angle < -math.pi / 2:
            angle += math.pi
            speed = -speed
        return angle, speed


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


class RoverStateMachine:
    """State machine with NORMAL, ROTATE, and TRANSIT states."""
    def __init__(self, align_threshold: float, scale_up_rate: float, scale_down_rate: float):
        self.align_threshold = align_threshold
        self.scale_up_rate = scale_up_rate
        self.scale_down_rate = scale_down_rate

        self.state = RoverState.NORMAL
        self.transit_dest = RoverState.NORMAL
        self.transit_target = WheelData.filled(0.0)
        
        self.transit_stopping = False
        self.current_scale = 0.0
        self.last_work_speeds = WheelData.filled(0.0)

    def enter_transit(self, target_angles: WheelData, dest: RoverState, logger=None):
        """Enter transit state."""
        self.state = RoverState.TRANSIT
        self.transit_dest = dest
        self.transit_target = target_angles
        self.transit_stopping = True
        if logger:
            logger.info(f'TRANSIT → {dest}')

    def update_transitions(self, vx: float, vy: float, wz: float, desired_angles: WheelData, current_angles: WheelData, logger=None):
        """Analyzes incoming velocities and controls transitions between states."""
        if self._should_rotate(vx, vy, wz):
            desired_dest = RoverState.ROTATE
        elif self._should_translate(vx, vy):
            desired_dest = RoverState.NORMAL
        else:
            desired_dest = self.transit_dest if self.state == RoverState.TRANSIT else self.state

        if self.state != RoverState.TRANSIT:
            if desired_dest != self.state:
                self.enter_transit(desired_angles, desired_dest, logger)
        else:
            self.transit_target = desired_angles
            self.transit_dest = desired_dest

            if not self.transit_stopping and self._transit_complete(current_angles):
                self.state = self.transit_dest
                if logger:
                    logger.info(f'Aligned → {self.state}')

    def update_scale(self, dt: float, work_angles: WheelData, current_angles: WheelData, logger=None) -> float:
        """Calculates the smoothed speed scale based on the current state and wheel angle errors."""
        if self.state == RoverState.TRANSIT:
            if self.transit_stopping:
                self.current_scale = max(0.0, self.current_scale - self.scale_down_rate * dt)
                if self.current_scale < 0.02:
                    self.current_scale = 0.0
                    self.transit_stopping = False
                    if logger:
                        logger.debug('TRANSIT: stopped, now aligning wheels')
            else:
                self.current_scale = 0.0
        else:
            max_err = max(abs(w_a - c_a) for w_a, c_a in zip(work_angles, current_angles))
            raw_scale = max(0.0, math.cos(max_err))
            if max_err > math.radians(20):
                raw_scale = min(raw_scale, 0.3)

            if raw_scale < self.current_scale:
                self.current_scale = max(raw_scale, self.current_scale - self.scale_down_rate * dt)
            else:
                self.current_scale = min(raw_scale, self.current_scale + self.scale_up_rate * dt)

        return self.current_scale

    def _transit_complete(self, current_angles: WheelData) -> bool:
        return all(
            abs(t_angle - c_angle) < self.align_threshold
            for t_angle, c_angle in zip(self.transit_target, current_angles)
        )

    def _should_rotate(self, vx: float, vy: float, wz: float) -> bool:
        return abs(vx) < _VXY_EPS and abs(vy) < _VXY_EPS and abs(wz) > _WZ_EPS

    def _should_translate(self, vx: float, vy: float) -> bool:
        return abs(vx) > _VXY_EPS or abs(vy) > _VXY_EPS


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

        self.kinematics = SwerveKinematics(
            wheelbase=self.wheelbase,
            track_width=self.track_width,
            wheel_radius=self.wheel_radius,
            max_steer=self.max_steer
        )

        self.state_machine = RoverStateMachine(
            align_threshold=self.align_threshold,
            scale_up_rate=self.scale_up_rate,
            scale_down_rate=self.scale_down_rate
        )

        self.current_angles = WheelData(fl=0.0, fr=0.0, rl=0.0, rr=0.0)

        self.vx_smoothed = 0.0
        self.vy_smoothed = 0.0
        self.wz_smoothed = 0.0

        # Raw targets from cmd_vel
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # Last real IK speeds — held during stopping so scale ramps down smoothly
        self._last_work_speeds = WheelData(fl=0.0, fr=0.0, rl=0.0, rr=0.0)

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
        if request.data == self.kinematics.compact_mode:
            response.success = True
            response.message = 'Already in requested mode'
            return response

        self.kinematics.set_compact_mode(request.data)
        self.get_logger().info(f'Compact mode {"ENABLED" if request.data else "DISABLED"}')

        vx, vy, wz = self.vx_smoothed, self.vy_smoothed, self.wz_smoothed
        is_idle = (abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3)

        target = self.kinematics.get_offset_angles() if is_idle else self.kinematics.ik_full(vx, vy, wz)[0]

        self.state_machine.enter_transit(target, self.state_machine.state, self.get_logger())

        response.success = True
        response.message = f'Compact mode {"enabled" if self.kinematics.compact_mode else "disabled"}'
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
        
        self.wz_smoothed = self._clamp_wz(self.vx_smoothed, self.vy_smoothed, self.wz_smoothed)

        vx = self.vx_smoothed
        vy = self.vy_smoothed
        wz = self.wz_smoothed

        desired_angles, _ = self.kinematics.ik_full(vx, vy, wz)

        self.state_machine.update_transitions(
            vx, vy, wz, desired_angles, self.current_angles, self.get_logger()
        )

        # Step 3 — compute work angles and speeds
        if self.state_machine.state == RoverState.NORMAL:
            work_angles, work_speeds = self.kinematics.ik_full(vx, vy, wz)
        elif self.state_machine.state == RoverState.ROTATE:
            work_angles, work_speeds = self.kinematics.ik_rotate(wz)
        else: # TRANSIT
            if self.state_machine.transit_stopping:
                work_angles = self.current_angles
                work_speeds = self._last_work_speeds
            else:
                work_angles = self.state_machine.transit_target
                work_speeds = WheelData.filled(0.0)

        if self.state_machine.state != RoverState.TRANSIT:
            self._last_work_speeds = work_speeds

        # Step 4 — speed scale
        speed_scale = self.state_machine.update_scale(dt, work_angles, self.current_angles, self.get_logger())

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

    def _step_angle(self, current: float, target: float, dt: float) -> float:
        """Advance steering angle toward target. No wrap — cable safe."""
        diff = target - current
        step = clamp(diff, -self.max_steer_rate * dt, self.max_steer_rate * dt)
        return current + step


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
