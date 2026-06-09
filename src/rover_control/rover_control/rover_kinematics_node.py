#!/usr/bin/env python3
"""
Rover Controller Node.

Converts cmd_vel (Twist) → wheel angles + speeds for 4-wheel swerve-drive rover.
Supports arbitrary ICR rotation (including internal points) and smooth 180-deg flipping.
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
    """Kinematics for 4-wheel swerve drive with smart stateful optimization and desaturation."""
    def __init__(self, wheelbase: float, track_width: float, wheel_radius: float, max_steer: float, max_linear: float):
        self.wheel_radius = wheel_radius
        self.max_steer = max_steer
        self.max_linear = max_linear
        self.L2 = wheelbase / 2.0
        self.W2 = track_width / 2.0

        self.compact_mode = False
        self._offset_angles = _ZERO_OFFSETS

    def set_compact_mode(self, enabled: bool):
        self.compact_mode = enabled
        self._offset_angles = _COMPACT_OFFSETS if enabled else _ZERO_OFFSETS

    def get_offset_angles(self) -> WheelData:
        return self._offset_angles

    def ik_full(self, vx: float, vy: float, wz: float, current_angles: WheelData) -> tuple[WheelData, WheelData]:
        """Full IK supporting general holonomic fields and internal/external ICR points."""
        if abs(vx) < _VXY_EPS and abs(vy) < _VXY_EPS and abs(wz) < _WZ_EPS:
            angles = self._apply_compact_offset(WheelData.filled(0.0))
            return angles, WheelData.filled(0.0)

        angles, speeds = self._ik_general(vx, vy, wz, current_angles)
        angles = self._apply_compact_offset(angles)
        
        if self.compact_mode:
            speeds = WheelData(fl=-speeds.fl, fr=-speeds.fr, rl=-speeds.rl, rr=-speeds.rr)
        return angles, speeds

    def ik_rotate(self, wz: float, current_angles: WheelData) -> tuple[WheelData, WheelData]:
        """IK for pure rotate-in-place around center."""
        angles, speeds = self._ik_general(0.0, 0.0, wz, current_angles)
        angles = self._apply_compact_offset(angles)
        
        if self.compact_mode:
            speeds = WheelData(fl=-speeds.fl, fr=-speeds.fr, rl=-speeds.rl, rr=-speeds.rr)
        return angles, speeds

    def _ik_general(self, vx: float, vy: float, wz: float, current_angles: WheelData) -> tuple[WheelData, WheelData]:
        """Core affine velocity mapping with wheel speed desaturation and stateful angle logic."""
        # 1. Translate chassis commands directly to wheel linear velocity vectors
        fl_vx = vx - wz * self.W2
        fl_vy = vy + wz * self.L2
        fr_vx = vx + wz * self.W2
        fr_vy = vy + wz * self.L2
        rl_vx = vx - wz * self.W2
        rl_vy = vy - wz * self.L2
        rr_vx = vx + wz * self.W2
        rr_vy = vy - wz * self.L2

        # 2. Calculate raw un-optimized wheel speed magnitudes
        fl_s_raw = math.hypot(fl_vx, fl_vy) / self.wheel_radius
        fr_s_raw = math.hypot(fr_vx, fr_vy) / self.wheel_radius
        rl_s_raw = math.hypot(rl_vx, rl_vy) / self.wheel_radius
        rr_s_raw = math.hypot(rr_vx, rr_vy) / self.wheel_radius

        # 3. Speed Desaturation: prevents over-speeding when ICR is tight/internal
        max_allowable_speed = self.max_linear / self.wheel_radius
        max_curr_speed = max(fl_s_raw, fr_s_raw, rl_s_raw, rr_s_raw)
        if max_curr_speed > max_allowable_speed:
            scale = max_allowable_speed / max_curr_speed
            fl_s_raw *= scale
            fr_s_raw *= scale
            rl_s_raw *= scale
            rr_s_raw *= scale

        # 4. Extract raw angles from atan2
        fl_a_raw = math.atan2(fl_vy, fl_vx)
        fr_a_raw = math.atan2(fr_vy, fr_vx)
        rl_a_raw = math.atan2(rl_vy, rl_vx)
        rr_a_raw = math.atan2(rr_vy, rr_vx)

        # 5. Stateful execution optimization taking physical hard-limits into account
        fl_a, fl_s = self._optimize_wheel(fl_a_raw, fl_s_raw, current_angles.fl)
        fr_a, fr_s = self._optimize_wheel(fr_a_raw, fr_s_raw, current_angles.fr)
        rl_a, rl_s = self._optimize_wheel(rl_a_raw, rl_s_raw, current_angles.rl)
        rr_a, rr_s = self._optimize_wheel(rr_a_raw, rr_s_raw, current_angles.rr)

        return WheelData(fl=fl_a, fr=fr_a, rl=rl_a, rr=rr_a), WheelData(fl=fl_s, fr=fr_s, rl=rl_s, rr=rr_s)

    def _optimize_wheel(self, target_angle: float, target_speed: float, current_angle: float) -> tuple[float, float]:
        """Finds closest valid angle representation inside max_steer limit to minimize steering joint travel."""
        opt1 = target_angle
        opt2 = target_angle + math.pi if target_angle < 0 else target_angle - math.pi
        
        valid_opts = []
        if -self.max_steer <= opt1 <= self.max_steer:
            valid_opts.append((opt1, target_speed))
        if -self.max_steer <= opt2 <= self.max_steer:
            valid_opts.append((opt2, -target_speed))
            
        if not valid_opts:
            dist1 = abs(opt1 - current_angle)
            dist2 = abs(opt2 - current_angle)
            if dist1 < dist2:
                return clamp(opt1, -self.max_steer, self.max_steer), target_speed
            else:
                return clamp(opt2, -self.max_steer, self.max_steer), -target_speed
                
        if len(valid_opts) == 2:
            dist1 = abs(valid_opts[0][0] - current_angle)
            dist2 = abs(valid_opts[1][0] - current_angle)
            return valid_opts[0] if dist1 <= dist2 else valid_opts[1]
        else:
            return valid_opts[0]

    def _apply_compact_offset(self, angles: WheelData) -> WheelData:
        if not self.compact_mode:
            return angles
        return WheelData(
            fl=angles.fl + self._offset_angles.fl,
            fr=angles.fr + self._offset_angles.fr,
            rl=angles.rl + self._offset_angles.rl,
            rr=angles.rr + self._offset_angles.rr
        )


class SlewRateLimiter:
    def __init__(self, max_accel: float, max_decel: float, initial_value: float = 0.0):
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.current = initial_value

    def update(self, target: float, dt: float) -> float:
        diff = target - self.current
        is_braking = (abs(target) < abs(self.current)) or (self.current * target < 0)
        rate = self.max_decel if is_braking else self.max_accel
        
        step = clamp(diff, -rate * dt, rate * dt)
        self.current += step
        return self.current


class RoverStateMachine:
    def __init__(self, align_threshold: float, scale_up_rate: float, scale_down_rate: float):
        self.align_threshold = align_threshold
        self.scale_up_rate = scale_up_rate
        self.scale_down_rate = scale_down_rate

        self.state = RoverState.NORMAL
        self.transit_dest = RoverState.NORMAL
        self.transit_target = WheelData.filled(0.0)
        
        self.transit_stopping = False
        self.current_scale = 0.0

    def enter_transit(self, target_angles: WheelData, dest: RoverState, logger=None):
        self.state = RoverState.TRANSIT
        self.transit_dest = dest
        self.transit_target = target_angles
        self.transit_stopping = True
        if logger:
            logger.info(f'TRANSIT → {dest}')

    def update_transitions(self, vx: float, vy: float, wz: float, desired_angles: WheelData, current_angles: WheelData, logger=None):
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
        if self.state == RoverState.TRANSIT:
            if self.transit_stopping:
                self.current_scale = max(0.0, self.current_scale - self.scale_down_rate * dt)
                if self.current_scale < 0.02:
                    self.current_scale = 0.0
                    self.transit_stopping = False
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

        self.declare_parameter('wheelbase',    1.20)
        self.declare_parameter('track_width',  0.80)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('max_steer_deg',      90.0)
        self.declare_parameter('max_steer_rate_deg', 45.0)
        self.declare_parameter('max_linear_speed',  0.50)
        self.declare_parameter('max_angular_speed', 0.50)
        self.declare_parameter('max_accel',         0.20)
        self.declare_parameter('max_decel',         0.50)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('cmd_vel_timeout_s', 2.0)
        self.declare_parameter('align_threshold_deg', 5.0)
        self.declare_parameter('scale_up_rate',   1.0)
        self.declare_parameter('scale_down_rate', 2.0)

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
            max_steer=self.max_steer,
            max_linear=self.max_linear
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

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        self._last_work_speeds = WheelData(fl=0.0, fr=0.0, rl=0.0, rr=0.0)

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(WheelTargets, 'wheel_targets', 10)
        self._compact_srv = self.create_service(SetBool, 'set_compact_mode', self._on_set_compact_mode)

        dt = 1.0 / self.control_freq
        self.timer = self.create_timer(dt, self.control_loop)

        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_timeout   = self.get_parameter('cmd_vel_timeout_s').value

        self.get_logger().info('RoverController started successfully with Internal ICR support.')

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

    def _on_set_compact_mode(self, request: SetBool.Request, response: SetBool.Response):
        if request.data == self.kinematics.compact_mode:
            response.success = True
            response.message = 'Already in requested mode'
            return response

        self.kinematics.set_compact_mode(request.data)
        vx, vy, wz = self.vx_smoothed, self.vy_smoothed, self.wz_smoothed
        is_idle = (abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3)

        target = self.kinematics.get_offset_angles() if is_idle else self.kinematics.ik_full(vx, vy, wz, self.current_angles)[0]
        self.state_machine.enter_transit(target, self.state_machine.state, self.get_logger())

        response.success = True
        response.message = f'Compact mode {"enabled" if self.kinematics.compact_mode else "disabled"}'
        return response

    def cmd_vel_callback(self, msg: Twist):
        self._last_cmd_vel_time = self.get_clock().now()
        self.target_vx = clamp(msg.linear.x, -self.max_linear, self.max_linear)
        self.target_vy = clamp(msg.linear.y, -self.max_linear, self.max_linear)
        self.target_wz = clamp(msg.angular.z, -self.max_angular, self.max_angular)

    def control_loop(self):
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
        if elapsed > self._cmd_vel_timeout:
            self.target_vx, self.target_vy, self.target_wz = 0.0, 0.0, 0.0

        dt = 1.0 / self.control_freq

        # Step 1 — Smooth chassis velocities via independent rate limiters
        self.vx_smoothed = self.limiters['vx'].update(self.target_vx, dt)
        self.vy_smoothed = self.limiters['vy'].update(self.target_vy, dt)
        self.wz_smoothed = self.limiters['wz'].update(self.target_wz, dt)

        vx = self.vx_smoothed
        vy = self.vy_smoothed
        wz = self.wz_smoothed

        # vx = self.target_vx
        # vy = self.target_vy
        # wz = self.target_wz

        # Step 2 — Handle transitions based on current kinematics targets
        desired_angles, _ = self.kinematics.ik_full(vx, vy, wz, self.current_angles)
        self.state_machine.update_transitions(vx, vy, wz, desired_angles, self.current_angles, self.get_logger())

        # Step 3 — Compute active execution vectors for the current state
        if self.state_machine.state == RoverState.NORMAL:
            work_angles, work_speeds = self.kinematics.ik_full(vx, vy, wz, self.current_angles)
        elif self.state_machine.state == RoverState.ROTATE:
            work_angles, work_speeds = self.kinematics.ik_rotate(wz, self.current_angles)
        else:  # TRANSIT
            if self.state_machine.transit_stopping:
                work_angles = self.current_angles
                work_speeds = self._last_work_speeds
            else:
                work_angles = self.state_machine.transit_target
                work_speeds = WheelData.filled(0.0)

        if self.state_machine.state != RoverState.TRANSIT:
            self._last_work_speeds = work_speeds

        # Step 4 — Move actual joints towards target configuration
        self.current_angles = WheelData(
            fl=self._step_angle(self.current_angles.fl, work_angles.fl, dt),
            fr=self._step_angle(self.current_angles.fr, work_angles.fr, dt),
            rl=self._step_angle(self.current_angles.rl, work_angles.rl, dt),
            rr=self._step_angle(self.current_angles.rr, work_angles.rr, dt)
        )

        # Крок 5 — ГЛОБАЛЬНА СИНХРОНІЗАЦІЯ ШВИДКОСТЕЙ (Захист підвіски та рокерів)
        
        # 1. Рахуємо коефіцієнти вирівнювання для кожного колеса
        c_fl = math.cos(work_angles.fl - self.current_angles.fl)
        c_fr = math.cos(work_angles.fr - self.current_angles.fr)
        c_rl = math.cos(work_angles.rl - self.current_angles.rl)
        c_rr = math.cos(work_angles.rr - self.current_angles.rr)

        # 2. Знаходимо глобальний масштаб за найменш вирівняним колісним модулем.
        # Це гарантує, що всі колеса сповільнюються пропорційно і жорсткість структури не порушується.
        global_align_scale = min(c_fl**2, c_fr**2, c_rl**2, c_rr**2)

        # 3. Отримуємо згладжувальний коефіцієнт від загального темпу прискорення машини
        speed_scale = self.state_machine.update_scale(dt, work_angles, self.current_angles, self.get_logger())
        total_chassis_scale = global_align_scale * speed_scale

        def get_sign(val):
            return 1.0 if val >= 0 else -1.0

        # 4. Публікація таргетів із локальним коригуванням знаку руху, але ГЛОБАЛЬНИМ масштабуванням швидкості
        out = WheelTargets()
        out.fl_angle = self.current_angles.fl
        out.fr_angle = self.current_angles.fr
        out.rl_angle = self.current_angles.rl
        out.rr_angle = self.current_angles.rr

        out.fl_speed = work_speeds.fl * get_sign(c_fl) * total_chassis_scale
        out.fr_speed = work_speeds.fr * get_sign(c_fr) * total_chassis_scale
        out.rl_speed = work_speeds.rl * get_sign(c_rl) * total_chassis_scale
        out.rr_speed = work_speeds.rr * get_sign(c_rr) * total_chassis_scale

        self.pub.publish(out)

    def _step_angle(self, current: float, target: float, dt: float) -> float:
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


if __name__ == '__main__':
    main()