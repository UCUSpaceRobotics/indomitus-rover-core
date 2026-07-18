#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class SwerveJumpWatchdog(Node):
    def __init__(self):
        super().__init__('swerve_jump_watchdog')
        self.declare_parameter('max_angle_jump_deg', 15.0)   # per control cycle
        self.declare_parameter('max_velocity_jump', 2.0)     # rad/s per cycle
        self.prev_positions = {}
        self.prev_velocities = {}
        self.prev_time = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 50)

    def cb(self, msg: JointState):
        now = self.get_clock().now()
        max_angle_jump = math.radians(self.get_parameter('max_angle_jump_deg').value)
        max_vel_jump = self.get_parameter('max_velocity_jump').value

        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if 'wheel_mount' in name:  # steering joints
                if name in self.prev_positions:
                    diff = abs(self._wrap(pos - self.prev_positions[name]))
                    if diff > max_angle_jump:
                        self.get_logger().error(
                            f'JUMP DETECTED: {name} moved {math.degrees(diff):.1f}° '
                            f'in one message ({self.prev_positions[name]:.3f} -> {pos:.3f})')
                self.prev_positions[name] = pos
            elif 'wheel_joint' in name:  # drive joints
                if name in self.prev_velocities:
                    diff = abs(vel - self.prev_velocities[name])
                    if diff > max_vel_jump:
                        self.get_logger().error(
                            f'JUMP DETECTED: {name} velocity jumped {diff:.2f} rad/s '
                            f'({self.prev_velocities[name]:.2f} -> {vel:.2f})')
                self.prev_velocities[name] = vel

    @staticmethod
    def _wrap(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main():
    rclpy.init()
    rclpy.spin(SwerveJumpWatchdog())

if __name__ == '__main__':
    main()
