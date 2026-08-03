"""Fault event logger.

Subscribes to the chassis motor status topic, detects fault *transitions*, and
appends them to a rotating JSONL file along with a freeze frame of the motor
state at the moment of the transition.

This node deliberately lives outside the hardware interface: writing to disk
from the ros2_control update loop risks stalling it, and a stall longer than
the Damiao TIMEOUT register would provoke the very communication-loss fault we
are trying to record.

Only transitions are logged, never steady state -- continuous telemetry belongs
in a rosbag. A one-hour run with no faults produces two lines.
"""

import datetime
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from indomitus_interfaces.msg import ChassisStatus

from rover_diagnostics.event_log import EventLog
from rover_diagnostics import fault_codes


def _utc_now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


class MotorTracker:
    """Remembers the last observed condition of one motor."""

    __slots__ = ('faults', 'health_valid', 'enabled', 'seen')

    def __init__(self):
        self.faults = []
        self.health_valid = True
        self.enabled = False
        self.seen = False


class FaultLoggerNode(Node):

    def __init__(self):
        super().__init__('fault_logger_node')

        self.declare_parameter('topic', '/chassis/motor_states')
        self.declare_parameter('log_dir', '~/.ros/rover_faults')
        self.declare_parameter('log_prefix', 'faults')
        self.declare_parameter('max_bytes', 5 * 1024 * 1024)
        self.declare_parameter('backup_count', 3)

        topic = self.get_parameter('topic').value

        self._log = EventLog(
            directory=self.get_parameter('log_dir').value,
            prefix=self.get_parameter('log_prefix').value,
            max_bytes=int(self.get_parameter('max_bytes').value),
            backup_count=int(self.get_parameter('backup_count').value),
        )

        self._started_at = time.monotonic()
        self._trackers = {}

        # Status is published at 10 Hz and only transitions matter, so a short
        # best-effort queue is right: never block the publisher, and a dropped
        # message costs at most 100 ms of resolution.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            ChassisStatus, topic, self._on_status, qos)

        self._emit({'event': 'LOG_OPENED', 'topic': topic})
        self.get_logger().info(
            f'Fault logger writing to {self._log.path} (watching {topic})')

    # ── event emission ────────────────────────────────────────────────────────

    def _emit(self, event: dict[str, str], stamp=None):
        """Stamp an event with three clocks and append it to the log.

        Wall time is what a human correlates with the run; ROS time is what
        lines up with a rosbag; uptime survives both a wrong system clock and
        a mid-run NTP step, which is common on a freshly booted rover.
        """
        record = {
            't_wall': _utc_now_iso(),
            't_uptime': round(time.monotonic() - self._started_at, 3),
        }
        if stamp is not None:
            record['t_ros'] = round(stamp.sec + stamp.nanosec * 1e-9, 3)
        record.update(event)

        try:
            self._log.write(record)
        except OSError as exc:
            # A logger that kills the process it is observing is worse than no
            # logger. Report and keep running.
            self.get_logger().error('Fault log write failed: {}'.format(exc))

    # ── transition detection ──────────────────────────────────────────────────

    def _on_status(self, msg):
        for motor in msg.motors:
            key = (motor.motor_type, int(motor.esc_id))
            tracker = self._trackers.get(key)
            if tracker is None:
                tracker = MotorTracker()
                self._trackers[key] = tracker
            self._check_motor(motor, tracker, msg.header.stamp)

    def _check_motor(self, motor, tracker, stamp):
        component = '{}/{}'.format(motor.motor_type, motor.joint_name or motor.esc_id)

        # Feedback presence is tracked separately from faults: a motor that has
        # gone silent reports no fault code at all, so without this a motor
        # dropping off the bus would look identical to a healthy one.
        if motor.health_valid != tracker.health_valid or not tracker.seen:
            if tracker.seen:
                self._emit({
                    'event': 'SIGNAL_RESTORED' if motor.health_valid else 'SIGNAL_LOST',
                    'component': component,
                    'esc_id': int(motor.esc_id),
                    'vendor': motor.motor_type,
                }, stamp)
            tracker.health_valid = motor.health_valid

        if not motor.health_valid:
            # Fault codes are meaningless without feedback; hold the last known
            # fault set so recovery is detected correctly once it returns.
            tracker.seen = True
            return

        faults = fault_codes.decode(motor.motor_type, motor.fault_code, motor.mode)

        if faults != tracker.faults:
            if not tracker.faults:
                event = 'FAULT_ENTER'
            elif not faults:
                event = 'FAULT_CLEAR'
            else:
                event = 'FAULT_CHANGE'

            record = {
                'event': event,
                'component': component,
                'esc_id': int(motor.esc_id),
                'vendor': motor.motor_type,
                'faults': faults,
                'raw_code': int(motor.fault_code),
            }
            if tracker.faults:
                record['previous_faults'] = tracker.faults
            if faults:
                record['freeze_frame'] = self._freeze_frame(motor)

            self._emit(record, stamp)
            tracker.faults = faults

        if motor.enabled != tracker.enabled and tracker.seen:
            self._emit({
                'event': 'ENABLED' if motor.enabled else 'DISABLED',
                'component': component,
                'esc_id': int(motor.esc_id),
                'vendor': motor.motor_type,
            }, stamp)
        tracker.enabled = motor.enabled

        tracker.seen = True

    @staticmethod
    def _freeze_frame(motor):
        """Snapshot of the motor at fault time -- the point of the whole log."""
        return {
            'position': float(motor.position),
            'velocity': float(motor.velocity),
            'torque': float(motor.torque),
            'temperature': float(motor.temperature),
            'voltage': float(motor.voltage),
            'current': float(motor.current),
            'mode': int(motor.mode),
            'enabled': bool(motor.enabled),
            'kinematic_valid': bool(motor.kinematic_valid),
        }

    def destroy_node(self):
        self._emit({'event': 'LOG_CLOSED'})
        self._log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaultLoggerNode()
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
