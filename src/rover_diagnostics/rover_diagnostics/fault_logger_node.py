"""Fault event logger that saves fault logs to file"""

import collections
import datetime
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from indomitus_interfaces.msg import FaultEvent

from rover_diagnostics.event_log import EventLog


EVENT_NAMES = {
    FaultEvent.EVENT_FAULT_ENTER: 'FAULT_ENTER',
    FaultEvent.EVENT_FAULT_CHANGE: 'FAULT_CHANGE',
    FaultEvent.EVENT_FAULT_CLEAR: 'FAULT_CLEAR',
    FaultEvent.EVENT_SIGNAL_LOST: 'SIGNAL_LOST',
    FaultEvent.EVENT_SIGNAL_OK: 'SIGNAL_RESTORED',
}

RECOVERY_NAMES = {
    FaultEvent.RECOVERY_NONE: 'none',
    FaultEvent.RECOVERY_IMMEDIATE: 'immediate',
    FaultEvent.RECOVERY_LIMITED: 'limited',
    FaultEvent.RECOVERY_BACKOFF: 'backoff',
    FaultEvent.RECOVERY_THERMAL: 'thermal',
    FaultEvent.RECOVERY_LATCH: 'latch',
}

# Faults carry a freeze frame; signal-presence changes have nothing meaningful
# to snapshot, since by definition no fresh feedback arrived.
FREEZE_FRAME_EVENTS = (
    FaultEvent.EVENT_FAULT_ENTER,
    FaultEvent.EVENT_FAULT_CHANGE,
)

class EventDeduper:
    """Suppresses events this logger has already written."""

    def __init__(self, history=128):
        self._history = history
        self._seen = collections.OrderedDict()

    def is_duplicate(self, msg):
        """True if this exact transition has already been seen."""
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False

        key = (
            stamp.sec, stamp.nanosec, msg.component,
            int(msg.event), int(msg.fault), int(msg.raw_code),
        )
        if key in self._seen:
            return True

        self._seen[key] = None
        while len(self._seen) > self._history:
            self._seen.popitem(last=False)
        return False


def _utc_now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


class FaultLoggerNode(Node):

    def __init__(self):
        super().__init__('fault_logger_node')

        self.declare_parameter('topic', '/fault_events')
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

        self._deduper = EventDeduper()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._sub = self.create_subscription(FaultEvent, topic, self._on_event, qos)

        self._emit({'event': 'LOG_OPENED', 'topic': topic})
        self.get_logger().info(
            f'Fault logger writing to {self._log.path} (watching {topic})')

    # ── event emission ────────────────────────────────────────────────────────

    def _emit(self, event, stamp=None):
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

    def _on_event(self, msg):
        if self._deduper.is_duplicate(msg):
            return

        record = {
            'event': EVENT_NAMES.get(msg.event, 'UNKNOWN_{}'.format(msg.event)),
            'component': msg.component,
            'esc_id': int(msg.esc_id),
            'vendor': msg.vendor,
            'fault': msg.fault_name,
            'raw_code': int(msg.raw_code),
            'recovery': RECOVERY_NAMES.get(msg.recovery, 'unknown'),
        }
        if msg.joint_name:
            record['joint_name'] = msg.joint_name
        if msg.previous_fault != msg.fault:
            record['previous_fault'] = int(msg.previous_fault)
        if msg.event in FREEZE_FRAME_EVENTS:
            record['freeze_frame'] = self._freeze_frame(msg)

        self._emit(record, msg.header.stamp)

    @staticmethod
    def _freeze_frame(msg):
        """Snapshot at fault time -- the point of the whole log."""
        return {
            'position': float(msg.position),
            'velocity': float(msg.velocity),
            'torque': float(msg.torque),
            'temperature': float(msg.temperature),
            'voltage': float(msg.voltage),
            'current': float(msg.current),
            'mode': int(msg.mode),
            'enabled': bool(msg.enabled),
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
