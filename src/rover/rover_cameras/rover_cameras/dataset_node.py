"""Dataset capture across all cameras: one place for "every camera at once".

    capture         std_srvs/Trigger  save one frame from every capturing camera
    record          std_srvs/SetBool  start / stop recording on every camera
    record/toggle   std_srvs/Trigger  on unless any camera already records
    dataset_state   indomitus_interfaces/DatasetState, latched, aggregated

Runs in the cameras namespace, next to the camera nodes, and only calls their
own <name>/capture and <name>/record; a single camera is captured by calling
those directly. Holds no state of its own: recording lives in the cameras
(a respawned camera comes back not recording) and is read back from their
<name>/dataset_state.
"""

import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from indomitus_interfaces.msg import DatasetState

STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Camera answers that mean "not mounted / not up", not a failure: cameras.yaml
# lists every camera that may be mounted.
SKIPPED = ('not capturing', 'not running')


def summarize_capture(results):
    """(success, message) for {camera: (ok, path_or_error)}."""
    saved = {cam: msg for cam, (ok, msg) in results.items() if ok}
    skipped = [cam for cam, (ok, msg) in results.items() if not ok and msg in SKIPPED]
    failed = {cam: msg for cam, (ok, msg) in results.items() if not ok and msg not in SKIPPED}
    parts = ['saved %d' % len(saved)]
    if saved:
        parts[0] += ': ' + ', '.join('%s=%s' % kv for kv in sorted(saved.items()))
    if failed:
        parts.append('failed: ' + ', '.join('%s (%s)' % kv for kv in sorted(failed.items())))
    if skipped:
        parts.append('skipped: ' + ', '.join(sorted(skipped)))
    return bool(saved) and not failed, '; '.join(parts)


def summarize_record(results, target):
    """(success, message) for {camera: (ok, message)} after setting `target`."""
    done = [cam for cam, (ok, _) in results.items() if ok]
    failed = {cam: msg for cam, (ok, msg) in results.items()
              if not ok and msg != 'not running'}
    message = 'recording %s (%d camera%s)' % ('ON' if target else 'OFF', len(done),
                                              '' if len(done) == 1 else 's')
    if failed:
        message += '; failed: ' + ', '.join('%s (%s)' % kv for kv in sorted(failed.items()))
    return bool(done) and not failed, message


def aggregate(states):
    """One DatasetState from the per-camera ones."""
    msg = DatasetState()
    msg.recording = any(s.recording for s in states)
    msg.cameras_live = sum(s.cameras_live for s in states)
    msg.saved = sum(s.saved for s in states)
    return msg


class DatasetNode(Node):

    def __init__(self):
        super().__init__('dataset')
        self.cameras = list(self.declare_parameter('cameras', ['']).value)
        self.cameras = [c for c in self.cameras if c]
        self.timeout = float(self.declare_parameter('timeout', 5.0).value)

        clients = ReentrantCallbackGroup()
        self._capture = {c: self.create_client(Trigger, '%s/capture' % c, callback_group=clients)
                         for c in self.cameras}
        self._record = {c: self.create_client(SetBool, '%s/record' % c, callback_group=clients)
                        for c in self.cameras}
        self._states = {}
        self._lock = threading.Lock()
        self.state_pub = self.create_publisher(DatasetState, 'dataset_state', STATE_QOS)
        for c in self.cameras:
            self.create_subscription(DatasetState, '%s/dataset_state' % c,
                                     lambda msg, c=c: self._on_state(c, msg), STATE_QOS,
                                     callback_group=clients)
        self.state_pub.publish(aggregate([]))

        # One request at a time; the calls to the cameras run in `clients`.
        services = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, 'capture', self._on_capture, callback_group=services)
        self.create_service(SetBool, 'record', self._on_record, callback_group=services)
        self.create_service(Trigger, 'record/toggle', self._on_toggle, callback_group=services)
        self.get_logger().info('dataset over %d camera(s): %s'
                               % (len(self.cameras), ', '.join(self.cameras)))

    def _on_state(self, camera, msg):
        with self._lock:
            self._states[camera] = msg
            total = aggregate(list(self._states.values()))
        self.state_pub.publish(total)

    def _call_all(self, clients, request):
        """{camera: (ok, message)}: every camera called at once, then waited for."""
        results, futures = {}, {}
        for cam, client in clients.items():
            if client.service_is_ready():
                futures[cam] = client.call_async(request)
            else:
                results[cam] = (False, 'not running')
        deadline = time.monotonic() + self.timeout
        while futures and time.monotonic() < deadline and \
                not all(f.done() for f in futures.values()):
            time.sleep(0.01)
        for cam, future in futures.items():
            if not future.done():
                future.cancel()
                results[cam] = (False, 'timeout')
            elif future.exception() is not None:
                results[cam] = (False, repr(future.exception()))
            else:
                res = future.result()
                results[cam] = (res.success, res.message)
        return results

    def _on_capture(self, request, response):
        response.success, response.message = summarize_capture(
            self._call_all(self._capture, Trigger.Request()))
        self._log(response)
        return response

    def _set_recording(self, target, response):
        response.success, response.message = summarize_record(
            self._call_all(self._record, SetBool.Request(data=target)), target)
        self._log(response)
        return response

    def _on_record(self, request, response):
        return self._set_recording(request.data, response)

    def _on_toggle(self, request, response):
        # On unless any camera already records: a set that is out of step (one
        # camera respawned mid-run) is first brought to all-off.
        with self._lock:
            recording = any(s.recording for s in self._states.values())
        return self._set_recording(not recording, response)

    def _log(self, response):
        # Separate call sites: rclpy pins a severity to each one.
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)


def main(args=None):
    rclpy.init(args=args)
    node = DatasetNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
