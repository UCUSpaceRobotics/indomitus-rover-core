"""Thin ROS pub/sub client for the end-effector CAN bridge.

See arm_peripherals/end_effector_can_node.py for the actual CAN link —
this module only talks to it over end_effector_controller/command and
end_effector_controller/state.
"""

from std_msgs.msg import String
from indomitus_interfaces.msg import EndEffectorState


class EndEffectorClient:
    """Publishes one-shot commands to end_effector_controller/command,
    tracks latest end_effector_controller/state. See arm_peripherals/
    end_effector_can_node.py for the actual CAN link."""

    _VALID_COMMANDS = frozenset({
        'open', 'close', 'drill_up', 'drill_down',
        'stop_step', 'stop_drill', 'lock', 'unlock',
    })

    def __init__(self, node):
        self._pub = node.create_publisher(String, 'end_effector_controller/command', 10)
        self._state_sub = node.create_subscription(
            EndEffectorState, 'end_effector_controller/state', self._on_state, 10)
        self._logger = node.get_logger()
        self.load_right_g = 0.0
        self.load_left_g = 0.0
        self.connected = False

    def send(self, command: str) -> None:
        assert command in self._VALID_COMMANDS, command
        self._pub.publish(String(data=command))

    def _on_state(self, msg) -> None:
        self.load_right_g = msg.load_right_g
        self.load_left_g = msg.load_left_g
        self.connected = msg.connected
        self._logger.info(
            f'End-effector load: right={self.load_right_g}g left={self.load_left_g}g',
            throttle_duration_sec=1.0,
        )


