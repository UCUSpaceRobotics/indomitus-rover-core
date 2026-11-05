"""Which joint state Servo should integrate from, with no ROS in it.

Humble's moveit_servo builds every outgoing point as "current state + delta",
and its current state is whatever arrives on ``joint_topic``. Fed the real
/arm/joint_states, each new point starts from where the arm has actually got
to, not from where Servo last sent it. The arm always trails its target a
little - arm_hardware_interface slew-limits the target to max_cmd_speed_rad_s,
and the MIT loop sags under load - so every cycle re-bases the motion on that
lag: steps come up short, the sag becomes the new reference, and the TCP
creeps away from the path the operator is steering.

servo_state_echo_node feeds Servo this module's answer instead: the last
command for the arm joints, real values for everything else. Servo then
integrates its own path, and the arm follows it with a constant lag rather
than one that accumulates.

The last command is only trusted while it is a plausible description of the
arm. It is dropped, and the real state used, when:

  * the forward controller is not active. Then something else (the JTC on
    home / teach) owns the joints and the last Servo command is history; the
    next teleop session must start from where the arm really is, or the
    target jumps back to the old pose on activation.
  * any joint's command is more than max_lead_rad away from the measurement.
    A blocked or colliding arm otherwise lets the command run off on its own
    while the real joint stays put.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence


class CommandState:
    """Holds the last Servo command and decides what Servo should see."""

    def __init__(self, joints: Sequence[str], max_lead_rad: float):
        self.joints: List[str] = list(joints)
        self.max_lead_rad = float(max_lead_rad)
        self._command: Optional[List[float]] = None
        self.forward_active = False
        #: Joint that broke the lead limit on the last drop, for the node's warning.
        self.last_drop_joint: Optional[str] = None

    @property
    def tracking(self) -> bool:
        """True while Servo is being fed its own last command."""
        return self._command is not None

    def set_forward_active(self, active: bool) -> None:
        self.forward_active = bool(active)
        if not self.forward_active:
            self._command = None

    def on_command(self, positions: Sequence[float]) -> None:
        """A new Servo command, in self.joints order."""
        if not self.forward_active or len(positions) != len(self.joints):
            self._command = None
            return
        self._command = [float(p) for p in positions]

    def resolve(self, measured: Dict[str, float]) -> Dict[str, float]:
        """Positions to report to Servo, given the measured ones by joint name.

        Joints outside self.joints, and every joint while not tracking, pass
        through unchanged. Drops the command (and reports the measurement) if
        it has drifted beyond max_lead_rad on any joint.
        """
        out = dict(measured)
        if self._command is None:
            return out
        for name, cmd in zip(self.joints, self._command):
            if name not in measured:
                # No measurement to bound the lead against: don't guess.
                self._command = None
                self.last_drop_joint = name
                return dict(measured)
            if abs(cmd - measured[name]) > self.max_lead_rad:
                self._command = None
                self.last_drop_joint = name
                return dict(measured)
        for name, cmd in zip(self.joints, self._command):
            out[name] = cmd
        return out
