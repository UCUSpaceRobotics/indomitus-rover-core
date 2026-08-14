#!/usr/bin/env python3
"""
Keyboard control node for MoveIt Servo.

Reads keyboard input and sends Cartesian velocity commands to MoveIt Servo.
Designed to be modular — core logic is in ServoController class,
so switching to gamepad (joy package) requires only replacing the input source.

Controls:
    EEF translation — absolute (in arm_mount_link):
        w / s  — +X / -X
        a / d  — +Y / -Y
        q / e  — +Z / -Z

    EEF translation — view-relative (in arm_camera_link, rigid with the
    gripper; axes are REP-103 so +X is where the camera and gripper point):
        Up / Down    — forward / back
        Left / Right — left / right
        t / g        — up / down

    Both translation sets are summed, so they can be held together.

    EEF rotation (about arm_tcp_link). Names are from the operator's point of
    view, i.e. the camera frame, which is what the TCP axes actually work out
    to: TCP +X is the camera's left-right axis (pitch), TCP +Y its vertical
    axis (yaw), TCP +Z its line of sight (roll):
        i / k  — pitch (+/- wx)
        u / o  — yaw   (+/- wy)
        j / l  — roll  (+/- wz)

    r      — move to home + start servo
    ESC/x  — exit

Gamepad controls (via ros2 joy joy_node, e.g. Stadia controller).
All gamepad translation is view-relative (arm_camera_link); rotation is
about arm_tcp_link, same as the keyboard's I/K/U/O/J/L:
    Left stick  left/right — EEF left / right (camera)
    Left stick  up/down    — EEF forward / back (camera)
    Right stick up/down    — EEF up / down (camera)
    Right stick left/right — yaw (TCP)
    R1 + right stick up/down    — pitch (TCP)
    R1 + right stick left/right — roll (TCP)
    A                — move to home + start servo
    X                — exit

Usage (stack in one terminal, input in another):
        ros2 launch arm_moveit_config demo.launch.py use_fake_hardware:=false
        ros2 run arm_tasks keyboard_servo_node
        # optional: pin a device — ros2 run ... --ros-args -p keyboard_device_path:=/dev/input/event19

    Gazebo sim (sim clock + faster speeds, see arm_sim/config/keyboard_servo_sim.yaml):
        ros2 run arm_tasks keyboard_servo_node --ros-args \\
            --params-file $(ros2 pkg prefix arm_sim)/share/arm_sim/config/keyboard_servo_sim.yaml

    Gamepad only (start the joystick driver first):
        ros2 run joy joy_node
        ros2 run arm_tasks gamepad_servo_node
        # optional: pin the shift button — ros2 run ... --ros-args -p gamepad_shift_button:=5
"""

import sys
import os
import fcntl
import threading
import termios
import time
import json
import math
import select
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, TwistStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import evdev
from evdev import ecodes


DEFAULT_LINEAR_SPEED  = 0.2
DEFAULT_ANGULAR_SPEED = 0.6
DEFAULT_PUBLISH_RATE  = 100.0
# Q/E (±Z) folds the shoulder/elbow; without ω the TCP pitch walks.
# Hold attitude only — do not scale XYZ (that made teleop feel slow).
HOLD_ANGULAR_GAIN = 6.0
HOLD_ANGULAR_MAX = 0.8
HOLD_CMD_EPS = 1e-4
# Cartesian teleop publishes the twist in arm_mount_link (Servo command frame).
# Stick XYZ are already in that frame. Stick roll/pitch/yaw are interpreted in
# arm_tcp_link and rotated into mount before publish so orientation stays
# EEF-local while translation stays world/mount-aligned.
DEFAULT_LINEAR_FRAME  = 'arm_mount_link'
DEFAULT_EE_FRAME      = 'arm_tcp_link'
# Second, view-relative translation set (arrows + T/G). The camera is
# rigidly bolted to arm_end_effector_link, same as the gripper, so "forward"
# is the same physical direction either way — but arm_camera_link is the frame
# whose axes are REP-103 (+X forward / +Y left / +Z up), matching the sign
# convention of the mount-frame keys. arm_tcp_link is NOT a drop-in substitute:
# it inherits the EEF axes, where the gripper points along +Z, so the arrow
# keys would mean something else entirely.
DEFAULT_VIEW_FRAME    = 'arm_camera_link'
# Fallback if poses.json "home" cannot be loaded (matches SRDF group_state home).
DEFAULT_HOME_POSE     = [-1.552, 0.5057, 1.1731, 0.717, 0.0093, -1.536]
# 'auto' picks a USB/external keyboard (Keychron, etc.) over the laptop's
# built-in AT Translated Set 2 device — the usual failure mode in Docker.
DEFAULT_KEYBOARD_DEVICE_PATH = 'auto'
# Index of the gamepad button held to shift the right stick from
# up-down + yaw to pitch + roll. Not portable across controller models or
# connections (a Stadia pad over Bluetooth had R1 at 10, not 5) — if it does
# nothing on your pad, read the real index from the /joy raw log
# (_log_raw_joy) and override with --ros-args -p gamepad_shift_button:=<n>.
DEFAULT_GAMEPAD_SHIFT_BUTTON = 5
DEFAULT_SAFE_POSE_TIMEOUT = 60.0
# The home move goes straight to the trajectory controller, so none of
# Servo's velocity scaling applies — this duration is the only thing bounding
# how fast the arm swings, however far it has to travel.
DEFAULT_SAFE_POSE_DURATION = 6.0

JTC_CONTROLLER_NAME = 'indomitus_arm_controller'
FORWARD_CONTROLLER_NAME = 'indomitus_arm_forward_position_controller'

HOME_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]
# Back-compat aliases used by older call sites / docs.
SAFE_POSE_JOINTS = HOME_POSE_JOINTS
DEFAULT_SAFE_POSE = DEFAULT_HOME_POSE


def _load_home_pose_from_json():
    """Return home joint positions from poses.json, or None if unavailable."""
    candidates = [
        Path('/opt/ws/src/arm/arm_tasks/poses.json'),
        Path(__file__).resolve().parent.parent / 'poses.json',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('arm_tasks')) / 'poses.json'
        candidates.insert(0, share)
    except Exception:
        pass

    for path in candidates:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            home = data.get('home') or {}
            return [float(home[name]) for name in HOME_POSE_JOINTS]
        except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
            continue
    return None


def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def _quat_conj(q: Quaternion) -> Quaternion:
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _quat_rotvec(q: Quaternion):
    w = max(-1.0, min(1.0, q.w))
    x, y, z = q.x, q.y, q.z
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    half = math.acos(w)
    sine = math.sqrt(max(0.0, 1.0 - w * w))
    if sine < 1e-8:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    scale = 2.0 * half / sine
    return (scale * x, scale * y, scale * z)


def _rotate_vector_by_quat(q, x: float, y: float, z: float):
    """Rotate a free vector by a geometry_msgs quaternion (x,y,z,w)."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _list_keyboard_candidates():
    """Return evdev devices that look like QWERTY keyboards (path, name, score)."""
    required = {ecodes.KEY_R, ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_ESC}
    candidates = []
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
        except (FileNotFoundError, PermissionError, OSError):
            continue
        keys = set(device.capabilities().get(ecodes.EV_KEY, []))
        if not required.issubset(keys):
            continue
        name = device.name or ''
        phys = device.phys or ''
        name_l = name.lower()
        # Skip obvious non-keyboards that still expose a few KEY_* codes.
        if any(bad in name_l for bad in ('sleep', 'lid', 'power', 'video bus', 'hdmi', 'headphone')):
            continue
        score = 0
        if 'usb' in phys or phys.startswith('usb-'):
            score += 100
        if '/input0' in phys:
            score += 20  # main HID collection on multi-interface boards
        if 'keychron' in name_l or 'keyboard' in name_l:
            score += 10
        if 'at translated' in name_l or phys.startswith('isa'):
            score -= 50  # laptop PS/2 — usually wrong when an external KB is plugged in
        score += min(len(keys), 200) / 200.0
        candidates.append((score, path, name, phys))
    candidates.sort(reverse=True)
    return candidates


def _resolve_keyboard_device_path(requested: str) -> str | None:
    """Resolve ``auto`` / empty to the best keyboard path, else return ``requested``."""
    requested = (requested or '').strip()
    if requested and requested.lower() != 'auto':
        return requested
    candidates = _list_keyboard_candidates()
    if not candidates:
        return None
    return candidates[0][1]

SERVO_STATUS_INVALID                              = -1
SERVO_STATUS_OK                                    = 0
SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY = 1
SERVO_STATUS_HALT_FOR_SINGULARITY                  = 2
SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY    = 3
SERVO_STATUS_DECELERATE_FOR_COLLISION              = 4
SERVO_STATUS_HALT_FOR_COLLISION                    = 5
SERVO_STATUS_JOINT_BOUND                           = 6

SERVO_STATUS_NAMES = {
    SERVO_STATUS_INVALID: 'INVALID',
    SERVO_STATUS_OK: 'NO_WARNING',
    SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
    SERVO_STATUS_HALT_FOR_SINGULARITY: 'HALT_FOR_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY: 'DECELERATE_FOR_LEAVING_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_COLLISION: 'DECELERATE_FOR_COLLISION',
    SERVO_STATUS_HALT_FOR_COLLISION: 'HALT_FOR_COLLISION',
    SERVO_STATUS_JOINT_BOUND: 'JOINT_BOUND',
}


class ServoController(Node):
    """ROS2 node that turns Cartesian velocity commands into MoveIt Servo messages.

    Declares and reads ROS parameters for speed, publish rate, command frame,
    safe pose and keyboard device path, publishes ``TwistStamped`` messages on
    a timer, and exposes helper methods to start/stop Servo and to drive the
    arm to a predefined safe pose via a ``FollowJointTrajectory`` action.
    """

    def __init__(self):
        """Initialize the node, declare parameters, and set up pub/sub/clients.

        Declares all ROS parameters (with defaults), reads their resolved
        values into instance attributes, zeroes the internal velocity state,
        creates the twist publisher, the Servo start/stop service clients,
        the trajectory action client, the publish timer, and the servo
        status subscription.
        """

        super().__init__('keyboard_servo_node')

        self.declare_parameter('linear_speed',  DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('publish_rate',  DEFAULT_PUBLISH_RATE)
        self.declare_parameter('linear_frame',  DEFAULT_LINEAR_FRAME)
        # Deprecated alias for linear_frame (older launch/params files).
        self.declare_parameter('command_frame', DEFAULT_LINEAR_FRAME)
        self.declare_parameter('ee_frame',      DEFAULT_EE_FRAME)
        self.declare_parameter('view_frame',    DEFAULT_VIEW_FRAME)
        # A / R move to this joint vector (defaults to poses.json "home").
        self.declare_parameter('safe_pose',     DEFAULT_HOME_POSE)
        self.declare_parameter('home_pose_name', 'home')
        self.declare_parameter('keyboard_device_path', DEFAULT_KEYBOARD_DEVICE_PATH)
        self.declare_parameter('gamepad_shift_button', DEFAULT_GAMEPAD_SHIFT_BUTTON)
        self.declare_parameter('safe_pose_timeout', DEFAULT_SAFE_POSE_TIMEOUT)
        self.declare_parameter('safe_pose_duration', DEFAULT_SAFE_POSE_DURATION)

        self._linear_speed  = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._publish_rate  = self.get_parameter('publish_rate').value
        linear_frame = self.get_parameter('linear_frame').value
        command_frame = self.get_parameter('command_frame').value
        if linear_frame != DEFAULT_LINEAR_FRAME:
            self._linear_frame = linear_frame
        elif command_frame != DEFAULT_LINEAR_FRAME:
            self._linear_frame = command_frame
        else:
            self._linear_frame = DEFAULT_LINEAR_FRAME
        self._ee_frame      = self.get_parameter('ee_frame').value
        self._view_frame    = self.get_parameter('view_frame').value
        self._home_pose_name = self.get_parameter('home_pose_name').value
        # Prefer poses.json home unless the caller overrode safe_pose explicitly.
        pose_from_param = list(self.get_parameter('safe_pose').value)
        pose_from_json = _load_home_pose_from_json()
        if pose_from_param == list(DEFAULT_HOME_POSE) and pose_from_json is not None:
            self._safe_pose = pose_from_json
            pose_source = f'poses.json["{self._home_pose_name}"]'
        else:
            self._safe_pose = pose_from_param
            pose_source = 'safe_pose parameter'
        self._keyboard_device_path = self.get_parameter('keyboard_device_path').value
        self._gamepad_shift_button = int(self.get_parameter('gamepad_shift_button').value)
        self._safe_pose_timeout    = self.get_parameter('safe_pose_timeout').value
        self._safe_pose_duration   = self.get_parameter('safe_pose_duration').value

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # View-relative translation, kept separate from vx/vy/vz because it is
        # expressed in view_frame and only resolved to linear_frame at publish
        # time — the transform changes as the arm moves.
        self.view_vx = 0.0
        self.view_vy = 0.0
        self.view_vz = 0.0
        self._hold_quat = None
        self._joint_positions = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Servo subscribes BEST_EFFORT; default RELIABLE can drop twists.
        self._pub = self.create_publisher(
            TwistStamped, 'servo_node/delta_twist_cmds', qos_profile_sensor_data
        )
        self._start_client = self.create_client(Trigger, 'servo_node/start_servo')
        self._stop_client  = self.create_client(Trigger, 'servo_node/stop_servo')
        self._switch_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller'
        )
        self._traj_client  = ActionClient(
            self,
            FollowJointTrajectory,
            'indomitus_arm_controller/follow_joint_trajectory'
        )
        self._js_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

        self._servo_status = SERVO_STATUS_OK
        self._status_sub = self.create_subscription(
            Int8,
            'servo_node/status',
            self._on_servo_status,
            10
        )

        self.get_logger().info(
            f'ServoController ready — '
            f'linear_speed={self._linear_speed}, '
            f'angular_speed={self._angular_speed}, '
            f'linear_frame={self._linear_frame} (XYZ + Servo twist frame), '
            f'ee_frame={self._ee_frame} (roll/pitch/yaw input), '
            f'view_frame={self._view_frame} (arrow-key translation), '
            f'A/R home from {pose_source}: {[round(v, 4) for v in self._safe_pose]}'
        )

    @property
    def linear_speed(self) -> float:
        """Return the configured linear speed scale, in meters per second."""
        return self._linear_speed

    @property
    def angular_speed(self) -> float:
        """Return the configured angular speed scale, in radians per second."""
        return self._angular_speed

    @property
    def keyboard_device_path(self) -> str:
        """Return the filesystem path of the keyboard input device (evdev)."""
        return self._keyboard_device_path

    @property
    def gamepad_shift_button(self) -> int:
        """Return the Joy button index that shifts the right stick."""
        return self._gamepad_shift_button

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0,
                     view_vx=0.0, view_vy=0.0, view_vz=0.0):
        """Set the current Cartesian velocity command.

        Args:
            vx: Linear velocity along global (mount) X, m/s.
            vy: Linear velocity along global (mount) Y, m/s.
            vz: Linear velocity along global (mount) Z, m/s.
            wx: Angular velocity about global X (roll of EEF), rad/s.
            wy: Angular velocity about global Y (pitch of EEF), rad/s.
            wz: Angular velocity about global Z (yaw of EEF), rad/s.
            view_vx: Linear velocity forward/back in ``view_frame``, m/s.
            view_vy: Linear velocity left/right in ``view_frame``, m/s.
            view_vz: Linear velocity up/down in ``view_frame``, m/s.

        Notes:
            Linear velocities are in ``linear_frame`` (default
            ``arm_mount_link``). Angular velocities are specified in
            ``ee_frame`` (default ``arm_tcp_link``) and rotated into
            ``linear_frame`` before publish. Servo's
            ``robot_link_command_frame`` must match ``linear_frame``.

            The ``view_*`` components are an independent translation set in
            ``view_frame`` (default ``arm_camera_link``); they are rotated
            into ``linear_frame`` and *added* to vx/vy/vz, so pressing keys
            from both sets at once simply sums the two motions.

            The three trailing arguments are keyword-friendly on purpose:
            ``gamepad_servo_node`` calls this with six positional values and
            must keep working unchanged.
        """
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.view_vx = view_vx
        self.view_vy = view_vy
        self.view_vz = view_vz

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = float(pos)

    def _current_arm_positions(self):
        """Latest measured arm joints, or None if any name is missing."""
        try:
            return [self._joint_positions[n] for n in HOME_POSE_JOINTS]
        except KeyError:
            return None

    @staticmethod
    def _home_trajectory(q0, q1, duration: float, n_points: int = 24):
        """Rest-to-rest quintic in joint space (zero vel/accel at ends)."""
        n_points = max(2, int(n_points))
        dq = [b - a for a, b in zip(q0, q1)]
        points = []
        for i in range(1, n_points + 1):
            u = i / n_points
            s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
            ds_du = 30.0 * u * u * (1.0 - 2.0 * u + u * u)
            sdot = ds_du / duration
            t = u * duration
            pt = JointTrajectoryPoint()
            pt.positions = [a + s * d for a, d in zip(q0, dq)]
            pt.velocities = [sdot * d for d in dq]
            pt.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            points.append(pt)
        points[-1].positions = list(q1)
        points[-1].velocities = [0.0] * len(q1)
        return points

    def stop(self):
        """Zero out all velocity components, halting Cartesian motion.

        Equivalent to calling ``set_velocity()`` with no arguments.
        """
        self.set_velocity()
        self._hold_quat = None

    def _switch_controllers(self, activate, deactivate) -> bool:
        """Activate/deactivate ros2_control controllers (JTC <-> forward).

        Args:
            activate: Controllers to activate.
            deactivate: Controllers to deactivate.

        Returns:
            True if the switch service reported success.
        """
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('controller_manager/switch_controller unavailable')
            return False

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = Duration(sec=3, nanosec=0)

        done_event = threading.Event()
        outcome = {'ok': False}

        def _cb(future):
            try:
                res = future.result()
                outcome['ok'] = bool(res.ok)
                if not res.ok:
                    self.get_logger().error(
                        f'Controller switch failed (activate={activate}, '
                        f'deactivate={deactivate})'
                    )
            except Exception as exc:
                self.get_logger().error(f'Controller switch exception: {exc!r}')
            finally:
                done_event.set()

        future = self._switch_client.call_async(req)
        future.add_done_callback(_cb)
        if not done_event.wait(timeout=5.0):
            self.get_logger().error('Controller switch timed out')
            return False
        if outcome['ok']:
            self.get_logger().info(
                f'Controllers: activate={list(activate)} deactivate={list(deactivate)}'
            )
        return outcome['ok']

    def use_trajectory_controller(self) -> bool:
        """Claim joints with JTC for home / Plan&Execute / teach_poses."""
        return self._switch_controllers(
            activate=[JTC_CONTROLLER_NAME],
            deactivate=[FORWARD_CONTROLLER_NAME],
        )

    def use_streaming_controller(self) -> bool:
        """Claim joints with forward position controller for Servo teleop."""
        return self._switch_controllers(
            activate=[FORWARD_CONTROLLER_NAME],
            deactivate=[JTC_CONTROLLER_NAME],
        )

    def stop_servo(self) -> bool:
        """Call the Servo ``stop_servo`` service and wait for confirmation.

        Waits up to 2 seconds for the service to become available and up to
        3 seconds for the asynchronous call to complete. After Servo stops,
        re-activates the trajectory controller so home / Execute can run.

        Returns:
            bool: True if the service was available and the call completed
            within the timeout; False if the service was unavailable or the
            call timed out.
        """
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo stop service not available')
            return False

        done_event = threading.Event()

        def _cb(future):
            """Mark the stop-servo call as complete (service response callback)."""
            done_event.set()

        future = self._stop_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=3.0):
            self.get_logger().warn('Servo stop timed out')
            return False

        # Prefer JTC when teleop is idle so Plan&Execute / home work.
        self.use_trajectory_controller()
        return True

    def move_to_safe_pose(self):
        """Stop motion and drive the arm to the configured home pose.

        Halts current velocity commands, confirms Servo has stopped, then
        sends a ``FollowJointTrajectory`` goal to move all joints to the
        home / ``safe_pose`` parameter values over ``safe_pose_duration``
        (of controller time, i.e. sim time under Gazebo) and blocks until
        the controller reports the goal finished, up to
        ``safe_pose_timeout`` wall-clock seconds (<= 0 waits forever; the
        default is generous because under a slow sim the trajectory
        legitimately takes longer in wall time). This method runs on a
        dedicated thread, so waiting does not stall keyboard handling.

        Returns:
            bool: True if the controller reported the goal SUCCEEDED with
            error code SUCCESSFUL; False if Servo could not be confirmed
            stopped, the action server was unavailable, the goal was
            rejected, the trajectory was aborted/canceled or finished with
            a controller error, or no result arrived within the timeout.
        """
        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting home move.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting home move.'
            )
            return False

        if not self._traj_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        traj = JointTrajectory()
        traj.joint_names = HOME_POSE_JOINTS
        q0 = self._current_arm_positions()
        q1 = list(self._safe_pose)
        if q0 is None:
            self.get_logger().warn(
                'No joint_states yet — home is a single waypoint'
            )
            point = JointTrajectoryPoint()
            point.positions = q1
            point.velocities = [0.0] * len(q1)
            point.time_from_start = Duration(
                sec=int(self._safe_pose_duration),
                nanosec=int((self._safe_pose_duration % 1.0) * 1e9),
            )
            traj.points = [point]
        else:
            traj.points = self._home_trajectory(
                q0, q1, self._safe_pose_duration
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(
            f'Moving to home ({self._home_pose_name}): '
            f'{[round(v, 4) for v in self._safe_pose]}'
        )

        done_event = threading.Event()
        outcome = {'success': False, 'error': ''}

        def result_cb(future):
            """Record the trajectory result's status and error code."""
            try:
                wrapped = future.result()
                result = wrapped.result
                if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                    outcome['success'] = True
                else:
                    outcome['error'] = (
                        f'goal status {wrapped.status}, '
                        f'controller error code {result.error_code}'
                        + (f' ({result.error_string})' if result.error_string else '')
                    )
            except Exception as e:
                outcome['error'] = f'failed to read result: {e!r}'
            finally:
                done_event.set()

        def goal_response_cb(future):
            """Handle the trajectory action's goal-acceptance response.

            Args:
                future: Future resolving to the goal handle returned by
                    ``send_goal_async``.
            """
            try:
                goal_handle = future.result()
            except Exception as e:
                goal_handle = None
                outcome['error'] = f'goal request failed: {e!r}'
            if not goal_handle or not goal_handle.accepted:
                outcome['error'] = outcome['error'] or 'goal rejected'
                done_event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_cb)

        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        timeout = self._safe_pose_timeout if self._safe_pose_timeout > 0.0 else None
        if not done_event.wait(timeout=timeout):
            self.get_logger().warn(
                f'No trajectory result within {self._safe_pose_timeout:.1f}s — '
                'controller may be unresponsive '
                '(raise the safe_pose_timeout parameter if the sim is just slow).'
            )
            return False
        if not outcome['success']:
            self.get_logger().error(f'Home move failed: {outcome["error"]}')
            return False
        self.get_logger().info('Home reached!')
        return True

    def start_servo(self):
        """Switch to streaming controller, then start MoveIt Servo.

        Waits up to 2 seconds for the service to become available, then
        issues an asynchronous call whose result is handled by
        ``_on_start_result``. Does not block for the call's completion.
        """
        if not self.use_streaming_controller():
            self.get_logger().error(
                'Could not activate forward position controller — Servo not started.'
            )
            return
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servo start service not available')
            return
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_result)

    def _on_start_result(self, future):
        """Log the outcome of an asynchronous ``start_servo`` service call.

        Args:
            future: Future resolving to the ``Trigger.Response`` returned by
                the ``start_servo`` service.
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Servo started successfully')
            else:
                self.get_logger().warn(f'Servo start failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Servo start error: {e}')

    def _on_servo_status(self, msg: Int8):
        """Handle incoming Servo status updates.

        Automatically restarts Servo only when the status transitions into
        SERVO_STATUS_HALT_FOR_SINGULARITY, since that halt is typically
        recoverable by re-issuing start_servo (Servo re-attempts the motion
        away from the singular configuration). Other halt states — e.g.
        SERVO_STATUS_HALT_FOR_COLLISION — are intentionally NOT
        auto-recovered: they represent conditions where continuing motion
        could be unsafe, so they are left for the operator to resolve
        manually (e.g. by moving the arm clear via keyboard input, or a
        workspace/collision object review) rather than being silently
        retried.
        """
        code = msg.data
        if code != self._servo_status:
            if code == SERVO_STATUS_HALT_FOR_SINGULARITY:
                self.start_servo()
        self._servo_status = code

    def _linear_in_command_frame(self):
        """Sum mount-frame and view-frame translation, both in ``linear_frame``.

        The view-frame part (arrow keys) is resolved through TF on every
        publish rather than once at key-press, so "forward" tracks the
        camera as the arm moves.

        Returns:
            tuple[float, float, float]: (vx, vy, vz) in ``linear_frame``.
            If TF is unavailable only the mount-frame part is returned, so
            W/S/A/D/Q/E keep working when the view frame does not resolve.
        """
        if (self.view_vx == 0.0 and self.view_vy == 0.0
                and self.view_vz == 0.0):
            return self.vx, self.vy, self.vz
        if self._view_frame == self._linear_frame:
            return (self.vx + self.view_vx,
                    self.vy + self.view_vy,
                    self.vz + self.view_vz)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame,
                self._view_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self._linear_frame} <- {self._view_frame} unavailable '
                f'({exc}); ignoring view-relative translation',
                throttle_duration_sec=2.0,
            )
            return self.vx, self.vy, self.vz
        rx, ry, rz = _rotate_vector_by_quat(
            transform.transform.rotation,
            self.view_vx, self.view_vy, self.view_vz,
        )
        return self.vx + rx, self.vy + ry, self.vz + rz

    def _angular_in_command_frame(self):
        """Map EEF-frame angular velocity into ``linear_frame`` via TF.

        Returns:
            tuple[float, float, float]: (wx, wy, wz) in ``linear_frame``.
            If TF is unavailable, returns (0, 0, 0).
        """
        if self.wx == 0.0 and self.wy == 0.0 and self.wz == 0.0:
            return 0.0, 0.0, 0.0
        if self._ee_frame == self._linear_frame:
            return self.wx, self.wy, self.wz
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame,
                self._ee_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self._linear_frame} <- {self._ee_frame} unavailable '
                f'({exc}); publishing zero angular command',
                throttle_duration_sec=2.0,
            )
            return 0.0, 0.0, 0.0
        return _rotate_vector_by_quat(
            transform.transform.rotation, self.wx, self.wy, self.wz
        )

    def _orientation_hold(self, wx, wy, wz):
        """Keep TCP attitude while translating (Q/E pitch, also WASD).

        Does not scale linear speed. I/K/U/O/J/L still command rotation.
        """
        # View-relative keys translate just as much as W/S/A/D/Q/E do, so the
        # attitude hold has to engage for them too — otherwise arrow-key moves
        # would be the one path where TCP orientation is left to walk freely.
        driving_lin = (
            abs(self.vx) > HOLD_CMD_EPS or abs(self.vy) > HOLD_CMD_EPS or
            abs(self.vz) > HOLD_CMD_EPS or
            abs(self.view_vx) > HOLD_CMD_EPS or
            abs(self.view_vy) > HOLD_CMD_EPS or
            abs(self.view_vz) > HOLD_CMD_EPS
        )
        driving_ang = (
            abs(self.wx) > HOLD_CMD_EPS or abs(self.wy) > HOLD_CMD_EPS or
            abs(self.wz) > HOLD_CMD_EPS
        )
        if not driving_lin:
            self._hold_quat = None
            return wx, wy, wz
        if driving_ang:
            self._hold_quat = None
            return wx, wy, wz
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame, self._ee_frame, rclpy.time.Time()
            )
        except TransformException:
            return wx, wy, wz
        q = transform.transform.rotation
        quat = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        if self._hold_quat is None:
            self._hold_quat = quat
            return wx, wy, wz
        q_err = _quat_multiply(_quat_conj(quat), self._hold_quat)
        rx, ry, rz = _quat_rotvec(q_err)
        hx, hy, hz = _rotate_vector_by_quat(quat, rx, ry, rz)
        wx = max(-HOLD_ANGULAR_MAX, min(HOLD_ANGULAR_MAX, wx + HOLD_ANGULAR_GAIN * hx))
        wy = max(-HOLD_ANGULAR_MAX, min(HOLD_ANGULAR_MAX, wy + HOLD_ANGULAR_GAIN * hy))
        wz = max(-HOLD_ANGULAR_MAX, min(HOLD_ANGULAR_MAX, wz + HOLD_ANGULAR_GAIN * hz))
        return wx, wy, wz

    def _publish(self):
        """Publish twist in ``linear_frame`` (mount).

        Mount XYZ as-is, view-frame XYZ and EEF ω rotated in via TF.

        Servo ``robot_link_command_frame`` must equal ``linear_frame``.
        Publishing Cartesian velocity in the TCP frame was observed to
        produce almost no joint motion for mount-aligned X; mount-frame
        twists move the EEF correctly. The view-relative keys therefore
        resolve to mount here instead of switching the published frame.
        """
        vx, vy, vz = self._linear_in_command_frame()
        wx, wy, wz = self._angular_in_command_frame()
        wx, wy, wz = self._orientation_hold(wx, wy, wz)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._linear_frame
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self._pub.publish(msg)


HELP = """
╔══════════════════════════════════════════════════╗
║  Keyboard Servo — EEF control                    ║
╠══════════════════════════════════════════════════╣
║  EEF translation (absolute, arm_mount_link):     ║
║    w / s  — +X / -X                              ║
║    a / d  — +Y / -Y                              ║
║    q / e  — +Z / -Z                              ║
║  EEF translation (view-relative, camera):        ║
║    Up/Dn  — forward / back                       ║
║    Lt/Rt  — left / right                         ║
║    t / g  — up / down                            ║
║  EEF rotation (about arm_tcp_link):              ║
║    i / k  — pitch (wx)                           ║
║    u / o  — yaw   (wy)                           ║
║    j / l  — roll  (wz)                           ║
║  Other:                                          ║
║    r      — move to home + start servo           ║
║    ESC/x  — exit                                 ║
╚══════════════════════════════════════════════════╝
"""


class KeyboardInputLoop:
    """Reads raw keyboard events via evdev and drives a ``ServoController``.

    Maintains the set of currently pressed direction keys, recomputes the
    combined Cartesian velocity whenever the set changes, and handles the
    special "safe pose" and "exit" key bindings.
    """

    # (vx, vy, vz, wx, wy, wz, view_vx, view_vy, view_vz)
    #   vx..vz      — arm_mount_link (absolute)
    #   wx..wz      — arm_tcp_link; in camera terms wx=pitch, wy=yaw, wz=roll
    #   view_vx..vz — arm_camera_link, REP-103: +X forward, +Y left, +Z up
    # Both translation sets use the same sign convention, so the arrow block
    # is the mount block re-expressed in the view frame — nothing else moved.
    _DIRECTIONS = {
        ecodes.KEY_W: ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_S: (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_A: ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_D: ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_Q: ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_E: ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_I: ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0),  # pitch
        ecodes.KEY_K: ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_U: ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0),  # yaw
        ecodes.KEY_O: ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
        ecodes.KEY_J: ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0),  # roll
        ecodes.KEY_L: ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
        # View-relative translation (camera / gripper point of view).
        # T/G rather than PgUp/PgDn: compact keyboards (Keychron and friends)
        # only expose the navigation cluster behind an Fn layer. T sits above
        # G, so the pair reads as up/down straight off the key layout, and it
        # leaves the left hand free while the right hand is on the arrows.
        ecodes.KEY_UP:    ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  1.0,  0.0,  0.0),
        ecodes.KEY_DOWN:  ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,  0.0,  0.0),
        ecodes.KEY_LEFT:  ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  1.0,  0.0),
        ecodes.KEY_RIGHT: ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, -1.0,  0.0),
        ecodes.KEY_T:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  1.0),
        ecodes.KEY_G:     ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,  0.0, -1.0),
    }

    _KEYSTATE_UP = 0
    _KEYSTATE_DOWN = 1
    _KEYSTATE_REPEAT = 2

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and initialize input state.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from keyboard input.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._device_path = controller.keyboard_device_path
        self._lock = threading.Lock()
        self._pressed = set()
        self._exit_event = threading.Event()
        self._devices = []
        self._read_thread = None
        self._servo_started = False

    def _open_device(self) -> bool:
        """Open evdev keyboard(s) for teleop.

        ``keyboard_device_path:=auto`` (default) opens every QWERTY-capable
        keyboard and merges events. Pinning only ``/dev/input/event3`` (laptop
        AT keyboard) while typing on a Keychron produced zero key events.

        Returns:
            bool: True if at least one device opened.
        """
        requested = (self._device_path or '').strip()
        if requested and requested.lower() != 'auto':
            paths = [requested]
        else:
            paths = [p for _s, p, _n, _ph in _list_keyboard_candidates()]

        if not paths:
            self._controller.get_logger().error(
                'No suitable keyboard found via evdev. Set keyboard_device_path '
                'to an explicit /dev/input/eventN.'
            )
            return False

        opened = []
        for path in paths:
            try:
                device = evdev.InputDevice(path)
                # Older python-evdev has no set_nonblocking(); use fcntl.
                flag = fcntl.fcntl(device.fd, fcntl.F_GETFL)
                fcntl.fcntl(device.fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)
            except (FileNotFoundError, PermissionError, OSError) as e:
                self._controller.get_logger().warn(f'Skipping {path!r}: {e!r}')
                continue
            opened.append(device)
            self._controller.get_logger().info(f'Listening on {path} ({device.name})')

        if not opened:
            self._controller.get_logger().error(f'Could not open any of {paths!r}')
            return False

        self._devices = opened
        self._device_path = opened[0].path
        print(
            '\nKeyboard input:\n  '
            + '\n  '.join(f'{d.name} ({d.path})' for d in opened)
            + '\nPress r = home + start Servo, then WASD to move.\n'
        )
        return True

    def _recompute_velocity(self):
        """Recompute and apply the combined velocity from all pressed keys.

        Sums the per-axis direction vectors of every currently pressed key
        in ``self._pressed``, scales the result by the controller's linear
        and angular speed settings, and forwards it to
        ``ServoController.set_velocity``.
        """
        vx = vy = vz = wx = wy = wz = 0.0
        cvx = cvy = cvz = 0.0
        with self._lock:
            active = list(self._pressed)
        for code in active:
            d = self._DIRECTIONS.get(code)
            if d is None:
                continue
            vx += d[0]
            vy += d[1]
            vz += d[2]
            wx += d[3]
            wy += d[4]
            wz += d[5]
            cvx += d[6]
            cvy += d[7]
            cvz += d[8]
        self._controller.set_velocity(
            vx * self._linear_speed, vy * self._linear_speed, vz * self._linear_speed,
            wx * self._angular_speed, wy * self._angular_speed, wz * self._angular_speed,
            view_vx=cvx * self._linear_speed,
            view_vy=cvy * self._linear_speed,
            view_vz=cvz * self._linear_speed,
        )

    def _handle_safe_pose(self):
        """Clear pressed keys, stop motion, and move to the safe pose.

        Servo is only started if the safe-pose move actually succeeded;
        starting it after a rejected/aborted/timed-out move would let the
        operator resume Cartesian teleop from a pose that never reached the
        intended safe configuration.

        Intended to run in its own thread (spawned from ``_read_loop``) so
        that the blocking safe-pose and servo-start calls do not stall
        keyboard event processing.
        """
        with self._lock:
            self._pressed.clear()
        self._controller.stop()
        print('Moving to home...')
        if self._controller.move_to_safe_pose():
            print('Starting servo...')
            self._controller.start_servo()
            self._servo_started = True
        else:
            print('Home move failed — Servo not started.')

    def _read_loop(self):
        """Continuously read raw key events from all opened keyboards."""
        try:
            while not self._exit_event.is_set():
                if not self._devices:
                    break
                try:
                    ready, _, _ = select.select(
                        [dev.fd for dev in self._devices], [], [], 0.2
                    )
                except (ValueError, OSError) as e:
                    self._controller.get_logger().error(f'Keyboard select failed: {e!r}')
                    break
                if not ready:
                    continue
                fd_to_dev = {dev.fd: dev for dev in self._devices}
                for fd in ready:
                    device = fd_to_dev.get(fd)
                    if device is None:
                        continue
                    try:
                        for event in device.read():
                            if event.type != ecodes.EV_KEY:
                                continue
                            code, value = event.code, event.value

                            if code in (ecodes.KEY_ESC, ecodes.KEY_X) and value == self._KEYSTATE_DOWN:
                                self._exit_event.set()
                                return

                            if code == ecodes.KEY_R and value == self._KEYSTATE_DOWN:
                                threading.Thread(
                                    target=self._handle_safe_pose, daemon=True
                                ).start()
                                continue

                            if code not in self._DIRECTIONS:
                                continue

                            if not self._servo_started:
                                continue

                            if value == self._KEYSTATE_DOWN:
                                with self._lock:
                                    already_pressed = code in self._pressed
                                    self._pressed.add(code)
                                if not already_pressed:
                                    self._recompute_velocity()
                                    key_name = ecodes.KEY[code].removeprefix('KEY_').lower()
                                    # view_* included or the arrow keys would
                                    # report all-zero and look like a no-op.
                                    print(
                                        f'{key_name} vx={self._controller.vx:.2f} '
                                        f'vy={self._controller.vy:.2f} '
                                        f'vz={self._controller.vz:.2f} '
                                        f'wx={self._controller.wx:.2f} '
                                        f'wy={self._controller.wy:.2f} '
                                        f'wz={self._controller.wz:.2f} '
                                        f'| fwd={self._controller.view_vx:.2f} '
                                        f'left={self._controller.view_vy:.2f} '
                                        f'up={self._controller.view_vz:.2f}'
                                    )
                            elif value == self._KEYSTATE_UP:
                                with self._lock:
                                    self._pressed.discard(code)
                                self._recompute_velocity()
                    except BlockingIOError:
                        continue
                    except OSError as e:
                        self._controller.get_logger().warn(
                            f'Lost keyboard {device.path}: {e!r}'
                        )
                        self._devices = [d for d in self._devices if d.fd != fd]
                        if not self._devices:
                            raise
        except OSError as e:
            self._controller.get_logger().error(f'Keyboard read loop failed: {e!r}')
        finally:
            self._exit_event.set()

    def run(self):
        """Open the keyboard device and run the input loop until exit.

        Opens the evdev device (returning early if this fails), prints the
        help banner, disables terminal echo where possible, starts the
        event-reading loop in a daemon thread, and blocks until the exit
        event is set. On exit, stops the controller's motion and restores
        the original terminal settings.
        """
        if not self._open_device():
            return
        print(HELP, flush=True)
        self._controller.get_logger().info(
            'Keyboard teleop ready. Press r = home + Servo, then WASD. '
            'Do not start a second keyboard_servo_node.'
        )

        # ros2 launch often has no TTY; fileno()/tcgetattr would abort the node.
        old_term_settings = None
        stdin_fd = None
        try:
            stdin_fd = sys.stdin.fileno()
        except (AttributeError, ValueError, OSError):
            stdin_fd = None
        if stdin_fd is not None:
            try:
                old_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings = termios.tcgetattr(stdin_fd)
                new_term_settings[3] &= ~termios.ECHO
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, new_term_settings)
            except (termios.error, OSError):
                old_term_settings = None

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()
            if old_term_settings is not None:
                termios.tcflush(stdin_fd, termios.TCIFLUSH)
                termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)


GAMEPAD_HELP = """
╔══════════════════════════════════════════════╗
║  Gamepad — EEF control (view-relative)       ║
╠══════════════════════════════════════════════╣
║  Left stick   ←→  — left / right  (camera)   ║
║               ↑↓  — forward / back (camera)  ║
║  Right stick  ↑↓  — up / down     (camera)   ║
║               ←→  — yaw   (TCP)              ║
║  R1 + right   ↑↓  — pitch (TCP)              ║
║               ←→  — roll  (TCP)              ║
║  A                — home + start servo       ║
║  X                — exit                     ║
╚══════════════════════════════════════════════╝
"""


class GamepadInputLoop:
    """Reads sensor_msgs/Joy messages and drives a ``ServoController``.

    Replaces the raw-keyboard evdev input of ``KeyboardInputLoop`` with a
    subscription to the ``joy`` package's ``/joy`` topic (published by
    ``ros2 run joy joy_node``) — as this module's docstring already
    promises, ``ServoController`` itself needs no changes.

    Axis/button indices and rest values below were established by
    watching `ros2 topic echo /joy` live with this controller over
    Bluetooth:

    * Axes 0/1 (left stick) and 2/3 (right stick) rest at 0.0, X left =
      +1.0, X right = -1.0, Y forward = +1.0, Y back = -1.0.
    * Axes 4 and 5 (L2 / R2) are often Stadia-style (rest **+1.0**, full
      press **-1.0**), but Bluetooth can leave one trigger resting at
      **0.0** instead. ``_trigger_amount`` uses a per-axis rest sample
      taken while sticks are centered so a 0-rest axis is not treated as
      a half-press (which was publishing a constant phantom yaw).
    * Buttons 0/2 are A/X; 5 is R1/RB, held as a shift for the right stick.
    """

    # Gamepad translation is view-relative (camera frame), not mount-frame —
    # the operator is looking through the camera, so "forward" should follow
    # it. Mount-frame XYZ stays available on the keyboard.
    AXIS_LEFT_X = 0     # view +Y / -Y  (left / right)
    AXIS_LEFT_Y = 1     # view +X / -X  (forward / back)
    AXIS_RIGHT_X = 2    # yaw (-wy)     — roll  (-wz) while R1 held
    AXIS_RIGHT_Y = 3    # view +Z / -Z  — pitch (+wx) while R1 held
    AXIS_L2 = 4         # unmapped; used only for trigger rest calibration
    AXIS_R2 = 5

    BUTTON_SAFE_POSE = 0   # 'A' — move to home + start servo
    BUTTON_EXIT = 2        # 'X' — exit
    BUTTON_LB = 4          # unmapped (settle check only)
    # Shift button has no class constant: its real index is controller-
    # dependent (see DEFAULT_GAMEPAD_SHIFT_BUTTON), read via self._shift_button.

    _DEADZONE = 0.2
    _JOY_TIMEOUT_SEC = 0.2
    _WATCHDOG_PERIOD_SEC = 0.1

    def __init__(self, controller: 'ServoController'):
        """Store a reference to the controller and subscribe to ``/joy``.

        Args:
            controller: The ``ServoController`` node that will receive
                velocity commands derived from gamepad input. Its own
                ``create_subscription`` is reused for the ``/joy`` topic
                so the callback runs on the node's existing executor —
                no separate thread is needed, unlike the keyboard's
                blocking evdev read loop.
        """
        self._controller = controller
        self._linear_speed = controller.linear_speed
        self._angular_speed = controller.angular_speed
        self._shift_button = controller.gamepad_shift_button
        self._exit_event = threading.Event()
        # None means "no trustworthy baseline yet" — the first message
        # after startup or a /joy dropout only seeds this, it never
        # fires a rising-edge action (a gamepad can report a stale
        # "pressed" button on that first message, which was causing
        # spurious exits).
        self._prev_buttons = None
        self._safe_pose_running = threading.Lock()
        self._safe_pose_active = False
        self._prev_cmd = (0.0,) * 6

        # Teleop (axes -> velocity) is locked out until the first
        # safe-pose move (A) completes — Servo hasn't been started yet
        # at that point, so nothing would move anyway, and the arm's
        # real-world pose is unknown to this process until then. This
        # is a one-time latch: once armed it stays armed, including
        # across /joy dropouts (e.g. a Bluetooth reconnect) — there is
        # no re-arm button and no trip back to the fixed safe pose.
        self._last_joy_time = None
        self._joy_silent = False
        self._teleop_locked = True

        # A dropout does not re-lock teleop, but it does set this: the
        # first /joy message(s) after a dropout can report stale values
        # (observed live: full deflection with nothing touched) — likely
        # a freshly re-created Bluetooth HID device reporting a default
        # before its first real report arrives. So we force zero
        # velocity and wait, with no timeout, until one message reports
        # everything centered/released, then resume from wherever the
        # arm already is.
        self._joy_settling = False
        # Per-trigger rest samples (axis index -> float). None until the
        # first centered settle so we do not assume both are +1.0.
        self._trigger_rest = {}

        self._sub = controller.create_subscription(Joy, 'joy', self._on_joy, 10)
        self._watchdog_timer = controller.create_timer(
            self._WATCHDOG_PERIOD_SEC, self._check_joy_timeout
        )

    @classmethod
    def _deadzone(cls, value: float) -> float:
        """Zero out small stick values so resting drift doesn't creep the arm."""
        return 0.0 if abs(value) < cls._DEADZONE else value

    def _axis(self, axes, index: int) -> float:
        """Return ``axes[index]`` with deadzone applied, or 0.0 if out of range."""
        if index >= len(axes):
            return 0.0
        return self._deadzone(axes[index])

    def _calibrate_triggers(self, axes) -> None:
        """Record L2/R2 rest values from a centered Joy snapshot."""
        for index in (self.AXIS_L2, self.AXIS_R2):
            if index < len(axes):
                self._trigger_rest[index] = float(axes[index])
        self._controller.get_logger().info(
            f'Trigger rest L2={self._trigger_rest.get(self.AXIS_L2, float("nan")):.2f} '
            f'R2={self._trigger_rest.get(self.AXIS_R2, float("nan")):.2f}'
        )

    def _sticks_centered(self, axes) -> bool:
        """True when both sticks are inside the deadzone (triggers ignored)."""
        return all(
            self._axis(axes, i) == 0.0
            for i in (self.AXIS_LEFT_X, self.AXIS_LEFT_Y,
                      self.AXIS_RIGHT_X, self.AXIS_RIGHT_Y)
        )

    def _trigger_amount(self, axes, index: int) -> float:
        """Return how far a trigger (L2/R2) is pressed: 0.0 .. 1.0.

        Supports both common rest conventions on this Stadia over ``joy_node``:
        rest near +1 (press toward -1) and rest near 0 (press toward ±1).
        Before calibration, only the +1-rest formula is used, and a raw
        value near 0 is treated as released — otherwise an uncalibrated
        0-rest R2 looks like a constant half-press (wz ≈ 0.5 * angular).
        """
        if index >= len(axes):
            return 0.0
        raw = float(axes[index])
        rest = self._trigger_rest.get(index)

        if rest is None:
            # Uncalibrated: never treat raw≈0 as a half-press (0-rest R2
            # phantom). Full +1-rest presses still register via raw≤-0.5.
            if raw >= 0.5 or raw <= -0.5:
                amount = (1.0 - raw) / 2.0
            else:
                amount = 0.0
        elif rest > 0.5:
            # Classic: +1 released → -1 fully pressed.
            amount = (rest - raw) / (rest - (-1.0))
        else:
            # Rest near 0: any deflection toward ±1 is a press.
            amount = abs(raw - rest)

        if amount < 0.0:
            amount = 0.0
        elif amount > 1.0:
            amount = 1.0
        return 0.0 if amount < self._DEADZONE else amount

    def _button_pressed(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` is currently held down."""
        return index < len(buttons) and buttons[index] == 1

    def _button_rising_edge(self, buttons, index: int) -> bool:
        """Return True if ``buttons[index]`` was just pressed this message.

        Requires a real previous reading — see ``_prev_buttons`` in
        ``__init__`` for why ``None`` (no baseline yet) always reports
        no edge rather than comparing against an assumed all-zero state.
        """
        if self._prev_buttons is None:
            return False
        was_pressed = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        return self._button_pressed(buttons, index) and not was_pressed

    @staticmethod
    def _active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift: bool) -> str:
        """Describe which physical control(s) are driving a nonzero command."""
        # wy (yaw) only ever comes from the plain right stick, wx/wz (pitch/
        # roll) only from the shifted one — so the axes identify the source.
        parts = []
        if view_vx or view_vy:
            parts.append('left stick')
        if view_vz or wy:
            parts.append('right stick')
        if wx or wz:
            parts.append('R1+right stick')
        return '+'.join(parts) if parts else ('R1' if shift else 'idle')

    def _check_joy_timeout(self):
        """Stop the arm if no ``/joy`` message has arrived recently.

        Runs on the node's own timer, independent of message arrival, so
        a disconnected controller or a dead ``joy_node`` can't
        leave the last commanded velocity republishing forever — Servo's
        own command timeout does not help here because ServoController
        keeps re-publishing that last twist every tick regardless of
        whether new input has arrived. This only zeroes the current
        command; it does not lock teleop out, so control resumes on its
        own the moment ``/joy`` messages start arriving again (e.g. a
        Bluetooth reconnect) — no separate re-arm step.
        """
        if self._last_joy_time is None:
            return
        elapsed = (self._controller.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if elapsed > self._JOY_TIMEOUT_SEC:
            if not self._joy_silent:
                self._joy_silent = True
                self._joy_settling = True
                self._trigger_rest.clear()
                self._prev_buttons = None
                self._controller.get_logger().warn(
                    f'/joy timed out after {elapsed:.2f}s — stopping arm.'
                )
            self._controller.stop()

    def _log_raw_joy(self, axes, buttons, note: str = ''):
        """Log the full Joy message — an out-of-range index fails silently otherwise."""
        axes_str = ', '.join(f'{i}:{v:+.2f}' for i, v in enumerate(axes))
        buttons_str = ', '.join(f'{i}:{b}' for i, b in enumerate(buttons))
        self._controller.get_logger().info(
            f'/joy raw{note} — axes[{len(axes)}]: {{{axes_str}}}  '
            f'buttons[{len(buttons)}]: {{{buttons_str}}}'
        )

    def _on_joy(self, msg: Joy):
        """Translate one Joy snapshot into a velocity command and edge-triggered actions."""
        axes = msg.axes
        buttons = msg.buttons

        if self._joy_silent:
            self._joy_silent = False
            self._controller.get_logger().info('/joy resumed.')
        self._last_joy_time = self._controller.get_clock().now()

        if self._prev_buttons is None:
            # First message (also re-fires after a /joy dropout). Warn early
            # if gamepad_shift_button is out of range for this controller.
            self._log_raw_joy(axes, buttons, ' (first message)')
            if self._shift_button >= len(buttons):
                self._controller.get_logger().warn(
                    f'gamepad_shift_button={self._shift_button} but this '
                    f'/joy only reports {len(buttons)} button(s) '
                    f'(0..{len(buttons) - 1}) — that index can never be '
                    f'pressed. Pick a real index from the buttons[] list above.'
                )

        # Button edges are always processed, even while teleop is locked
        # out — buttons are digital (no analog drift), and the safe-pose
        # button is the only way to clear the lock, so it must keep
        # working while locked or the arm could never be armed at all.
        safe_pose_pressed = self._button_rising_edge(buttons, self.BUTTON_SAFE_POSE)
        exit_pressed = self._button_rising_edge(buttons, self.BUTTON_EXIT)

        # Log the raw state on any button change, not just the mapped ones,
        # so an unmapped shift button still shows up.
        if self._prev_buttons is not None:
            width = max(len(buttons), len(self._prev_buttons))
            changed = [
                i for i in range(width)
                if self._button_pressed(buttons, i)
                != (i < len(self._prev_buttons) and self._prev_buttons[i] == 1)
            ]
            if changed:
                self._log_raw_joy(
                    axes, buttons,
                    f' (button(s) {changed} changed; shift configured as '
                    f'{self._shift_button})',
                )

        self._prev_buttons = list(buttons)

        if exit_pressed:
            self._exit_event.set()

        if safe_pose_pressed:
            threading.Thread(target=self._handle_safe_pose, daemon=True).start()

        if self._teleop_locked or self._safe_pose_active:
            self._controller.stop()
            return

        if self._joy_settling:
            centered = (
                self._sticks_centered(axes)
                and self._trigger_amount(axes, self.AXIS_L2) == 0.0
                and self._trigger_amount(axes, self.AXIS_R2) == 0.0
                and not self._button_pressed(buttons, self.BUTTON_LB)
                and not self._button_pressed(buttons, self._shift_button)
            )
            self._controller.stop()
            if centered:
                self._calibrate_triggers(axes)
                self._joy_settling = False
                self._controller.get_logger().info('Sticks centered — resuming control.')
            return

        if not self._trigger_rest and self._sticks_centered(axes):
            self._calibrate_triggers(axes)

        # Left stick — view-relative translation in the horizontal plane.
        # Stick sign convention (see class docstring): left = +1, forward = +1;
        # camera frame is REP-103 (+X forward, +Y left), so both pass straight
        # through with no flip.
        view_vy = self._axis(axes, self.AXIS_LEFT_X) * self._linear_speed
        view_vx = self._axis(axes, self.AXIS_LEFT_Y) * self._linear_speed

        # Right stick — two modes, mutually exclusive so a held R1 can never
        # translate and rotate at once:
        #   plain:   up/down = view +Z/-Z, left/right = yaw
        #   R1 held: up/down = pitch,      left/right = roll
        # Rotation stays about the TCP axes, same as the keyboard's I/K/U/O/J/L.
        right_x = self._axis(axes, self.AXIS_RIGHT_X)
        right_y = self._axis(axes, self.AXIS_RIGHT_Y)
        shift = self._button_pressed(buttons, self._shift_button)

        view_vz = 0.0
        wx = wy = wz = 0.0
        if shift:
            wx = right_y * self._angular_speed          # pitch
            wz = -right_x * self._angular_speed         # roll
        else:
            view_vz = -right_y * self._linear_speed      # stick up = view +Z
            wy = -right_x * self._angular_speed         # yaw

        # Mount-frame translation is unused here — every gamepad axis is
        # view-relative or a rotation.
        self._controller.set_velocity(
            0.0, 0.0, 0.0, wx, wy, wz,
            view_vx=view_vx, view_vy=view_vy, view_vz=view_vz,
        )

        cmd = (view_vx, view_vy, view_vz, wx, wy, wz)
        if cmd != self._prev_cmd and any(c != 0.0 for c in cmd):
            label = self._active_label(view_vx, view_vy, view_vz, wx, wy, wz, shift)
            print(f'{label} fwd={view_vx:.2f} left={view_vy:.2f} up={view_vz:.2f} '
                  f'wx={wx:.2f} wy={wy:.2f} wz={wz:.2f}')
        self._prev_cmd = cmd

    def _handle_safe_pose(self):
        """Stop motion and move to the safe pose (mirrors KeyboardInputLoop's 'r').

        Guarded by a non-blocking lock so a second button press while a
        move is already in progress is ignored instead of racing a
        redundant safe-pose goal against the first. Also sets
        ``_safe_pose_active`` for the duration so ``_on_joy`` ignores
        stick/trigger input while it's set — otherwise a stick held
        during the move would resume Cartesian motion the instant
        ``start_servo()`` re-enables Servo, before the operator has a
        chance to let go.

        This is also the only place ``_teleop_locked`` is cleared (see
        its declaration in ``__init__``).
        """
        if not self._safe_pose_running.acquire(blocking=False):
            return
        self._safe_pose_active = True
        try:
            self._controller.stop()
            print('Moving to home...')
            if self._controller.move_to_safe_pose():
                print('Starting servo...')
                self._controller.start_servo()
                self._teleop_locked = False
                self._controller.get_logger().info('Teleop enabled.')
            else:
                print('Home move failed — Servo not started.')
        finally:
            self._safe_pose_active = False
            self._safe_pose_running.release()

    def run(self):
        """Print the help banner and block until the exit button is pressed."""
        print(GAMEPAD_HELP)
        try:
            self._exit_event.wait()
        finally:
            print('\nExiting...')
            self._controller.stop()


def _run_teleop(controller: 'ServoController', input_loop) -> None:
    """Spin ``controller`` in a background thread and run ``input_loop`` until exit.

    Shared by ``main`` (keyboard) and ``main_gamepad`` (gamepad): both
    input loops expose the same ``run()`` contract (block until an exit
    condition, leave the controller stopped), so the ROS lifecycle
    around them doesn't need to be duplicated per input source.
    """
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    try:
        input_loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        if not controller.stop_servo():
            controller.get_logger().warn(
                'Could not confirm Servo stopped before shutdown — '
                'it may still be active.'
            )
        controller.destroy_node()
        rclpy.shutdown()


def main():
    """Entry point: initialize ROS2, run the keyboard input loop, and clean up.

    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, KeyboardInputLoop(controller))


def main_gamepad():
    """Entry point: initialize ROS2, run the gamepad input loop, and clean up.

    Requires a running ``joy`` publisher (``ros2 run joy joy_node``).
    See ``_run_teleop`` for the shared spin/cleanup lifecycle.
    """
    rclpy.init()
    controller = ServoController()
    _run_teleop(controller, GamepadInputLoop(controller))


if __name__ == '__main__':
    main()