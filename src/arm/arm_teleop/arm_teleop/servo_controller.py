"""Shared Cartesian-teleop engine behind the arm's keyboard and gamepad
input front-ends.

``ServoController`` turns velocity/pose commands into MoveIt Servo
twists, ``FollowJointTrajectory`` goals, and ``MoveGroup`` plans: home-
pose motion, collision-checked "level tool", panel-align delegation
(see ``arm_tasks/panel_align_node.py``), gripper position integration,
and the ERC 2026 activity-indicator gating (``run_planned_activity``).

This module has no input source of its own — see ``keyboard_input.py``/
``keyboard_teleop_node.py`` and ``gamepad_input.py``/
``gamepad_teleop_node.py`` for the two front-ends that drive it.
"""

import os
import socket
import threading
import time
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, Float64MultiArray, ColorRGBA
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers, SwitchController
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from indomitus_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock

from arm_teleop.arm_motion_lock import ArmMotionBusy, arm_motion_lock


DEFAULT_LINEAR_SPEED  = 0.6
DEFAULT_ANGULAR_SPEED = 1.8
DEFAULT_PUBLISH_RATE  = 100.0
# Q/E (±Z) folds the shoulder/elbow; without ω the TCP pitch walks.
# Hold attitude only — do not scale XYZ (that made teleop feel slow).
HOLD_ANGULAR_GAIN = 6.0
HOLD_ANGULAR_MAX = 0.8
HOLD_CMD_EPS = 1e-4
# _level_hold target (sampling + drill modes): world-down, roll unconstrained.
DOWN_AXIS = (0.0, 0.0, -1.0)

SAMPLING_POINT_AXIS = (0.0, 0.0, 1.0)
DRILL_POINT_AXIS = (0.0, -1.0, 0.0)
# Untested on real hardware yet — B/Y (sampling_home/drill_home) and the
# level button log and do nothing while False.
SAMPLING_DRILL_MODES_ENABLED = True

LEVEL_TOOL_TARGET_POSE = [
    -0.06367867092618458,  # arm_mount_base_joint
    0.8861027135101929,    # arm_base_shoulder_joint
    -0.36962295375967424,  # arm_shoulder_forearm_joint
    0.26862548294026134,   # arm_forearm_wrist_1_joint
    2.125629420127624,     # arm_wrist_1_wrist_2_joint
    -1.0466698958262262,   # arm_wrist_2_end_effector_joint
]

DEFAULT_LINEAR_FRAME  = 'arm_mount_link'
DEFAULT_EE_FRAME      = 'arm_tcp_link'

DEFAULT_VIEW_FRAME    = 'arm_camera_link'
# Fallback if poses.json "home" cannot be loaded (matches SRDF group_state home).
DEFAULT_HOME_POSE     = [-1.552, 0.5057, 1.1731, 0.717, 0.0093, -1.536]
# 'auto' picks a USB/external keyboard (Keychron, etc.) over the laptop's
# built-in AT Translated Set 2 device — the usual failure mode in Docker.
DEFAULT_KEYBOARD_DEVICE_PATH = 'auto'
DEFAULT_GAMEPAD_SHIFT_BUTTON = 10
DEFAULT_SAFE_POSE_TIMEOUT = 60.0

DEFAULT_GRIPPER_SPEED = 0.006   # m/s
DEFAULT_GRIPPER_STROKE = 0.012  # m — matches finger_stroke in arm_macro.xacro

GRIPPER_JOINT_NAME = 'arm_jaw_gripper_finger_right_joint'

DEFAULT_PANEL_POSE_TOPIC = '/panel_pose'
DEFAULT_PANEL_VISIBLE_MAX_AGE_SEC = 3.0
DEFAULT_PANEL_ALIGN_TIMEOUT = 120.0

ACTIVITY_INDICATOR_PRE_DELAY_SEC = 5.0
DEFAULT_ACTIVITY_INDICATOR_TOPIC = 'activity_indicator'
ACTIVITY_INDICATOR_COLOR_ACTIVE = (0.0, 0.0, 1.0, 1.0)  # blue, a=1 (lit)
ACTIVITY_INDICATOR_COLOR_IDLE = (0.0, 0.0, 0.0, 0.0)    # off

JTC_CONTROLLER_NAME = 'indomitus_arm_controller'
FORWARD_CONTROLLER_NAME = 'indomitus_arm_forward_position_controller'
MOVEIT_GROUP_NAME = 'indomitus_arm'

HOME_POSE_JOINTS = [
    'arm_mount_base_joint',
    'arm_base_shoulder_joint',
    'arm_shoulder_forearm_joint',
    'arm_forearm_wrist_1_joint',
    'arm_wrist_1_wrist_2_joint',
    'arm_wrist_2_end_effector_joint',
]
# Same values as arm_macro.xacro's <limit> tags — duplicated (like
# panel_align_node.py's own JOINT_LIMITS) so a bad poses.json entry can
# be caught here, before it's ever sent as a home/mode-engage target.
HOME_POSE_JOINT_LIMITS = {
    'arm_mount_base_joint': (-2 * math.pi, 2 * math.pi),
    'arm_base_shoulder_joint': (-2 * math.pi, 2 * math.pi),
    'arm_shoulder_forearm_joint': (-2 * math.pi, 2 * math.pi),
    'arm_forearm_wrist_1_joint': (-2 * math.pi, 2 * math.pi),
    'arm_wrist_1_wrist_2_joint': (-2 * math.pi, 2 * math.pi),
    'arm_wrist_2_end_effector_joint': (-2 * math.pi, 2 * math.pi),
}


def _pose_limit_violation(pose: list) -> str:
    """Empty string if every joint in ``pose`` (HOME_POSE_JOINTS order) is
    within HOME_POSE_JOINT_LIMITS, else a description of the first one
    that isn't."""
    for name, value in zip(HOME_POSE_JOINTS, pose):
        lo, hi = HOME_POSE_JOINT_LIMITS[name]
        if not (lo <= value <= hi):
            return f'{name}={value:.4f} outside [{lo:.4f}, {hi:.4f}]'
    return ''


def _load_home_pose_from_json(pose_name='home'):
    """Return ``pose_name`` joint positions from poses.json, or None if unavailable."""
    candidates = [
        Path('/opt/ws/src/arm/arm_teleop/poses.json'),
        Path(__file__).resolve().parent.parent / 'poses.json',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('arm_teleop')) / 'poses.json'
        candidates.insert(0, share)
    except Exception:
        pass

    for path in candidates:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text())
            pose = data.get(pose_name) or {}
            return [float(pose[name]) for name in HOME_POSE_JOINTS]
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


# Must mirror moveit_servo::StatusCode (status_codes.h) exactly — a prior
# version of this table was off by one, mislabeling JOINT_BOUND as HALT_FOR_COLLISION.
SERVO_STATUS_INVALID                              = -1
SERVO_STATUS_OK                                    = 0
SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY = 1
SERVO_STATUS_HALT_FOR_SINGULARITY                  = 2
SERVO_STATUS_DECELERATE_FOR_COLLISION              = 3
SERVO_STATUS_HALT_FOR_COLLISION                    = 4
SERVO_STATUS_JOINT_BOUND                           = 5
SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY    = 6

SERVO_STATUS_NAMES = {
    SERVO_STATUS_INVALID: 'INVALID',
    SERVO_STATUS_OK: 'NO_WARNING',
    SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY: 'DECELERATE_FOR_APPROACHING_SINGULARITY',
    SERVO_STATUS_HALT_FOR_SINGULARITY: 'HALT_FOR_SINGULARITY',
    SERVO_STATUS_DECELERATE_FOR_COLLISION: 'DECELERATE_FOR_COLLISION',
    SERVO_STATUS_HALT_FOR_COLLISION: 'HALT_FOR_COLLISION',
    SERVO_STATUS_JOINT_BOUND: 'JOINT_BOUND',
    SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY: 'DECELERATE_FOR_LEAVING_SINGULARITY',
}



class ServoController(Node):
    """ROS2 node that turns Cartesian velocity commands into MoveIt Servo messages.

    Declares and reads ROS parameters for speed, publish rate, command frame,
    safe pose and keyboard device path, publishes ``TwistStamped`` messages on
    a timer, and exposes helper methods to start/stop Servo and to drive the
    arm to a predefined safe pose via a ``FollowJointTrajectory`` action.
    """

    def __init__(self, node_name='servo_controller'):
        """Initialize the node, declare parameters, and set up pub/sub/clients.

        Declares all ROS parameters (with defaults), reads their resolved
        values into instance attributes, zeroes the internal velocity state,
        creates the twist publisher, the Servo start/stop service clients,
        the trajectory action client, the publish timer, and the servo
        status subscription.
        """

        super().__init__(node_name)

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
        self.declare_parameter('gripper_speed', DEFAULT_GRIPPER_SPEED)
        self.declare_parameter('gripper_stroke', DEFAULT_GRIPPER_STROKE)
        self.declare_parameter('end_effector', 'jaw')
        self.declare_parameter('panel_pose_topic', DEFAULT_PANEL_POSE_TOPIC)
        self.declare_parameter('panel_visible_max_age_sec', DEFAULT_PANEL_VISIBLE_MAX_AGE_SEC)
        self.declare_parameter('panel_align_timeout', DEFAULT_PANEL_ALIGN_TIMEOUT)
        self.declare_parameter('activity_indicator_topic', DEFAULT_ACTIVITY_INDICATOR_TOPIC)

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
        pose_from_json = _load_home_pose_from_json(self._home_pose_name)
        if pose_from_json is not None:
            violation = _pose_limit_violation(pose_from_json)
            if violation:
                self.get_logger().error(
                    f'poses.json["{self._home_pose_name}"] {violation} — ignoring it.'
                )
                pose_from_json = None
        if pose_from_param == list(DEFAULT_HOME_POSE) and pose_from_json is not None:
            self._safe_pose = pose_from_json
            pose_source = f'poses.json["{self._home_pose_name}"]'
        else:
            self._safe_pose = pose_from_param
            pose_source = 'safe_pose parameter'
        # Sampling/drill modes' own A/R targets; each falls back to the jaw
        # home pose until poses.json has a real entry.
        self._sampling_home_pose_name = 'sampling_home'
        self._sampling_home_pose = self._load_tool_home_pose(self._sampling_home_pose_name)
        self._drill_home_pose_name = 'drill_home'
        self._drill_home_pose = self._load_tool_home_pose(self._drill_home_pose_name)
        # Reachable only from sampling_home/drill_home respectively (see
        # GamepadInputLoop's Y handler in drill_sampling mode) — not
        # mode-engage targets themselves, just sub-positions within the
        # current sampling/drill context.
        self._sampling_container_pose_name = 'sampling_container'
        self._sampling_container_pose = self._load_tool_home_pose(self._sampling_container_pose_name)
        self._drill_container_pose_name = 'drill_container'
        self._drill_container_pose = self._load_tool_home_pose(self._drill_container_pose_name)
        self._keyboard_device_path = self.get_parameter('keyboard_device_path').value
        self._gamepad_shift_button = int(self.get_parameter('gamepad_shift_button').value)
        self._safe_pose_timeout    = self.get_parameter('safe_pose_timeout').value
        self._gripper_speed        = self.get_parameter('gripper_speed').value
        self._gripper_stroke       = self.get_parameter('gripper_stroke').value
        self._end_effector         = self.get_parameter('end_effector').value
        self._panel_pose_topic          = self.get_parameter('panel_pose_topic').value
        self._panel_visible_max_age_sec = self.get_parameter('panel_visible_max_age_sec').value
        self._panel_align_timeout       = self.get_parameter('panel_align_timeout').value
        self._activity_indicator_topic  = self.get_parameter('activity_indicator_topic').value

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
        # Scales HOLD_ANGULAR_GAIN/MAX in _orientation_hold for boosted push (set_velocity's hold_boost).
        self._hold_boost = 1.0
        # Selects _level_hold over _orientation_hold in _publish(); set via
        # set_sampling_mode()/set_drill_mode(). Mutually exclusive (see those).
        self._sampling_mode = False
        self._drill_mode = False
        self._pitch_yaw_locked = False
        # Set for the duration of run_planned_activity()'s 5s pre-delay —
        # set_velocity()/set_gripper_velocity() force zero while this is
        # True, regardless of what's requested, so held teleop input can't
        # move the arm during the ERC-mandated stationary window.
        self._activity_delay_active = False
        self._joint_positions = {}

        self.gripper_vel = 0.0
        # Guess (closed) until the first /joint_states reading syncs this —
        # see _on_joint_state. Avoids commanding a jump from a wrong assumed
        # position on startup if the gripper wasn't actually closed.
        self._gripper_position = 0.0
        self._gripper_state_received = False
        self._last_gripper_tick_time = None

        self._panel_align_succeeded_once = False
        self._last_panel_visible_time = None
        self._motion_lock = threading.Lock()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Servo subscribes BEST_EFFORT; default RELIABLE can drop twists.
        self._pub = self.create_publisher(
            TwistStamped, 'servo_node/delta_twist_cmds', qos_profile_sensor_data
        )
        self._gripper_right_pub = self.create_publisher(
            Float64MultiArray, 'gripper_right_controller/commands', 10
        )
        self._gripper_left_pub = self.create_publisher(
            Float64MultiArray, 'gripper_left_controller/commands', 10
        )
        self._start_client = self.create_client(Trigger, 'servo_node/start_servo')
        self._stop_client  = self.create_client(Trigger, 'servo_node/stop_servo')
        self._panel_align_client = self.create_client(Trigger, 'panel_align/align')
        self._orient_gripper_client = self.create_client(Trigger, 'panel_align/orient_gripper')
        self._switch_client = self.create_client(
            SwitchController, 'controller_manager/switch_controller'
        )
        self._list_controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers'
        )
        # Home/mode-engage moves plan through this (OMPL, collision-checked
        # against the live planning scene) instead of a raw FollowJointTrajectory
        # goal — see _move_to_joint_positions_locked()/level_tool().
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._acquire_lock_client = self.create_client(
            AcquireArmMotionLock, 'arm_motion_lock/acquire')
        self._release_lock_client = self.create_client(
            ReleaseArmMotionLock, 'arm_motion_lock/release')
        # Identifies THIS process to arm_motion_lock_server — hostname
        # covers the actual cross-host case (GS vs Jetson), pid separates
        # two runs on the same host (e.g. sim + a stray leftover process).
        self._motion_lock_holder_id = f'{socket.gethostname()}/{node_name}/{os.getpid()}'
        self._js_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 10
        )
        self._panel_pose_sub = self.create_subscription(
            PoseStamped, self._panel_pose_topic, self._on_panel_pose, 10
        )
        # See ACTIVITY_INDICATOR_PRE_DELAY_SEC's comment (REQ-OPS-080/090/100)
        # — publishes activity-indicator INTENT; nothing in this repo drives
        # a physical lamp off it yet.
        self._activity_indicator_pub = self.create_publisher(
            ColorRGBA, self._activity_indicator_topic, 10
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
        self.get_logger().warn(
            'Servo runs with check_collisions=true (self/scene proximity '
            'thresholds 0.003/0.005 m, see servo.yaml) — teleop WILL decelerate '
            'near a modeled collision, watch for "Close to a collision, '
            'decelerating" in servo_node\'s own log. Singularity deceleration is '
            'still effectively disabled (lower_singularity_threshold=10000.0).'
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
    def gripper_speed(self) -> float:
        """Return the configured gripper speed, in meters per second."""
        return self._gripper_speed

    @property
    def gripper_stroke(self) -> float:
        """Return the configured gripper stroke (fully-open finger position, in meters)."""
        return self._gripper_stroke

    @property
    def keyboard_device_path(self) -> str:
        """Return the filesystem path of the keyboard input device (evdev)."""
        return self._keyboard_device_path

    @property
    def gamepad_shift_button(self) -> int:
        """Return the Joy button index that shifts the right stick."""
        return self._gamepad_shift_button

    @property
    def sampling_home_pose(self):
        """Return the joint targets A/R drives to while sampling mode is armed."""
        return self._sampling_home_pose

    @property
    def sampling_home_pose_name(self) -> str:
        """Return the poses.json key sampling_home_pose came from (for logging)."""
        return self._sampling_home_pose_name

    @property
    def drill_home_pose(self):
        """Return the joint targets A/R drives to while drill mode is armed."""
        return self._drill_home_pose

    @property
    def drill_home_pose_name(self) -> str:
        """Return the poses.json key drill_home_pose came from (for logging)."""
        return self._drill_home_pose_name

    @property
    def sampling_container_pose(self):
        """Return the joint targets Y drives to from sampling_home (drill_sampling mode)."""
        return self._sampling_container_pose

    @property
    def sampling_container_pose_name(self) -> str:
        """Return the poses.json key sampling_container_pose came from (for logging)."""
        return self._sampling_container_pose_name

    @property
    def drill_container_pose(self):
        """Return the joint targets Y drives to from drill_home (drill_sampling mode)."""
        return self._drill_container_pose

    @property
    def drill_container_pose_name(self) -> str:
        """Return the poses.json key drill_container_pose came from (for logging)."""
        return self._drill_container_pose_name

    @property
    def end_effector(self) -> str:
        """Return the 'end_effector' parameter (which tool is mounted)."""
        return self._end_effector

    def _load_tool_home_pose(self, pose_name: str):
        """Load poses.json[pose_name], falling back to the jaw safe pose with a warning."""
        pose = _load_home_pose_from_json(pose_name)
        if pose is not None:
            violation = _pose_limit_violation(pose)
            if violation:
                self.get_logger().error(
                    f'poses.json["{pose_name}"] {violation} — refusing to use it '
                    'as a home/mode-engage target; re-teach it. Falling back to '
                    'the jaw home pose until then.'
                )
                pose = None
        if pose is not None:
            return pose
        self.get_logger().warn(
            f'No valid "{pose_name}" entry in poses.json — its mode\'s A/R '
            'will use the jaw home pose until one is added.'
        )
        return self._safe_pose

    @property
    def has_remembered_panel_position(self) -> bool:
        """True once panel_align/align has succeeded at least once this session.

        panel_align_node owns the actual remembered joint target (and
        replans a fresh, collision-checked path to it every call) — this
        is just the local echo of "does one exist", for prompt/gating
        purposes only.
        """
        return self._panel_align_succeeded_once

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                     wx=0.0, wy=0.0, wz=0.0,
                     view_vx=0.0, view_vy=0.0, view_vz=0.0,
                     hold_boost=1.0):
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

            The three trailing arguments (plus hold_boost) are keyword-
            friendly on purpose: ``GamepadInputLoop`` (gamepad_input.py)
            calls this with six positional values and must keep working
            unchanged.

            hold_boost: Multiplier applied to _orientation_hold's gain/cap
                for this cycle. Pass the same multiplier used to scale the
                push (e.g. a held push-boost button) so the wrist's
                resistance to being bent by reaction torque grows with the
                push strength instead of staying fixed while the push
                triples.
        """
        if self._activity_delay_active:
            # ERC-mandated stationary window (see run_planned_activity) —
            # every component forced to zero regardless of what's asked.
            vx = vy = vz = wx = wy = wz = view_vx = view_vy = view_vz = 0.0
        self.vx = vx
        self.vy = vy
        self.vz = vz
        # pitch/yaw stay locked out after level_tool() until move_to_safe_pose()
        # clears it (see _pitch_yaw_locked's own comment) — roll (wz) and
        # translation are unaffected.
        self.wx = 0.0 if self._pitch_yaw_locked else wx
        self.wy = 0.0 if self._pitch_yaw_locked else wy
        self.wz = wz
        self.view_vx = view_vx
        self.view_vy = view_vy
        self.view_vz = view_vz
        self._hold_boost = hold_boost

    def set_gripper_velocity(self, vel: float):
        """Set the current gripper velocity command, in meters per second.

        Positive opens (toward gripper_stroke), negative closes (toward 0,
        the touching/closed position set by finger_x_closed in the URDF).
        """
        self.gripper_vel = 0.0 if self._activity_delay_active else vel

    def set_gripper_target(self, position: float):
        """Jump the visualized gripper straight to ``position`` (meters).

        For one-shot commands (gamepad SAFE_OPEN/SAFE_CLOSE) instead of
        the velocity-integrated path set_gripper_velocity() drives.
        Clamped and picked up by the very next _publish_gripper() tick,
        same as any other change to _gripper_position — including that
        method's own "withheld until the first real /joint_states sync"
        guard, so a press before that arrives doesn't slam a stale
        default onto gripper_right/left_controller.
        """
        self._gripper_position = max(0.0, min(self._gripper_stroke, position))

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = float(pos)
        # Runs only until the first message that names GRIPPER_JOINT_NAME —
        # after that, _gripper_position is our own commanded state and the
        # real joint may legitimately lag behind it while moving.
        if not self._gripper_state_received and GRIPPER_JOINT_NAME in msg.name:
            index = msg.name.index(GRIPPER_JOINT_NAME)
            self._gripper_position = msg.position[index]
            self._gripper_state_received = True

    def stop(self):
        """Zero out all velocity components, halting Cartesian and gripper motion.

        Equivalent to calling ``set_velocity()`` with no arguments plus
        ``set_gripper_velocity(0.0)``. The publish timer keeps running on
        exit (it's spun on its own thread until destroy_node()), so every
        exit/stop path — ESC, X, a lost keyboard device, a /joy timeout —
        must go through this to also stop the gripper, not just the arm.
        """
        self.set_velocity()
        self.set_gripper_velocity(0.0)
        self._hold_quat = None

    def set_sampling_mode(self, active: bool):
        """Arm/disarm sampling mode; drops _hold_quat and disarms drill mode (mutually exclusive)."""
        self._sampling_mode = active
        if active:
            self._drill_mode = False
        self._hold_quat = None

    def set_drill_mode(self, active: bool):
        """Arm/disarm drill mode; drops _hold_quat and disarms sampling mode (mutually exclusive)."""
        self._drill_mode = active
        if active:
            self._sampling_mode = False
        self._hold_quat = None

    def _controller_states(self) -> dict:
        """Return {controller_name: state} via list_controllers, or {} on failure.

        STRICT switching errors on a controller already in its requested
        state (already active / already inactive), so callers use this to
        drop no-op entries before asking to switch.
        """
        if not self._list_controllers_client.wait_for_service(timeout_sec=2.0):
            return {}
        done_event = threading.Event()
        states = {}

        def _cb(future):
            try:
                for c in future.result().controller:
                    states[c.name] = c.state
            except Exception as exc:
                self.get_logger().error(f'list_controllers exception: {exc!r}')
            finally:
                done_event.set()

        future = self._list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(_cb)
        done_event.wait(timeout=3.0)
        return states

    def _switch_controllers(self, activate, deactivate) -> bool:
        """Activate/deactivate ros2_control controllers (JTC <-> forward).

        Args:
            activate: Controllers to activate.
            deactivate: Controllers to deactivate.

        Returns:
            True if the switch service reported success, or if every
            controller was already in its requested state.
        """
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('controller_manager/switch_controller unavailable')
            return False

        states = self._controller_states()
        if states:
            activate = [c for c in activate if states.get(c) != 'active']
            deactivate = [c for c in deactivate if states.get(c) == 'active']
            if not activate and not deactivate:
                return True
        # If list_controllers itself failed, fall through with the original,
        # unfiltered lists rather than silently dropping deactivate targets.

        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
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

    def _signal_activity_indicator(self, active: bool) -> None:
        """Publish the activity-indicator colour (best-effort, never raises).

        ``active`` picks ACTIVITY_INDICATOR_COLOR_ACTIVE (blue) or
        ACTIVITY_INDICATOR_COLOR_IDLE (off) — see that pair's own comment
        for why blue and for the caveat that nothing yet turns this into
        light on real hardware.
        """
        r, g, b, a = ACTIVITY_INDICATOR_COLOR_ACTIVE if active else ACTIVITY_INDICATOR_COLOR_IDLE
        msg = ColorRGBA(r=r, g=g, b=b, a=a)
        self._activity_indicator_pub.publish(msg)

    def run_planned_activity(self, action, label: str):
        """Run ``action`` (a zero-arg callable) gated by the ERC-mandated
        activity-indicator warm-up — the single choke point 'r'/'p'/'f'/'m'
        and their gamepad equivalents all route their actual move/align/
        level/orient call through, so REQ-OPS-080/090/100 are satisfied
        exactly once instead of separately at each call site.

        Sequence:
          1. stop() plus _activity_delay_active=True — set_velocity() and
             set_gripper_velocity() force zero while this is set, so held
             stick/key input during the wait can't sneak a command through
             (review-flagged: the sleep alone did not guarantee this).
          2. Publish the indicator ON (blue).
          3. Sleep ACTIVITY_INDICATOR_PRE_DELAY_SEC (5s) — REQ-OPS-090.
          4. Clear _activity_delay_active, then call ``action()`` —
             REQ-OPS-100's "at least 5s after the command was issued" —
             and return whatever it returns.
          5. Publish the indicator OFF once ``action()`` returns, success or
             failure alike (``finally``) — REQ-OPS-080's "continue to emit
             ... until all rover activities are finished".

        Callers are already running this on their own background thread
        (spawned from ``_read_loop``/``_on_joy``'s button dispatch), so the
        5s sleep here does not stall keyboard/joy event processing.
        """
        self.get_logger().info(
            f'{label}: activity indicator on, holding {ACTIVITY_INDICATOR_PRE_DELAY_SEC:.0f}s '
            f'before moving (ERC REQ-OPS-080/090/100)...'
        )
        self.stop()
        self._activity_delay_active = True
        self._signal_activity_indicator(True)
        try:
            time.sleep(ACTIVITY_INDICATOR_PRE_DELAY_SEC)
            self._activity_delay_active = False
            return action()
        finally:
            self._activity_delay_active = False
            self._signal_activity_indicator(False)

    def stop_servo(self) -> bool:
        """Call the Servo ``stop_servo`` service and wait for confirmation.

        Waits up to 2 seconds for the service to become available and up to
        10 seconds for the asynchronous call to complete. After Servo stops,
        re-activates the trajectory controller so home / Execute can run.

        Returns:
            bool: True if Servo stopped AND the trajectory controller took
            over; False otherwise.
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

        if not done_event.wait(timeout=10.0):
            self.get_logger().warn('Servo stop timed out')
            return False

        # Prefer JTC when teleop is idle so Plan&Execute / home work.
        return self.use_trajectory_controller()

    def move_to_safe_pose(self, positions=None, name=None):
        """Stop motion and drive the arm to the configured home pose.

        Halts current velocity commands, confirms Servo has stopped and
        the trajectory controller is active, then delegates the actual
        move to ``_move_to_joint_positions``.

        Args:
            positions: Joint targets to use instead of ``self._safe_pose``
                (same order as ``HOME_POSE_JOINTS``) — e.g. drill mode
                passes ``self._drill_home_pose`` here so A/R lands
                somewhere else than the jaw home pose. Defaults to
                ``self._safe_pose`` when omitted.
            name: Label for the log line only (defaults to
                ``self._home_pose_name``); has no effect on motion.

        Returns:
            bool: True if the controller reported the goal SUCCEEDED with
            error code SUCCESSFUL; False if Servo could not be confirmed
            stopped, the trajectory controller could not be activated, the
            action server was unavailable, the goal was rejected, the
            trajectory was aborted/canceled or finished with a controller
            error, or no result arrived within the timeout.
        """
        target_positions = list(self._safe_pose) if positions is None else list(positions)
        target_name = self._home_pose_name if name is None else name

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

        success = self._move_to_joint_positions(target_positions, f'home ({target_name})')
        if success:
            self._pitch_yaw_locked = False
        return success

    def _move_to_joint_positions(self, target_positions, label: str) -> bool:
        """Plan (OMPL, collision-checked) and execute a move from the
        current joint state to ``target_positions``, then wait.

        Used by ``move_to_safe_pose()`` for the home move, and by
        panel_align_node.py's own remembered-position replay for the same
        reason: a raw point-to-point move can't tell whether the
        straight-line path to the target is actually clear. Callers are
        responsible for ``stop()``/``stop_servo()``/switching to the
        trajectory controller first.

        Non-blocking-locked (``_motion_lock``) rather than queued: two 'r'
        presses racing (see the lock's declaration in ``__init__``) should
        fail one of them cleanly, not silently submit a second goal that
        preempts the first mid-motion. ``_motion_lock`` alone only
        protects against racing WITHIN this process though — also takes
        ``arm_motion_lock()`` (see that module) so a home move here can't
        race a live align running in the separate panel_align_node
        process either, for the same reason.

        Blocks the calling thread up to ``safe_pose_timeout`` wall-clock
        seconds (<= 0 waits forever).

        Returns:
            bool: True if the controller reported the goal SUCCEEDED with
            error code SUCCESSFUL; False if another motion was already in
            progress (this process or panel_align_node's), the action
            server was unavailable, the goal was rejected, the trajectory
            was aborted/canceled or finished with a controller error, or
            no result arrived within the timeout.
        """
        if not self._motion_lock.acquire(blocking=False):
            self.get_logger().error(
                f'Another arm motion is already in progress — aborting {label}.'
            )
            return False
        # Lease covers _execute_move_group_constraints' own timeout
        # (self._safe_pose_timeout) plus margin for the planning/service-
        # call overhead around it; <=0 there means "wait forever", so
        # give the lease itself a long-but-finite cap instead of a lease
        # too short to survive an intentionally unbounded wait.
        lease_sec = self._safe_pose_timeout + 15.0 if self._safe_pose_timeout > 0.0 else 600.0
        try:
            try:
                with arm_motion_lock(
                        self._acquire_lock_client, self._release_lock_client,
                        self._motion_lock_holder_id, lease_sec):
                    return self._move_to_joint_positions_locked(target_positions, label)
            except ArmMotionBusy as exc:
                self.get_logger().error(f'{exc} — aborting {label}.')
                return False
        finally:
            self._motion_lock.release()

    def _move_to_joint_positions_locked(self, target_positions, label: str) -> bool:
        joint_constraints = [
            JointConstraint(joint_name=n, position=p, tolerance_above=0.005, tolerance_below=0.005, weight=1.0)
            for n, p in zip(HOME_POSE_JOINTS, target_positions)
        ]
        self.get_logger().info(
            f'Moving to {label} (collision-checked plan): {[round(v, 4) for v in target_positions]}'
        )
        success, error = self._execute_move_group_constraints(
            Constraints(joint_constraints=joint_constraints)
        )
        if not success:
            self.get_logger().error(f'{label} move failed: {error}')
            return False
        self.get_logger().info(f'{label} reached!')
        return True

    def level_tool(self) -> bool:
        """Move to ``LEVEL_TOOL_TARGET_POSE``, collision-checked.

        A fixed joint-space target rather than a computed one — see
        ``LEVEL_TOOL_TARGET_POSE``'s own comment for how it was picked and
        verified. No live geometry computation, which is why it's
        predictable.

        On success this also sets ``_pitch_yaw_locked`` (see its own
        comment) so teleop can't tilt the tool back off vertical
        afterward — translation and roll keep working normally.
        ``move_to_safe_pose()`` clears it.

        Returns:
            bool: True if move_group reported the goal SUCCEEDED; False
            on any failure — see ``move_to_safe_pose``'s own return-value
            contract, same failure modes apply here (Servo not stopped,
            trajectory controller unavailable, action server unavailable,
            goal rejected, no collision-free plan found, or no result
            within the timeout).
        """
        self.stop()

        if not self.stop_servo():
            self.get_logger().error(
                'Could not confirm Servo stopped — aborting level move.'
            )
            return False

        if not self.use_trajectory_controller():
            self.get_logger().error(
                'Could not activate trajectory controller — aborting level move.'
            )
            return False

        if not self._move_group_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('move_group action server not available')
            return False

        joint_constraints = [
            JointConstraint(
                joint_name=name, position=pos,
                tolerance_above=0.005, tolerance_below=0.005, weight=1.0,
            )
            for name, pos in zip(HOME_POSE_JOINTS, LEVEL_TOOL_TARGET_POSE)
        ]

        self.get_logger().info('Leveling tool (fixed target pose, collision-checked plan)...')
        success, error = self._execute_move_group_constraints(
            Constraints(joint_constraints=joint_constraints)
        )
        if not success:
            self.get_logger().error(f'Level move failed: {error}')
            return False
        self._pitch_yaw_locked = True
        self.get_logger().info('Tool leveled! Pitch/yaw locked — "r" unlocks.')
        return True

    def _execute_move_group_constraints(self, constraints: Constraints) -> tuple[bool, str]:
        """Plan and execute a single move_action goal for ``constraints``.

        Shared by ``move_to_safe_pose()`` and ``level_tool()``. Returns
        ``(success, error)`` — ``error`` is empty on success, otherwise a
        short description (goal rejected, no result within the timeout, or
        the MoveIt status/error code on failure).
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = MOVEIT_GROUP_NAME
        goal.request.pipeline_id = 'ompl'
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0

        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.planning_options.plan_only = False  # plan then execute in one goal
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        done_event = threading.Event()
        outcome = {'success': False, 'error': ''}
        goal_handle_box = {}

        def result_cb(future):
            """Record the move_group result's status and MoveIt error code."""
            try:
                wrapped = future.result()
                result = wrapped.result
                if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                        and result.error_code.val == MoveItErrorCodes.SUCCESS):
                    outcome['success'] = True
                else:
                    outcome['error'] = (
                        f'goal status {wrapped.status}, '
                        f'MoveIt error code {result.error_code.val}'
                    )
            except Exception as e:
                outcome['error'] = f'failed to read result: {e!r}'
            finally:
                done_event.set()

        def goal_response_cb(future):
            """Handle move_action's goal-acceptance response."""
            try:
                goal_handle = future.result()
            except Exception as e:
                goal_handle = None
                outcome['error'] = f'goal request failed: {e!r}'
            if not goal_handle or not goal_handle.accepted:
                outcome['error'] = outcome['error'] or 'goal rejected'
                done_event.set()
                return
            goal_handle_box['gh'] = goal_handle
            goal_handle.get_result_async().add_done_callback(result_cb)

        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(goal_response_cb)

        timeout = self._safe_pose_timeout if self._safe_pose_timeout > 0.0 else None
        if not done_event.wait(timeout=timeout):
            self.get_logger().warn(
                f'No move_group result within {self._safe_pose_timeout:.1f}s — '
                'controller may be unresponsive '
                '(raise the safe_pose_timeout parameter if the sim is just slow).'
            )
            # Critical: not cancelling here would leave the goal running
            # server-side after this returns "failed" — callers restart
            # Servo right after any outcome, which would then race an
            # execution still in flight. See panel_align_node.py's
            # _execute() for the same pattern.
            gh = goal_handle_box.get('gh')
            if gh is not None:
                cancel_done = threading.Event()
                gh.cancel_goal_async().add_done_callback(lambda _f: cancel_done.set())
                cancel_done.wait(timeout=5.0)
            return False, 'timed out waiting for a result'
        return outcome['success'], outcome['error']

    def _on_panel_pose(self, msg: PoseStamped):
        """Record the arrival time of a panel_pose message (see is_panel_visible).

        panel_pose_fuser_node only publishes this when its own 2+-marker
        and disagreement checks pass (see panel_perception) — deliberately
        NOT the cheaper panel_visible (1+ marker) topic, since a
        single-marker pose estimate was confirmed live to be unreliable
        enough to compute a physically unreachable align target. Reading
        the same topic panel_align_node itself acts on means "the operator
        sees the prompt" and "align would actually accept this pose" agree.
        """
        self._last_panel_visible_time = self.get_clock().now()

    def is_panel_visible(self) -> bool:
        """Return True if a panel_pose message has arrived recently.

        This is the same 2+-marker bar panel_align_node itself requires
        before it will plan a move — see _on_panel_pose.
        """
        if self._last_panel_visible_time is None:
            return False
        age = (self.get_clock().now() - self._last_panel_visible_time).nanoseconds / 1e9
        return age <= self._panel_visible_max_age_sec

    def align_to_panel(self) -> bool:
        """Ask panel_align_node to align to the panel.

        This node no longer owns the remembered target itself —
        panel_align_node does (see its own align_to_panel(): live
        CV+MoveIt align on the first successful call, then a fresh
        collision-checked MoveIt plan to that exact remembered joint
        target on every call after, rather than a raw point-to-point
        move — see PR review for why the earlier point-to-point replay
        wasn't safe). This method is now just the service call:
        panel_align_node itself decides whether it can act (remembered
        target, or live panel visibility) and reports why not otherwise.

        Returns:
            bool: True if the arm reached a panel-aligned pose; False on
            any failure — see the logged message for which.
        """
        self.stop()

        if not self._panel_align_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('panel_align/align service not available')
            return False

        done_event = threading.Event()
        outcome = {'success': False, 'message': ''}

        def _cb(future):
            try:
                result = future.result()
                outcome['success'] = result.success
                outcome['message'] = result.message
            except Exception as e:
                outcome['message'] = f'panel align call failed: {e!r}'
            finally:
                done_event.set()

        future = self._panel_align_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=self._panel_align_timeout):
            self.get_logger().warn(
                f'panel_align/align timed out after {self._panel_align_timeout:.1f}s'
            )
            return False
        if not outcome['success']:
            self.get_logger().error(f'Panel align failed: {outcome["message"]}')
            return False
        self.get_logger().info(f'Panel align succeeded: {outcome["message"]}')
        self._panel_align_succeeded_once = True
        return True

    def orient_gripper_to_panel(self) -> bool:
        """Ask panel_align_node to reorient just the gripper toward the
        remembered panel direction, without moving the arm back to the
        remembered position. Mirrors align_to_panel()'s call pattern.
        """
        self.stop()

        if not self._orient_gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('panel_align/orient_gripper service not available')
            return False

        done_event = threading.Event()
        outcome = {'success': False, 'message': ''}

        def _cb(future):
            try:
                result = future.result()
                outcome['success'] = result.success
                outcome['message'] = result.message
            except Exception as e:
                outcome['message'] = f'orient gripper call failed: {e!r}'
            finally:
                done_event.set()

        future = self._orient_gripper_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=self._panel_align_timeout):
            self.get_logger().warn(
                f'panel_align/orient_gripper timed out after {self._panel_align_timeout:.1f}s'
            )
            return False
        if not outcome['success']:
            self.get_logger().error(f'Gripper orient failed: {outcome["message"]}')
            return False
        self.get_logger().info(f'Gripper orient succeeded: {outcome["message"]}')
        return True

    def start_servo(self) -> bool:
        """Switch to streaming controller, then start MoveIt Servo.

        Waits up to 2 seconds for the service to become available and up to
        20 seconds for the call to complete. Falls back to the trajectory
        controller on any failure, so the joints are never left claimed by
        the streaming controller with Servo not actually running.

        Returns:
            bool: True if Servo confirmed it started.
        """
        if not self.use_streaming_controller():
            self.get_logger().error(
                'Could not activate forward position controller — Servo not started.'
            )
            return False
        if not self._start_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servo start service not available')
            self.use_trajectory_controller()
            return False

        done_event = threading.Event()
        outcome = {'ok': False}

        def _cb(future):
            try:
                result = future.result()
                outcome['ok'] = bool(result.success)
                if not result.success:
                    self.get_logger().warn(f'Servo start failed: {result.message}')
            except Exception as e:
                self.get_logger().error(f'Servo start error: {e}')
            finally:
                done_event.set()

        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(_cb)

        if not done_event.wait(timeout=20.0):
            self.get_logger().error('Servo start timed out')
            self.use_trajectory_controller()
            return False

        if outcome['ok']:
            self.get_logger().info('Servo started successfully')
        else:
            self.use_trajectory_controller()
        return outcome['ok']

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
            name = SERVO_STATUS_NAMES.get(code, f'UNKNOWN({code})')
            if code in (SERVO_STATUS_OK, SERVO_STATUS_DECELERATE_FOR_APPROACHING_SINGULARITY,
                        SERVO_STATUS_DECELERATE_FOR_LEAVING_SINGULARITY):
                self.get_logger().info(f'Servo status -> {name}')
            elif code == SERVO_STATUS_DECELERATE_FOR_COLLISION:
                # Scales velocity down, doesn't zero it — motion continues.
                self.get_logger().warn(f'Servo status -> {name} (decelerating, not stopped)')
            else:
                self.get_logger().warn(f'Servo status -> {name} (motion stopped by Servo)')
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
        # Scaled by hold_boost (see set_velocity) so a boosted push doesn't
        # out-muscle a fixed-strength hold — the wrist's resistance to being
        # bent grows with the push instead of staying constant.
        gain = HOLD_ANGULAR_GAIN * self._hold_boost
        cap = HOLD_ANGULAR_MAX * self._hold_boost
        wx = max(-cap, min(cap, wx + gain * hx))
        wy = max(-cap, min(cap, wy + gain * hy))
        wz = max(-cap, min(cap, wz + gain * hz))
        return wx, wy, wz

    def _level_hold(self, wx, wy, wz, local_axis=SAMPLING_POINT_AXIS):
        """Keep local_axis (in ee_frame) pointed at fixed DOWN_AXIS; wz (roll) passes through untouched.

        Shared by sampling mode and drill mode — they differ only in which
        local_axis (SAMPLING_POINT_AXIS vs DRILL_POINT_AXIS) gets aligned.
        Separate from _orientation_hold: that one drops on any rotation input
        and targets a captured reference, but here roll is continuous and the
        target is a fixed constant. Correction uses exact cross-product
        rotation (not _orientation_hold's small-angle quaternion-drop trick,
        which spun wildly on large initial errors here).
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._linear_frame, self._ee_frame, rclpy.time.Time()
            )
        except TransformException:
            return wx, wy, wz
        q = transform.transform.rotation
        # Current pointing axis, in linear_frame.
        zx, zy, zz = _rotate_vector_by_quat(q, *local_axis)
        tx, ty, tz = DOWN_AXIS
        dot = max(-1.0, min(1.0, zx * tx + zy * ty + zz * tz))
        angle = math.acos(dot)
        axis_x = zy * tz - zz * ty
        axis_y = zz * tx - zx * tz
        axis_len = math.hypot(axis_x, axis_y)
        if axis_len < 1e-8:
            # Already pointing down (or exactly opposite — no unique
            # shortest-rotation axis either way; leave uncorrected rather
            # than divide by ~0).
            return wx, wy, wz
        scale = angle / axis_len
        hx, hy = axis_x * scale, axis_y * scale
        gain = HOLD_ANGULAR_GAIN * self._hold_boost
        cap = HOLD_ANGULAR_MAX * self._hold_boost
        wx = max(-cap, min(cap, wx + gain * hx))
        wy = max(-cap, min(cap, wy + gain * hy))
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
        if self._sampling_mode:
            wx, wy, wz = self._level_hold(wx, wy, wz, SAMPLING_POINT_AXIS)
        elif self._drill_mode:
            wx, wy, wz = self._level_hold(wx, wy, wz, DRILL_POINT_AXIS)
        else:
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

        self._publish_gripper()

    def _publish_gripper(self):
        """Integrate gripper position from ``gripper_vel`` and publish.

        Withheld entirely until the first /joint_states sync (see
        _gripper_position above) — otherwise this would command the
        default-assumed 0.0 (closed) from the very first tick, which on a
        restart while the real gripper is open would slam it shut before
        any real state ever arrived.

        Runs every publish tick (100Hz) for smooth motion. Right and left
        each have their own single-joint controller (see GRIPPER_JOINT_NAME
        above) and are published separately every time; left is always
        -right.
        """
        if not self._gripper_state_received:
            return

        now = time.monotonic()
        if self.gripper_vel != 0.0:
            if self._last_gripper_tick_time is not None:
                dt = now - self._last_gripper_tick_time
                self._gripper_position += self.gripper_vel * dt
            self._last_gripper_tick_time = now
        else:
            self._last_gripper_tick_time = None

        self._gripper_position = max(0.0, min(self._gripper_stroke, self._gripper_position))

        self._gripper_right_pub.publish(Float64MultiArray(data=[self._gripper_position]))
        self._gripper_left_pub.publish(Float64MultiArray(data=[-self._gripper_position]))



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


