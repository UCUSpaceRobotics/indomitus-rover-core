"""MoveIt-planned, collision-checked alignment to a detected panel.

Separate node from ``keyboard_servo_node.py`` on purpose: MoveIt planning
is a new, heavier dependency this codebase hasn't used before, and
shouldn't be mixed into the input-handling module. Exposes one
``std_srvs/srv/Trigger`` service (``align``), the same idiom already used
for ``servo_node/start_servo``/``stop_servo``.

``align_to_panel()`` mirrors ``ServoController.move_to_safe_pose()``'s
contract: returns ``bool``, never raises, logs a distinguishing message
per failure mode, and never re-enables Servo itself — the caller (the
'p'-key/gamepad-button handler in ``keyboard_servo_node.py``) decides
what to do next.

The panel is a completely separate Gazebo model, not part of the arm's
own robot_description — nothing else in this repo inserts it (or
anything) into MoveIt's planning scene. Without step 5 below explicitly
adding a CollisionObject for it before planning, "collision-checked"
would silently mean "collision-checked against everything except the one
object we're deliberately moving toward."

Every blocking sub-call in align_to_panel() (stop_servo, apply_planning_
scene, /move_action, /execute_trajectory) waits on a threading.Event set
from an rclpy done-callback — the same pattern keyboard_servo_node.py
uses successfully, but THERE it always runs on a background thread
separate from the ROS spin thread. Here, align_to_panel() runs FROM
_on_align_request, itself a callback invoked BY the executor — with the
default SingleThreadedExecutor, that self-deadlocks: the executor can't
get back around to invoke the done-callback that would set the Event
while it's still inside the callback that's waiting on it. Fixed by
running everything on one ReentrantCallbackGroup under a
MultiThreadedExecutor (see main()), so the service callback and its
sub-calls' response callbacks can interleave on different threads.
"""

import math
import threading
import xml.etree.ElementTree as ET

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, CollisionObject, Constraints, MoveItErrorCodes,
    OrientationConstraint, PlanningOptions, PlanningScene, PlanningSceneWorld,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

from arm_tasks.camera_target_math import (
    StandoffResult, compose_transforms, compute_standoff_distance, compute_target_tip_pose,
)

GROUP_NAME = 'indomitus_arm'
TIP_LINK = 'arm_tcp_link'  # confirmed in arm_moveit_config/config/indomitus_arm.srdf
CAMERA_OPTICAL_FRAME = 'arm_camera_optical_frame'
CONTROLLER_NAME = 'indomitus_arm_controller'  # matches move_to_safe_pose()'s controller

DEFAULT_PANEL_POSE_TOPIC = '/panel_pose'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera_info'
# Matches keyboard_servo_node.py's own panel_visible_max_age_sec default —
# see its comment for why 1s was too tight (confirmed live: detection
# drops out for over a second even with the panel fully in frame).
DEFAULT_MAX_PANEL_POSE_AGE_SEC = 3.0

# Panel collision box (panel_description/urdf/panel_macro.xacro's
# panel_base_link collision geometry). Duplicated here — see
# panel_geometry.py's KNOWN_MARKER_LOCAL_POSITIONS for the same tradeoff.
PANEL_WIDTH = 0.33
PANEL_DEPTH = 0.072
PANEL_HEIGHT = 0.45
# panel_base_link's own <collision><origin> in panel_macro.xacro — the
# fused panel_pose we receive is panel_base_link's own origin (bottom
# edge, per how panel_geometry.py's KNOWN_MARKER_LOCAL_POSITIONS were
# derived), NOT the collision box's center, so this local offset has to
# be applied before inserting the CollisionObject. Missing this made the
# box straddle the wrong volume (half sunk below the real panel, and not
# reaching its actual top) — confirmed live: OMPL's goal-tree sampling
# failed 100% of the time until this was added, because target poses
# computed relative to the (correctly-positioned) marker detections kept
# landing inside this mispositioned collision volume.
PANEL_COLLISION_LOCAL_OFFSET = ((0.0, 0.013, 0.225), (0.0, 0.0, 0.0, 1.0))
# The point we actually aim the camera at: the middle of the panel's own
# FRONT FACE, in panel_base_link's local frame — not panel_base_link's
# origin itself (near the bottom edge — see PANEL_COLLISION_LOCAL_OFFSET
# above) and not the collision box's volumetric center (that one's Y
# accounts for the solid mesh's depth, not the face plane). x=0 (panel is
# symmetric left/right: top_left x=-0.135, top_right x=+0.135 in
# panel_geometry.py), z=0.225 (half of PANEL_HEIGHT), y=-0.015 (front
# face plane, per panel_macro.xacro's own comment on panel.stl's
# geometry). Always the same fixed point relative to the panel, however
# it was detected — this is what makes align_to_panel()'s target
# deterministic/repeatable rather than depending on exactly which
# markers happened to be visible.
PANEL_CENTER_LOCAL_OFFSET = ((0.0, -0.015, 0.225), (0.0, 0.0, 0.0, 1.0))

# Joint position limits (arm_description/urdf/arm_macro.xacro's <limit> tags
# on each revolute joint) — duplicated rather than parsed from
# robot_description at runtime, same tradeoff as PANEL_WIDTH/HEIGHT above.
# Used only for the post-plan joint-limit-margin check (step 7); NOT used
# for IK/collision — MoveGroup's own planner already enforces the real
# ones from the URDF/SRDF.
JOINT_LIMITS = {
    'arm_mount_base_joint': (-math.pi, math.pi),
    'arm_base_shoulder_joint': (-math.pi, math.pi),
    'arm_shoulder_forearm_joint': (-math.pi / 2, math.pi / 2),
    'arm_forearm_wrist_1_joint': (-math.pi / 2, math.pi / 2),
    'arm_wrist_1_wrist_2_joint': (-math.pi, math.pi),
    'arm_wrist_2_end_effector_joint': (-math.pi, math.pi),
}


class PanelAlignNode(Node):
    def __init__(self):
        super().__init__('panel_align_node')

        self.declare_parameter('panel_pose_topic', DEFAULT_PANEL_POSE_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('max_panel_pose_age_sec', DEFAULT_MAX_PANEL_POSE_AGE_SEC)
        self.declare_parameter('standoff_margin_multiplier', 1.15)
        self.declare_parameter('standoff_min_floor', 0.15)
        self.declare_parameter('standoff_max_reach', 0.75)
        # TEMPORARY, for verifying the align mechanism itself end-to-end
        # without fighting reach: overrides the real FOV-fit standoff
        # (compute_standoff_distance, ~0.6-0.7m for this panel/camera —
        # correct for "whole panel fits in frame", but that plus however
        # far the arm already is from the panel easily exceeds real reach)
        # with a small fixed distance close to wherever the arm currently
        # is. Set use_fixed_test_standoff:=false to go back to the real
        # FOV-based computation.
        self.declare_parameter('use_fixed_test_standoff', True)
        self.declare_parameter('fixed_test_standoff', 0.2)
        self.declare_parameter('joint_limit_margin_fraction', 0.08)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('max_velocity_scaling_factor', 0.3)
        self.declare_parameter('max_acceleration_scaling_factor', 0.3)
        # Loosened from an initial 0.005/0.05 (5mm / ~3deg) — confirmed
        # live those were too tight for OMPL's goal-tree sampler to find
        # any valid state at all ("Unable to sample any valid states for
        # goal tree", 100% failure), most likely because
        # arm_moveit_config/config/kinematics.yaml's KDL solver only gets
        # a 5ms timeout per IK attempt (a pre-existing, separately-flagged
        # tuning risk) — a wider target region needs far fewer solver
        # iterations to land inside it. 2cm / ~11deg is still precise
        # enough for "camera aimed at the panel".
        self.declare_parameter('position_tolerance', 0.02)
        self.declare_parameter('orientation_tolerance', 0.2)

        self._panel_pose_topic = self.get_parameter('panel_pose_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value

        self._latest_panel_pose: PoseStamped | None = None
        self._latest_camera_info: CameraInfo | None = None
        self._camera_to_tip = None  # cached at startup, see _try_cache_camera_to_tip

        # See module docstring: everything here shares one reentrant group
        # so the align service callback and its own sub-calls' response
        # callbacks can interleave (requires MultiThreadedExecutor, see
        # main() — a plain rclpy.spin() would deadlock).
        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            PoseStamped, self._panel_pose_topic, self._on_panel_pose, 10,
            callback_group=cb_group)
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info, 10,
            callback_group=cb_group)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_retry_timer = self.create_timer(
            1.0, self._try_cache_camera_to_tip, callback_group=cb_group)

        self._move_action_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=cb_group)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory', callback_group=cb_group)
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=cb_group)
        self._stop_servo_client = self.create_client(
            Trigger, 'servo_node/stop_servo', callback_group=cb_group)

        self.create_service(
            Trigger, 'panel_align/align', self._on_align_request, callback_group=cb_group)

        self.get_logger().info('panel_align_node ready — waiting for camera<->tip TF...')

    # ---- subscriptions -------------------------------------------------

    def _on_panel_pose(self, msg: PoseStamped) -> None:
        self._latest_panel_pose = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _try_cache_camera_to_tip(self) -> None:
        if self._camera_to_tip is not None:
            self._tf_retry_timer.cancel()
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                CAMERA_OPTICAL_FRAME, TIP_LINK, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().debug(f'camera->tip TF not yet available: {exc}')
            return
        t = tf.transform.translation
        r = tf.transform.rotation
        self._camera_to_tip = ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
        self._tf_retry_timer.cancel()
        self.get_logger().info(
            f'Cached static {CAMERA_OPTICAL_FRAME} -> {TIP_LINK} transform.'
        )

    # ---- service entry point -------------------------------------------

    def _on_align_request(self, request, response):
        response.success = self.align_to_panel()
        response.message = self._last_status_message
        return response

    # ---- the actual sequence --------------------------------------------

    def align_to_panel(self) -> bool:
        self._last_status_message = ''

        if not self._call_stop_servo():
            return self._fail('Could not confirm Servo stopped — aborting panel align.')

        panel_pose = self._latest_panel_pose
        if panel_pose is None:
            return self._fail('No panel pose has been received yet.')
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(panel_pose.header.stamp)).nanoseconds / 1e9
        max_age = self.get_parameter('max_panel_pose_age_sec').value
        if age > max_age:
            return self._fail(f'Panel pose is stale ({age:.2f}s old, max {max_age:.2f}s) — panel out of view?')

        if self._camera_to_tip is None:
            return self._fail(
                f'{CAMERA_OPTICAL_FRAME} -> {TIP_LINK} TF unavailable — '
                'startup TF lookup never succeeded, refusing to guess. Restart the node.'
            )

        info = self._latest_camera_info
        if info is None or info.k[0] == 0.0 or info.k[4] == 0.0:
            return self._fail('No valid CameraInfo received yet (fx/fy unknown).')

        if self.get_parameter('use_fixed_test_standoff').value:
            standoff = StandoffResult(
                distance=self.get_parameter('fixed_test_standoff').value,
                within_bounds=True, reason='')
        else:
            standoff = compute_standoff_distance(
                fx=info.k[0], fy=info.k[4], image_width=info.width, image_height=info.height,
                panel_width=PANEL_WIDTH, panel_height=PANEL_HEIGHT,
                margin_multiplier=self.get_parameter('standoff_margin_multiplier').value,
                min_floor=self.get_parameter('standoff_min_floor').value,
                max_reach=self.get_parameter('standoff_max_reach').value,
            )
        if not standoff.within_bounds:
            return self._fail(f'Standoff distance out of range: {standoff.reason}')

        p = panel_pose.pose.position
        o = panel_pose.pose.orientation
        panel_pose_in_camera = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        # Aim at the panel's own fixed center point (see
        # PANEL_CENTER_LOCAL_OFFSET), not panel_base_link's origin
        # directly — this is what makes the target always the same point
        # on the panel regardless of exactly how the pose was detected.
        panel_center_in_camera = compose_transforms(
            panel_pose_in_camera, PANEL_CENTER_LOCAL_OFFSET)
        target_position, target_orientation = compute_target_tip_pose(
            panel_pose_in_camera=panel_center_in_camera,
            camera_to_tip=self._camera_to_tip,
            standoff=standoff.distance,
        )
        target_pose_msg = PoseStamped()
        target_pose_msg.header.frame_id = panel_pose.header.frame_id
        (target_pose_msg.pose.position.x, target_pose_msg.pose.position.y,
         target_pose_msg.pose.position.z) = target_position
        (target_pose_msg.pose.orientation.x, target_pose_msg.pose.orientation.y,
         target_pose_msg.pose.orientation.z, target_pose_msg.pose.orientation.w) = target_orientation

        if not self._apply_panel_collision_object(panel_pose):
            return self._fail('Could not insert panel CollisionObject into the planning scene.')

        plan_result = self._request_plan(target_pose_msg)
        if plan_result is None:
            return self._fail('MoveGroup planning request timed out or was rejected.')
        if plan_result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self._fail(f'Planning failed (MoveItErrorCodes.val={plan_result.error_code.val}).')

        margin_ok, margin_reason = self._check_joint_margins(plan_result.planned_trajectory)
        if not margin_ok:
            return self._fail(f'Planned pose too close to a joint limit: {margin_reason}')

        exec_ok, exec_reason = self._execute(plan_result.planned_trajectory)
        if not exec_ok:
            return self._fail(f'Trajectory execution failed: {exec_reason}')

        self._last_status_message = f'Aligned to panel (standoff={standoff.distance:.3f}m).'
        self.get_logger().info(self._last_status_message)
        return True

    def _fail(self, message: str) -> bool:
        self._last_status_message = message
        self.get_logger().error(message)
        return False

    # ---- steps -----------------------------------------------------------

    def _call_stop_servo(self) -> bool:
        if not self._stop_servo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('servo_node/stop_servo not available')
            return False
        done = threading.Event()
        result = {}

        def _cb(fut):
            result['r'] = fut.result()
            done.set()

        self._stop_servo_client.call_async(Trigger.Request()).add_done_callback(_cb)
        if not done.wait(timeout=3.0):
            return False
        return bool(result.get('r') and result['r'].success)

    def _apply_panel_collision_object(self, panel_pose: PoseStamped) -> bool:
        if not self._apply_scene_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/apply_planning_scene service not available')
            return False

        p = panel_pose.pose.position
        o = panel_pose.pose.orientation
        collision_center_pos, collision_center_quat = compose_transforms(
            ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)),
            PANEL_COLLISION_LOCAL_OFFSET,
        )
        collision_pose = Pose()
        (collision_pose.position.x, collision_pose.position.y,
         collision_pose.position.z) = collision_center_pos
        (collision_pose.orientation.x, collision_pose.orientation.y,
         collision_pose.orientation.z, collision_pose.orientation.w) = collision_center_quat

        co = CollisionObject()
        co.header = panel_pose.header
        co.id = 'panel'
        co.primitives = [SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT])]
        co.primitive_poses = [collision_pose]
        co.operation = CollisionObject.ADD

        scene = PlanningScene(is_diff=True)
        scene.world = PlanningSceneWorld(collision_objects=[co])

        req = ApplyPlanningScene.Request(scene=scene)
        done = threading.Event()
        result = {}

        def _cb(fut):
            result['r'] = fut.result()
            done.set()

        self._apply_scene_client.call_async(req).add_done_callback(_cb)
        if not done.wait(timeout=3.0):
            return False
        return bool(result.get('r') and result['r'].success)

    def _request_plan(self, target_pose: PoseStamped):
        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/move_action server not available')
            return None

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP_NAME
        req.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        req.allowed_planning_time = self.get_parameter('allowed_planning_time').value
        req.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        req.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor').value
        req.start_state.is_diff = True

        pos_tol = self.get_parameter('position_tolerance').value
        orient_tol = self.get_parameter('orientation_tolerance').value

        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = TIP_LINK
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])],
            primitive_poses=[target_pose.pose],
        )
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = target_pose.header
        oc.link_name = TIP_LINK
        oc.orientation = target_pose.pose.orientation
        oc.absolute_x_axis_tolerance = orient_tol
        oc.absolute_y_axis_tolerance = orient_tol
        oc.absolute_z_axis_tolerance = orient_tol
        oc.weight = 1.0

        req.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        goal.planning_options = PlanningOptions(plan_only=True)

        done = threading.Event()
        result = {}

        def _result_cb(fut):
            result['r'] = fut.result().result
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        self._move_action_client.send_goal_async(goal).add_done_callback(_goal_cb)
        if not done.wait(timeout=self.get_parameter('allowed_planning_time').value + 5.0):
            return None
        return result.get('r')

    def _check_joint_margins(self, planned_trajectory) -> tuple[bool, str]:
        points = planned_trajectory.joint_trajectory.points
        names = planned_trajectory.joint_trajectory.joint_names
        if not points:
            return False, 'planned trajectory has no points'
        last = points[-1]
        margin_fraction = self.get_parameter('joint_limit_margin_fraction').value

        worst_joint, worst_margin = None, float('inf')
        for name, position in zip(names, last.positions):
            if name not in JOINT_LIMITS:
                continue
            lower, upper = JOINT_LIMITS[name]
            span = upper - lower
            margin = min(position - lower, upper - position) / span
            if margin < worst_margin:
                worst_joint, worst_margin = name, margin

        if worst_joint is not None and worst_margin < margin_fraction:
            return False, f'{worst_joint} only {worst_margin:.1%} clear of its limit (need {margin_fraction:.1%})'
        return True, ''

    def _execute(self, trajectory) -> tuple[bool, str]:
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            return False, '/execute_trajectory server not available'

        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        done = threading.Event()
        result = {}

        def _result_cb(fut):
            result['r'] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        self._execute_client.send_goal_async(goal).add_done_callback(_goal_cb)
        if not done.wait(timeout=60.0):
            return False, 'execution timed out'

        wrapped = result.get('r')
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'execution goal status={wrapped.status if wrapped else "none"}'
        if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, f'MoveItErrorCodes.val={wrapped.result.error_code.val}'
        return True, ''


def main():
    rclpy.init()
    node = PanelAlignNode()
    # MultiThreadedExecutor, not the default rclpy.spin() — required so
    # the align service callback and its own sub-calls' response
    # callbacks (same ReentrantCallbackGroup, see __init__) can actually
    # interleave across threads instead of self-deadlocking.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
