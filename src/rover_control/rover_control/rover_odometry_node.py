"""
Node that estimates and publishes 2D wheel odometry.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from indomitus_interfaces.msg import ChassisStatus


DRIVE_SIGNS = [1, -1, 1, -1]


def build_kinematics_matrix(wheel_positions: np.ndarray) -> np.ndarray:
    rows = []
    for x, y in wheel_positions:
        rows.append([1, 0, -y])
        rows.append([0, 1,  x])
    return np.array(rows, dtype=float)


class RoverOdometryPublisher(Node):

    def __init__(self):
        super().__init__("rover_odometry_publisher")

        # Config: rover_description/config/rover_geometry.yaml
        self.declare_parameter("wheelbase",    0.842)
        self.declare_parameter("track_width",  0.682)
        self.declare_parameter("wheel_radius", 0.16)

        wb = self.get_parameter("wheelbase").value
        tw = self.get_parameter("track_width").value
        self._wheel_radius = self.get_parameter("wheel_radius").value

        # Wheels positions
        hx = wb / 2.0
        hy = tw / 2.0

        self._wheel_positions = np.array([
            [ hx,  hy],   # FL
            [ hx, -hy],   # FR
            [-hx,  hy],   # RL
            [-hx, -hy],   # RR
        ])

        A = build_kinematics_matrix(self._wheel_positions)
        self._A_pinv = np.linalg.pinv(A)

        self.get_logger().info(
            f"Geometry loaded — wheelbase={wb} track={tw} radius={self._wheel_radius}"
        )
        self.get_logger().info(f"Wheel positions:\n{self._wheel_positions}")

        # Rover position
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_stamp = None

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._chassis_sub = self.create_subscription(
            ChassisStatus, "/chassis/motor_states",
            self._chassis_status_callback, 10
        )

    # ──────────────────────────────────────────────────────────────
    STEER_NAMES = ["fl_wheel_mount_joint", "fr_wheel_mount_joint", "bl_wheel_mount_joint", "br_wheel_mount_joint"]
    DRIVE_NAMES = ["fl_wheel_joint",       "fr_wheel_joint",       "bl_wheel_joint",       "br_wheel_joint"]

    def _chassis_status_callback(self, msg: ChassisStatus):
        now = self.get_clock().now()

        steer_angles = {}
        drive_speeds = {}

        for motor in msg.motors:
            if not motor.kinematic_valid:
                continue
            name = motor.joint_name
            for i, sname in enumerate(self.STEER_NAMES):
                if sname in name:
                    steer_angles[i] = motor.position
            for i, dname in enumerate(self.DRIVE_NAMES):
                if dname in name:
                    drive_speeds[i] = motor.velocity * DRIVE_SIGNS[i]

        if len(steer_angles) < 4 or len(drive_speeds) < 4:
            self.get_logger().warn(
                f'Недостатньо даних: steer={list(steer_angles.keys())} drive={list(drive_speeds.keys())}'
            )
            self._last_stamp = now
            return

        # Speeds in Cartesian coordinates
        wheel_vels = np.zeros(8)
        for i in range(4):
            speed = drive_speeds[i] * self._wheel_radius
            angle = steer_angles[i]
            wheel_vels[2*i    ] = speed * np.cos(angle)
            wheel_vels[2*i + 1] = speed * np.sin(angle)

        # Least square solution, system is overdetermined
        vx, vy, wz = self._A_pinv @ wheel_vels

        VEL_EPS = 0.05  # m/s
        WZ_EPS  = 0.001 # rad/s
        if abs(vx) < VEL_EPS: vx = 0.0
        if abs(vy) < VEL_EPS: vy = 0.0
        if abs(wz) < WZ_EPS:  wz = 0.0

        # exp map
        if self._last_stamp is not None:
            dt = (now - self._last_stamp).nanoseconds * 1e-9

            if abs(wz) > 1e-9:
                # shift in local (rover) coordinates
                dx_local = (vx * np.sin(wz * dt) - vy * (1 - np.cos(wz * dt))) / wz
                dy_local = (vx * (1 - np.cos(wz * dt)) + vy * np.sin(wz * dt)) / wz
                # shift in global coordinates
                self._x += dx_local * np.cos(self._theta) - dy_local * np.sin(self._theta)
                self._y += dx_local * np.sin(self._theta) + dy_local * np.cos(self._theta)
            else:
                self._x += (vx * np.cos(self._theta) - vy * np.sin(self._theta)) * dt
                self._y += (vx * np.sin(self._theta) + vy * np.cos(self._theta)) * dt

            self._theta += wz * dt

        # self.get_logger().info(
        #     f'x={self._x:.3f} y={self._y:.3f} theta={self._theta:.3f}'
        # )

        self._last_stamp = now
        self._publish_odom(vx, vy, wz, now)

    def _publish_odom(self, vx, vy, wz, stamp):
        from math import cos, sin

        qz = sin(self._theta / 2.0)
        qw = cos(self._theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = RoverOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
