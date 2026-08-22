/**
 * @brief Deterministic 3D Tilt Broadcaster
 *
 * This node replaces a 3D EKF for calculating rover body tilt. It subscribes to 
 * the IMU orientation, dynamically removes any static mounting rotation (e.g., camera 
 * tilt) via TF, extracts the pure roll and pitch of the base, and forcefully locks 
 * yaw to exactly 0.0. It then broadcasts the base_footprint -> base_link transform, 
 * guaranteeing a perfectly flat navigation footprint without kinematic yaw drift.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class TiltBroadcasterNode : public rclcpp::Node
{
public:
  TiltBroadcasterNode()
  : Node("tilt_broadcaster_node")
  {
    declare_parameter("imu_topic", std::string("/zed2i/imu/data"));
    declare_parameter("parent_frame", std::string("base_footprint"));
    declare_parameter("child_frame", std::string("base_link"));
    // Height of base_link above base_footprint/ground -- must match
    // base_link_ground_height in rover_description/urdf/properties.xacro.
    declare_parameter("ground_height");
    declare_parameter("tf_tolerance", 0.1);

    parent_frame_ = get_parameter("parent_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();
    ground_height_ = get_parameter("ground_height").as_double();
    tf_tolerance_ = get_parameter("tf_tolerance").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&TiltBroadcasterNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // The IMU is rigidly mounted wherever the URDF puts it (e.g. tilted for
    // the ZED's camera-aim angle), so its raw reading bakes in that mount
    // offset alongside the vehicle's real attitude. Look the offset up live
    // from TF rather than hardcoding it, so this keeps working regardless
    // of where the sensor is actually mounted or if that ever changes.
    tf2::Quaternion q_mount(0.0, 0.0, 0.0, 1.0);
    try {
      const geometry_msgs::msg::TransformStamped mount_tf = tf_buffer_->lookupTransform(
        child_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_tolerance_));
      const auto & r = mount_tf.transform.rotation;
      q_mount = tf2::Quaternion(r.x, r.y, r.z, r.w);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "tilt_broadcaster_node: TF lookup for IMU mount (%s -> %s) failed: %s",
        child_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    tf2::Quaternion q_imu_raw(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    // q_imu_raw is the IMU frame's orientation in the world; q_mount is the
    // IMU frame's orientation as seen from child_frame_ (the static mount
    // offset). Removing it leaves child_frame_'s own orientation in world.
    const tf2::Quaternion q_world_child = q_imu_raw * q_mount.inverse();

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_world_child).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_tilt;
    q_tilt.setRPY(roll, pitch, 0.0);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = ground_height_;
    tf_msg.transform.rotation.x = q_tilt.x();
    tf_msg.transform.rotation.y = q_tilt.y();
    tf_msg.transform.rotation.z = q_tilt.z();
    tf_msg.transform.rotation.w = q_tilt.w();

    broadcaster_->sendTransform(tf_msg);
  }

  std::string parent_frame_, child_frame_;
  double ground_height_;
  double tf_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TiltBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
