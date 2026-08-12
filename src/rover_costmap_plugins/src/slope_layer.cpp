#include "rover_costmap_plugins/slope_layer.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace rover_costmap_plugins
{

SlopeLayer::SlopeLayer() {}

void SlopeLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("cloud_topic", rclcpp::ParameterValue(std::string("/zed2i/points")));
  declareParameter("base_frame", rclcpp::ParameterValue(std::string("base_footprint")));
  declareParameter("grid_resolution", rclcpp::ParameterValue(0.15));
  declareParameter("grid_range", rclcpp::ParameterValue(4.0));
  declareParameter("min_height", rclcpp::ParameterValue(-0.30));
  declareParameter("max_height", rclcpp::ParameterValue(1.20));
  declareParameter("min_points_per_cell", rclcpp::ParameterValue(6));
  declareParameter("traversable_slope_deg", rclcpp::ParameterValue(20.0));
  declareParameter("lethal_slope_deg", rclcpp::ParameterValue(32.0));
  declareParameter("roughness_std_thresh", rclcpp::ParameterValue(0.06));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".cloud_topic", cloud_topic_);
  node->get_parameter(name_ + ".base_frame", base_frame_);
  node->get_parameter(name_ + ".grid_resolution", grid_resolution_);
  node->get_parameter(name_ + ".grid_range", grid_range_);
  node->get_parameter(name_ + ".min_height", min_height_);
  node->get_parameter(name_ + ".max_height", max_height_);
  node->get_parameter(name_ + ".min_points_per_cell", min_points_per_cell_);
  node->get_parameter(name_ + ".traversable_slope_deg", traversable_slope_deg_);
  node->get_parameter(name_ + ".lethal_slope_deg", lethal_slope_deg_);
  node->get_parameter(name_ + ".roughness_std_thresh", roughness_std_thresh_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SlopeLayer::cloudCallback, this, std::placeholders::_1));

  grid_w_ = static_cast<int>(std::ceil((2.0 * grid_range_) / grid_resolution_));
  grid_h_ = grid_w_;
  grid_origin_x_ = -grid_range_;
  grid_origin_y_ = -grid_range_;
  grid_.assign(grid_w_ * grid_h_, SlopeCell());

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "SlopeLayer '%s' up: topic=%s base_frame=%s traversable<=%.1f lethal>=%.1f deg",
    name_.c_str(), cloud_topic_.c_str(), base_frame_.c_str(),
    traversable_slope_deg_, lethal_slope_deg_);
}

void SlopeLayer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // ZED points arrive in zed2i_left_camera_frame (tilted ~15 deg down per the
  // xacro). Transform into base_footprint so height/slope thresholds are
  // relative to the chassis, not the tilted camera. The static TF for this
  // comes from robot_state_publisher via the URDF, so it's always available.
  sensor_msgs::msg::PointCloud2 cloud_in_base;
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      base_frame_, msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(*msg, cloud_in_base, tf);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("SlopeLayer"), *rclcpp::Clock::make_shared(), 5000,
      "TF %s->%s failed: %s", msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  std::fill(grid_.begin(), grid_.end(), SlopeCell());

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_in_base, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_in_base, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_in_base, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    float x = *it_x, y = *it_y, z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
    if (z < min_height_ || z > max_height_) {continue;}
    if (std::abs(x) > grid_range_ || std::abs(y) > grid_range_) {continue;}

    int gx = static_cast<int>((x - grid_origin_x_) / grid_resolution_);
    int gy = static_cast<int>((y - grid_origin_y_) / grid_resolution_);
    if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_) {continue;}

    SlopeCell & c = grid_[gy * grid_w_ + gx];
    c.point_count++;
    c.sum_x += x; c.sum_y += y; c.sum_z += z;
    c.sxx += x * x; c.syy += y * y; c.szz += z * z;
    c.sxy += x * y; c.sxz += x * z; c.syz += y * z;
  }

  recomputeGrid();
  has_data_ = true;
}

void SlopeLayer::recomputeGrid()
{
  for (auto & c : grid_) {
    if (c.point_count < min_points_per_cell_) {
      c.valid = false;
      continue;
    }
    const double n = c.point_count;
    const double mx = c.sum_x / n, my = c.sum_y / n, mz = c.sum_z / n;

    double cxx = c.sxx / n - mx * mx;
    double cyy = c.syy / n - my * my;
    double czz = c.szz / n - mz * mz;
    double cxy = c.sxy / n - mx * my;
    double cxz = c.sxz / n - mx * mz;
    double cyz = c.syz / n - my * mz;

    // A few power iterations toward the smallest-eigenvalue direction of
    // (trace*I - C) approximate the plane normal well enough for a single
    // small cell, without pulling in a linear-algebra dependency.
    double trace = cxx + cyy + czz;
    double Ax = trace - cxx, Ay = -cxy, Az = -cxz;
    double Bx = -cxy, By = trace - cyy, Bz = -cyz;
    double Cx = -cxz, Cy = -cyz, Cz = trace - czz;

    double vx = 0.0, vy = 0.0, vz = 1.0;
    for (int iter = 0; iter < 8; ++iter) {
      double nx = Ax * vx + Ay * vy + Az * vz;
      double ny = Bx * vx + By * vy + Bz * vz;
      double nz = Cx * vx + Cy * vy + Cz * vz;
      double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (norm < 1e-9) {break;}
      vx = nx / norm; vy = ny / norm; vz = nz / norm;
    }
    if (vz < 0) {vx = -vx; vy = -vy; vz = -vz;}

    double slope_rad = std::acos(std::clamp(vz, -1.0, 1.0));
    c.slope_deg = slope_rad * 180.0 / M_PI;

    double planar_residual = std::sqrt(std::max(0.0, cxx + cyy + czz - trace));
    c.valid = true;
    if (planar_residual > roughness_std_thresh_) {
      c.slope_deg = std::max(c.slope_deg, lethal_slope_deg_ + 1.0);  // rock/ledge -> lethal
    }
  }
}

void SlopeLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || !has_data_) {return;}
  // Bound the update window around the robot's current pose in the
  // costmap's own global frame (odom for local_costmap, map for global).
  *min_x = std::min(*min_x, robot_x - grid_range_);
  *min_y = std::min(*min_y, robot_y - grid_range_);
  *max_x = std::max(*max_x, robot_x + grid_range_);
  *max_y = std::max(*max_y, robot_y + grid_range_);
}

void SlopeLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_data_) {return;}
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Our slope grid is stored in base_frame_ coordinates, but the costmap
  // cells are in the costmap's own global frame (odom or map). Look up the
  // rigid transform ONCE per update and apply it manually per-cell instead
  // of paying a full TF lookup per cell.
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  double tx = 0.0, ty = 0.0, yaw = 0.0;
  try {
    geometry_msgs::msg::TransformStamped tf =
      tf_buffer_->lookupTransform(base_frame_, global_frame, tf2::TimePointZero);
    tx = tf.transform.translation.x;
    ty = tf.transform.translation.y;
    // Manual yaw-from-quaternion instead of tf2::getYaw(), to avoid pulling
    // in tf2_geometry_msgs just for one conversion (that's what caused the
    // "undefined symbol: tf2::fromMsg" crash at activation).
    const auto & q = tf.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("SlopeLayer"), *rclcpp::Clock::make_shared(), 5000,
      "TF %s->%s failed in updateCosts: %s", global_frame.c_str(), base_frame_.c_str(), ex.what());
    return;
  }
  const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // world (global_frame) -> base_frame_
      double bx = cos_yaw * wx - sin_yaw * wy + tx;
      double by = sin_yaw * wx + cos_yaw * wy + ty;

      int gx = static_cast<int>((bx - grid_origin_x_) / grid_resolution_);
      int gy = static_cast<int>((by - grid_origin_y_) / grid_resolution_);
      if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_) {continue;}

      const SlopeCell & c = grid_[gy * grid_w_ + gx];
      if (!c.valid) {continue;}  // no info here -> don't overwrite other layers

      unsigned char cost;
      if (c.slope_deg >= lethal_slope_deg_) {
        cost = LETHAL_OBSTACLE;
      } else if (c.slope_deg <= traversable_slope_deg_) {
        cost = FREE_SPACE;
      } else {
        double t = (c.slope_deg - traversable_slope_deg_) /
          (lethal_slope_deg_ - traversable_slope_deg_);
        cost = static_cast<unsigned char>(t * (LETHAL_OBSTACLE - 1));
      }

      //   unsigned int idx = master_grid.getIndex(i, j);
      master_grid.setCost(i, j, cost);
    }
  }
}

void SlopeLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::fill(grid_.begin(), grid_.end(), SlopeCell());
  has_data_ = false;
}

}  // namespace rover_costmap_plugins

PLUGINLIB_EXPORT_CLASS(rover_costmap_plugins::SlopeLayer, nav2_costmap_2d::Layer)