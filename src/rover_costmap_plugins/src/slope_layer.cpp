#include "rover_costmap_plugins/slope_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace rover_costmap_plugins
{

namespace
{
// Rotate vector (x,y,z) by unit quaternion (qw,qx,qy,qz): v' = q*v*q^-1.
void rotateByQuat(
  double qw, double qx, double qy, double qz,
  double x, double y, double z,
  double & ox, double & oy, double & oz)
{
  double tx = 2.0 * (qy * z - qz * y);
  double ty = 2.0 * (qz * x - qx * z);
  double tz = 2.0 * (qx * y - qy * x);
  ox = x + qw * tx + (qy * tz - qz * ty);
  oy = y + qw * ty + (qz * tx - qx * tz);
  oz = z + qw * tz + (qx * ty - qy * tx);
}
}  // namespace

SlopeLayer::SlopeLayer() {}

void SlopeLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("cloud_topic", rclcpp::ParameterValue(std::string("/zed2i/points")));
  declareParameter("base_frame", rclcpp::ParameterValue(std::string("base_footprint")));
  declareParameter("grid_resolution", rclcpp::ParameterValue(0.25));
  declareParameter("grid_range", rclcpp::ParameterValue(4.0));
  declareParameter("min_height", rclcpp::ParameterValue(-0.30));
  declareParameter("max_height", rclcpp::ParameterValue(2.40));
  declareParameter("min_points_per_cell", rclcpp::ParameterValue(7));
  declareParameter("traversable_slope_deg", rclcpp::ParameterValue(16.0));
  declareParameter("lethal_slope_deg", rclcpp::ParameterValue(28.0));
  declareParameter("roughness_std_thresh", rclcpp::ParameterValue(0.045));
  declareParameter("roughness_range_coeff", rclcpp::ParameterValue(0.015));
  declareParameter("self_filter_margin", rclcpp::ParameterValue(0.10));
  declareParameter("roughness_saturation_mult", rclcpp::ParameterValue(4.0));
  declareParameter("roughness_lethal", rclcpp::ParameterValue(false));
  declareParameter("lethal_min_support", rclcpp::ParameterValue(2));
  declareParameter("robot_clear_radius", rclcpp::ParameterValue(0.7));
  declareParameter("persist", rclcpp::ParameterValue(false));
  declareParameter("persist_half_extent", rclcpp::ParameterValue(50.0));
  declareParameter("tf_tolerance", rclcpp::ParameterValue(0.1));

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
  node->get_parameter(name_ + ".roughness_range_coeff", roughness_range_coeff_);
  node->get_parameter(name_ + ".self_filter_margin", self_filter_margin_);
  node->get_parameter(name_ + ".roughness_saturation_mult", roughness_saturation_mult_);
  node->get_parameter(name_ + ".roughness_lethal", roughness_lethal_);
  node->get_parameter(name_ + ".lethal_min_support", lethal_min_support_);
  node->get_parameter(name_ + ".robot_clear_radius", robot_clear_radius_);
  node->get_parameter(name_ + ".persist", persist_);
  node->get_parameter(name_ + ".persist_half_extent", persist_half_extent_);
  node->get_parameter(name_ + ".tf_tolerance", tf_tolerance_);

  clock_ = node->get_clock();
  logger_ = node->get_logger();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SlopeLayer::cloudCallback, this, std::placeholders::_1));

  debug_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    name_ + "/debug_cloud", rclcpp::SensorDataQoS());

  // Footprint is usually not set yet at init, so this may no-op;
  // onFootprintChanged() recomputes the self-filter box once it lands.
  updateSelfFilter();

  grid_w_ = static_cast<int>(std::ceil((2.0 * grid_range_) / grid_resolution_));
  grid_h_ = grid_w_;
  grid_origin_x_ = -grid_range_;
  grid_origin_y_ = -grid_range_;
  grid_.assign(grid_w_ * grid_h_, SlopeCell());

  if (persist_) {
    world_w_ = static_cast<int>(std::ceil((2.0 * persist_half_extent_) / grid_resolution_));
    world_h_ = world_w_;
    world_origin_x_ = -persist_half_extent_;
    world_origin_y_ = -persist_half_extent_;
    world_cost_.assign(world_w_ * world_h_, NO_INFORMATION);
  }

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "SlopeLayer '%s' up: topic=%s base_frame=%s traversable<=%.1f lethal>=%.1f deg",
    name_.c_str(), cloud_topic_.c_str(), base_frame_.c_str(),
    traversable_slope_deg_, lethal_slope_deg_);
}

void SlopeLayer::updateSelfFilter()
{
  const auto & footprint = layered_costmap_->getFootprint();
  if (footprint.empty()) {
    RCLCPP_WARN(
      logger_,
      "SlopeLayer '%s': footprint unavailable, self-filter disabled until it is set",
      name_.c_str());
    return;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & pt : footprint) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  self_filter_min_x_ = min_x - self_filter_margin_;
  self_filter_max_x_ = max_x + self_filter_margin_;
  self_filter_min_y_ = min_y - self_filter_margin_;
  self_filter_max_y_ = max_y + self_filter_margin_;
  self_filter_valid_ = true;
  RCLCPP_INFO(
    logger_,
    "SlopeLayer '%s': self-filter box x[%.2f, %.2f] y[%.2f, %.2f]",
    name_.c_str(), self_filter_min_x_, self_filter_max_x_,
    self_filter_min_y_, self_filter_max_y_);
}

void SlopeLayer::onFootprintChanged()
{
  updateSelfFilter();
}

void SlopeLayer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_in_base;
  double base_tx = 0.0, base_ty = 0.0, base_yaw = 0.0;
  double grav_qw = 1.0, grav_qx = 0.0, grav_qy = 0.0, grav_qz = 0.0;
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      base_frame_, msg->header.frame_id, msg->header.stamp,
      rclcpp::Duration::from_seconds(tf_tolerance_));
    tf2::doTransform(*msg, cloud_in_base, tf);

    // Pose at the cloud's timestamp, reused in updateCosts so the grid is
    // reprojected with the pose it was built from.
    geometry_msgs::msg::TransformStamped base_tf = tf_buffer_->lookupTransform(
      base_frame_, layered_costmap_->getGlobalFrameID(), msg->header.stamp,
      rclcpp::Duration::from_seconds(tf_tolerance_));
    base_tx = base_tf.transform.translation.x;
    base_ty = base_tf.transform.translation.y;
    const auto & q = base_tf.transform.rotation;
    base_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // base_tf's rotation is global_frame -> base_frame_; its conjugate
    // (base_frame_ -> global_frame) is what recomputeGrid needs.
    grav_qw = q.w; grav_qx = -q.x; grav_qy = -q.y; grav_qz = -q.z;
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "TF lookup failed for SlopeLayer '%s': %s", name_.c_str(), ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  captured_tx_ = base_tx;
  captured_ty_ = base_ty;
  captured_yaw_ = base_yaw;
  captured_gqw_ = grav_qw;
  captured_gqx_ = grav_qx;
  captured_gqy_ = grav_qy;
  captured_gqz_ = grav_qz;
  std::fill(grid_.begin(), grid_.end(), SlopeCell());

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_in_base, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_in_base, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_in_base, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    float x = *it_x, y = *it_y, z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
    if (z < min_height_ || z > max_height_) {continue;}
    if (std::abs(x) > grid_range_ || std::abs(y) > grid_range_) {continue;}
    // Rocker/wheel self-returns: skip points inside the robot's own footprint.
    if (x > self_filter_min_x_ && x < self_filter_max_x_ &&
        y > self_filter_min_y_ && y < self_filter_max_y_) {continue;}

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
  accumulateWorld();
  has_data_ = true;
  publishDebugCloud();
}

unsigned char SlopeLayer::slopeToCost(double slope_deg) const
{
  if (slope_deg >= lethal_slope_deg_) {return LETHAL_OBSTACLE;}
  if (slope_deg <= traversable_slope_deg_) {return FREE_SPACE;}
  double t = (slope_deg - traversable_slope_deg_) /
    (lethal_slope_deg_ - traversable_slope_deg_);
  return static_cast<unsigned char>(t * (LETHAL_OBSTACLE - 1));
}

void SlopeLayer::accumulateWorld()
{
  if (!persist_ || world_cost_.empty()) {return;}

  // grid_ is in base_frame_; captured_{tx,ty,yaw} is global_frame -> base_frame_
  // (p_base = R(yaw) p_world + t), so world = R(-yaw) (p_base - t).
  const double cos_yaw = std::cos(captured_yaw_), sin_yaw = std::sin(captured_yaw_);
  for (int gy = 0; gy < grid_h_; ++gy) {
    for (int gx = 0; gx < grid_w_; ++gx) {
      const SlopeCell & c = grid_[gy * grid_w_ + gx];
      if (!c.valid) {continue;}  // only cells observed this cloud
      const double bx = grid_origin_x_ + (gx + 0.5) * grid_resolution_;
      const double by = grid_origin_y_ + (gy + 0.5) * grid_resolution_;
      const double dx = bx - captured_tx_, dy = by - captured_ty_;
      const double wx = cos_yaw * dx + sin_yaw * dy;
      const double wy = -sin_yaw * dx + cos_yaw * dy;
      int wgx = static_cast<int>((wx - world_origin_x_) / grid_resolution_);
      int wgy = static_cast<int>((wy - world_origin_y_) / grid_resolution_);
      if (wgx < 0 || wgx >= world_w_ || wgy < 0 || wgy >= world_h_) {continue;}
      world_cost_[wgy * world_w_ + wgx] = slopeToCost(c.slope_deg);
    }
  }
}

void SlopeLayer::recomputeGrid()
{
  for (int gy = 0; gy < grid_h_; ++gy) {
    for (int gx = 0; gx < grid_w_; ++gx) {
      SlopeCell & c = grid_[gy * grid_w_ + gx];
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

      Eigen::Matrix3d cov;
      cov << cxx, cxy, cxz,
             cxy, cyy, cyz,
             cxz, cyz, czz;

      // Smallest eigenvalue's eigenvector is the plane normal; the eigenvalue
      // itself is the planar residual variance (roughness).
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
      solver.computeDirect(cov);
      const Eigen::Vector3d normal = solver.eigenvectors().col(0);
      double vx = normal.x(), vy = normal.y(), vz = normal.z();
      double planar_residual = std::sqrt(std::max(0.0, solver.eigenvalues()(0)));

      // Rotate the fitted normal from base_frame_ into the global costmap
      // frame so slope is measured against gravity, not vehicle attitude.
      double gvx, gvy, gvz;
      rotateByQuat(
        captured_gqw_, captured_gqx_, captured_gqy_, captured_gqz_,
        vx, vy, vz, gvx, gvy, gvz);
      if (gvz < 0) {gvx = -gvx; gvy = -gvy; gvz = -gvz;}

      double slope_rad = std::acos(std::clamp(gvz, -1.0, 1.0));
      c.slope_deg = slope_rad * 180.0 / M_PI;

      // Widen the residual threshold with range (stereo noise grows with
      // distance/angle) and with point sparsity (few points -> unreliable fit).
      double cell_x = grid_origin_x_ + (gx + 0.5) * grid_resolution_;
      double cell_y = grid_origin_y_ + (gy + 0.5) * grid_resolution_;
      double range_m = std::sqrt(cell_x * cell_x + cell_y * cell_y);
      double effective_thresh = roughness_std_thresh_ + roughness_range_coeff_ * range_m;

      const int sparsity_span = min_points_per_cell_ * 2;
      if (sparsity_span > 0 && c.point_count < min_points_per_cell_ + sparsity_span) {
        double sparsity_factor = static_cast<double>(
          min_points_per_cell_ + sparsity_span - c.point_count) / sparsity_span;
        effective_thresh *= (1.0 + sparsity_factor);
      }

      c.mean_z = mz;
      c.residual = planar_residual;
      c.valid = true;
      if (planar_residual > effective_thresh) {
        // Map roughness into the graded cost band; saturates to lethal only
        // well past the threshold.
        const double slope_span = lethal_slope_deg_ - traversable_slope_deg_;
        double excess_ratio = (planar_residual - effective_thresh) / effective_thresh;
        double roughness_slope = traversable_slope_deg_ +
          std::min(excess_ratio, roughness_saturation_mult_) *
          slope_span / roughness_saturation_mult_;
        // Roughness is a noisy single-cell measure. Unless roughness_lethal is
        // set, cap it just below lethal so it strongly avoids but never hard-
        // blocks; real steep terrain still goes lethal via the geometric slope.
        if (!roughness_lethal_) {
          roughness_slope = std::min(
            roughness_slope, traversable_slope_deg_ + 0.98 * slope_span);
        }
        c.slope_deg = std::max(c.slope_deg, roughness_slope);
      }

      // Trust the ground right around the rover: near-field stereo is the
      // noisiest (grazing angle) and a false mark here boxes the rover in.
      // Force these cells free; the 2D lidar still catches real close obstacles.
      if (robot_clear_radius_ > 0.0 && range_m < robot_clear_radius_) {
        c.slope_deg = std::min(c.slope_deg, traversable_slope_deg_);
      }
    }
  }

  // Support gate: a lethal cell must have enough non-traversable neighbors to
  // be trusted. A real steep patch spans several cells; isolated noise gets
  // demoted to just-below-lethal (strong avoid, not a hard block).
  if (lethal_min_support_ > 0) {
    std::vector<char> steep(grid_.size(), 0);
    for (size_t idx = 0; idx < grid_.size(); ++idx) {
      const SlopeCell & c = grid_[idx];
      if (c.valid && c.slope_deg >= traversable_slope_deg_) {steep[idx] = 1;}
    }
    const double demoted =
      traversable_slope_deg_ + 0.98 * (lethal_slope_deg_ - traversable_slope_deg_);
    for (int gy = 0; gy < grid_h_; ++gy) {
      for (int gx = 0; gx < grid_w_; ++gx) {
        SlopeCell & c = grid_[gy * grid_w_ + gx];
        if (!c.valid || c.slope_deg < lethal_slope_deg_) {continue;}
        int support = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {continue;}
            int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= grid_w_ || ny < 0 || ny >= grid_h_) {continue;}
            support += steep[ny * grid_w_ + nx];
          }
        }
        if (support < lethal_min_support_) {c.slope_deg = std::min(c.slope_deg, demoted);}
      }
    }
  }
}

void SlopeLayer::publishDebugCloud()
{
  if (!debug_pub_ || debug_pub_->get_subscription_count() == 0) {return;}

  size_t valid_count = 0;
  for (const auto & c : grid_) {if (c.valid) {valid_count++;}}

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = base_frame_;
  msg.header.stamp = clock_->now();

  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "slope_deg", 1, sensor_msgs::msg::PointField::FLOAT32,
    "residual", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(valid_count);

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> it_s(msg, "slope_deg");
  sensor_msgs::PointCloud2Iterator<float> it_r(msg, "residual");

  for (int gy = 0; gy < grid_h_; ++gy) {
    for (int gx = 0; gx < grid_w_; ++gx) {
      const SlopeCell & c = grid_[gy * grid_w_ + gx];
      if (!c.valid) {continue;}
      *it_x = static_cast<float>(grid_origin_x_ + (gx + 0.5) * grid_resolution_);
      *it_y = static_cast<float>(grid_origin_y_ + (gy + 0.5) * grid_resolution_);
      *it_z = static_cast<float>(c.mean_z);
      *it_s = static_cast<float>(c.slope_deg);
      *it_r = static_cast<float>(c.residual);
      ++it_x; ++it_y; ++it_z; ++it_s; ++it_r;
    }
  }

  debug_pub_->publish(msg);
}

void SlopeLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // Remember the live robot pose (costmap frame) for the under-rover clear in
  // updateCosts. Store it before the has_data_ gate so it is always current.
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  have_robot_pose_ = true;

  if (!enabled_ || !has_data_) {return;}
  const double r = full_repaint_ ? persist_half_extent_ : grid_range_;
  *min_x = std::min(*min_x, robot_x - r);
  *min_y = std::min(*min_y, robot_y - r);
  *max_x = std::max(*max_x, robot_x + r);
  *max_y = std::max(*max_y, robot_y + r);
}

void SlopeLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_data_) {return;}
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Never let this layer raise cost right under the rover. A cell it drove onto
  // (from live slope or persisted memory) can't be re-observed by the forward
  // camera, so it would sit lethal forever and trap the rover on its own mark.
  const double clear_sq = robot_clear_radius_ * robot_clear_radius_;

  // Persist path: world memory is in the costmap's global frame, so master
  // cells map straight into it. Unobserved cells (NO_INFORMATION) are skipped.
  if (persist_) {
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        int wgx = static_cast<int>((wx - world_origin_x_) / grid_resolution_);
        int wgy = static_cast<int>((wy - world_origin_y_) / grid_resolution_);
        if (wgx < 0 || wgx >= world_w_ || wgy < 0 || wgy >= world_h_) {continue;}
        // Under the rover: it drove here, so the cell is traversable. Correct the
        // memory to free (permanent, so it doesn't reappear in the rover's wake)
        // and leave the master cell to the other layers.
        if (have_robot_pose_ && robot_clear_radius_ > 0.0) {
          const double dx = wx - robot_x_, dy = wy - robot_y_;
          if (dx * dx + dy * dy < clear_sq) {
            world_cost_[wgy * world_w_ + wgx] = FREE_SPACE;
            continue;
          }
        }
        unsigned char cost = world_cost_[wgy * world_w_ + wgx];
        if (cost == NO_INFORMATION) {continue;}
        unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost == NO_INFORMATION || cost > old_cost) {
          master_grid.setCost(i, j, cost);
        }
      }
    }
    full_repaint_ = false;  // full memory is back on the master; resume cheap bounds
    return;
  }

  const double tx = captured_tx_, ty = captured_ty_, yaw = captured_yaw_;
  const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      if (have_robot_pose_ && robot_clear_radius_ > 0.0) {
        const double dx = wx - robot_x_, dy = wy - robot_y_;
        if (dx * dx + dy * dy < clear_sq) {continue;}
      }

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

      // Only raise cost over what earlier layers already wrote here.
      unsigned char old_cost = master_grid.getCost(i, j);
      if (old_cost == NO_INFORMATION || cost > old_cost) {
        master_grid.setCost(i, j, cost);
      }
    }
  }
}

void SlopeLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::fill(grid_.begin(), grid_.end(), SlopeCell());
  // Persist mode keeps its world memory across a clear (wiping it blanks the
  // global map and the rover gets lost; re-observation corrects stale cells).
  // The clear blanked the master, so repaint the full memory next update.
  if (persist_) {
    full_repaint_ = true;
  } else {
    has_data_ = false;
  }
}

}  // namespace rover_costmap_plugins

PLUGINLIB_EXPORT_CLASS(rover_costmap_plugins::SlopeLayer, nav2_costmap_2d::Layer)