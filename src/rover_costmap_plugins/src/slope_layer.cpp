#include "rover_costmap_plugins/slope_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rover_costmap_plugins/slope_math.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace rover_costmap_plugins
{

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
  declareParameter("lethal_min_support", rclcpp::ParameterValue(1));
  declareParameter("robot_clear_radius", rclcpp::ParameterValue(0.7));
  declareParameter("min_plane_spread", rclcpp::ParameterValue(1.0e-4));
  declareParameter("mark_unobserved_unknown", rclcpp::ParameterValue(false));
  declareParameter("cloud_timeout", rclcpp::ParameterValue(2.0));
  declareParameter("persist", rclcpp::ParameterValue(false));
  declareParameter("persist_half_extent", rclcpp::ParameterValue(50.0));
  declareParameter("persist_update_range", rclcpp::ParameterValue(6.0));
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
  node->get_parameter(name_ + ".min_plane_spread", min_plane_spread_);
  node->get_parameter(name_ + ".mark_unobserved_unknown", mark_unobserved_unknown_);
  node->get_parameter(name_ + ".cloud_timeout", cloud_timeout_);
  node->get_parameter(name_ + ".persist", persist_);
  node->get_parameter(name_ + ".persist_half_extent", persist_half_extent_);
  node->get_parameter(name_ + ".persist_update_range", persist_update_range_);
  node->get_parameter(name_ + ".tf_tolerance", tf_tolerance_);

  clock_ = node->get_clock();
  logger_ = node->get_logger();
  last_cloud_time_ = clock_->now();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SlopeLayer::cloudCallback, this, std::placeholders::_1));

  debug_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    name_ + "/debug_cloud", rclcpp::SensorDataQoS());

  // Footprint is usually empty at init; onFootprintChanged() redoes this later.
  updateSelfFilter();

  grid_w_ = static_cast<int>(std::ceil((2.0 * grid_range_) / grid_resolution_));
  grid_h_ = grid_w_;
  grid_origin_x_ = -grid_range_;
  grid_origin_y_ = -grid_range_;
  grid_.assign(grid_w_ * grid_h_, SlopeCell());

  if (persist_) {
    world_w_ = static_cast<int>(std::ceil((2.0 * persist_half_extent_) / grid_resolution_));
    world_h_ = world_w_;
    world_cost_.assign(
      static_cast<size_t>(world_w_) * static_cast<size_t>(world_h_), NO_INFORMATION);
    // Origin is placed on the first observed rover pose in accumulateWorld().
    world_origin_set_ = false;
  }

  // Without unknown tracking the costmap starts cells free, so the marks below
  // would be overwritten and unseen ground would read as traversable.
  if (mark_unobserved_unknown_ && !layered_costmap_->isTrackingUnknown()) {
    RCLCPP_WARN(
      logger_,
      "SlopeLayer '%s': mark_unobserved_unknown is set but this costmap does not "
      "track unknown space; unobserved terrain will still read as free.",
      name_.c_str());
  }

  // Nothing observed yet, so the layer is not current until the first cloud.
  current_ = false;

  RCLCPP_INFO(
    logger_,
    "SlopeLayer '%s' up: topic=%s base_frame=%s traversable<=%.1f lethal>=%.1f deg persist=%s",
    name_.c_str(), cloud_topic_.c_str(), base_frame_.c_str(),
    traversable_slope_deg_, lethal_slope_deg_, persist_ ? "true" : "false");
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

bool SlopeLayer::cloudIsStale() const
{
  if (!have_cloud_) {return true;}
  return slope_math::sensorIsStale(
    (clock_->now() - last_cloud_time_).seconds(), cloud_timeout_);
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

    // Pose at the cloud's own time, so the grid is reprojected with the pose
    // it was built from.
    geometry_msgs::msg::TransformStamped base_tf = tf_buffer_->lookupTransform(
      base_frame_, layered_costmap_->getGlobalFrameID(), msg->header.stamp,
      rclcpp::Duration::from_seconds(tf_tolerance_));
    base_tx = base_tf.transform.translation.x;
    base_ty = base_tf.transform.translation.y;
    const auto & q = base_tf.transform.rotation;
    base_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Conjugate flips it to base -> costmap frame, which is what slope needs.
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
    // Points inside the footprint are the rover's own wheels and body.
    if (self_filter_valid_ &&
        x > self_filter_min_x_ && x < self_filter_max_x_ &&
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
  have_cloud_ = true;
  last_cloud_time_ = clock_->now();
  publishDebugCloud();
}

void SlopeLayer::accumulateWorld()
{
  if (!persist_ || world_cost_.empty()) {return;}

  // grid_ is in base_frame_; captured_{tx,ty,yaw} is global_frame -> base_frame_
  // (p_base = R(yaw) p_world + t), so world = R(-yaw) (p_base - t).
  const double cos_yaw = std::cos(captured_yaw_), sin_yaw = std::sin(captured_yaw_);

  if (!world_origin_set_) {
    // Center on the rover, so the memory covers the arena wherever it starts.
    const double robot_wx = cos_yaw * (-captured_tx_) + sin_yaw * (-captured_ty_);
    const double robot_wy = -sin_yaw * (-captured_tx_) + cos_yaw * (-captured_ty_);
    world_origin_x_ = robot_wx - persist_half_extent_;
    world_origin_y_ = robot_wy - persist_half_extent_;
    world_origin_set_ = true;
    RCLCPP_INFO(
      logger_,
      "SlopeLayer '%s': terrain memory centered on (%.2f, %.2f), half-extent %.1f m",
      name_.c_str(), robot_wx, robot_wy, persist_half_extent_);
  }

  size_t dropped = 0;
  for (int gy = 0; gy < grid_h_; ++gy) {
    for (int gx = 0; gx < grid_w_; ++gx) {
      const SlopeCell & c = grid_[gy * grid_w_ + gx];
      if (!c.valid) {continue;}     // no plane, nothing to store
      if (c.near_field) {continue;} // too noisy to keep
      const double bx = grid_origin_x_ + (gx + 0.5) * grid_resolution_;
      const double by = grid_origin_y_ + (gy + 0.5) * grid_resolution_;
      double wx, wy;
      slope_math::baseToWorld(bx, by, captured_tx_, captured_ty_, cos_yaw, sin_yaw, wx, wy);
      int wgx = 0, wgy = 0;
      if (!slope_math::worldCell(
          wx, wy, world_origin_x_, world_origin_y_, grid_resolution_,
          world_w_, world_h_, wgx, wgy))
      {
        dropped++;
        continue;
      }
      // Overwrite, so seeing a spot again corrects it while unseen spots keep
      // their value.
      world_cost_[static_cast<size_t>(wgy) * world_w_ + wgx] =
        slope_math::slopeToCost(c.slope_deg, traversable_slope_deg_, lethal_slope_deg_);
    }
  }

  if (dropped > 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 10000,
      "SlopeLayer '%s': %zu terrain cells fell outside the %.1f m memory and were "
      "dropped; raise persist_half_extent to cover the arena.",
      name_.c_str(), dropped, persist_half_extent_);
  }
}

void SlopeLayer::recomputeGrid()
{
  for (int gy = 0; gy < grid_h_; ++gy) {
    for (int gx = 0; gx < grid_w_; ++gx) {
      SlopeCell & c = grid_[gy * grid_w_ + gx];

      slope_math::CellMoments moments;
      moments.count = c.point_count;
      moments.sum_x = c.sum_x; moments.sum_y = c.sum_y; moments.sum_z = c.sum_z;
      moments.sxx = c.sxx; moments.syy = c.syy; moments.szz = c.szz;
      moments.sxy = c.sxy; moments.sxz = c.sxz; moments.syz = c.syz;

      // Drops cells whose points give no usable plane.
      const slope_math::PlaneFit fit =
        slope_math::fitPlane(moments, min_points_per_cell_, min_plane_spread_);
      if (!fit.valid) {
        c.valid = false;
        continue;
      }

      // Measured against gravity, so rover tilt is not read as slope.
      c.slope_deg = slope_math::slopeDegFromNormal(
        fit.nx, fit.ny, fit.nz,
        captured_gqw_, captured_gqx_, captured_gqy_, captured_gqz_);

      const double cell_x = grid_origin_x_ + (gx + 0.5) * grid_resolution_;
      const double cell_y = grid_origin_y_ + (gy + 0.5) * grid_resolution_;
      const double range_m = std::sqrt(cell_x * cell_x + cell_y * cell_y);

      const double effective_thresh = slope_math::effectiveRoughnessThreshold(
        roughness_std_thresh_, roughness_range_coeff_, range_m,
        c.point_count, min_points_per_cell_);

      c.mean_z = fit.mean_z;
      c.residual = fit.residual;
      c.valid = true;
      // Stereo close to the body is the noisiest, so flag it as untrusted.
      c.near_field = (robot_clear_radius_ > 0.0 && range_m < robot_clear_radius_);

      if (fit.residual > effective_thresh) {
        c.slope_deg = std::max(
          c.slope_deg,
          slope_math::roughnessToSlopeDeg(
            fit.residual, effective_thresh, traversable_slope_deg_, lethal_slope_deg_,
            roughness_saturation_mult_, roughness_lethal_));
      }
    }
  }

  // A lethal cell needs steep neighbours to be believed. Real hazards cover
  // several cells, single noisy cells do not.
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
  // Stored before the gates below so updateCosts always has a fresh pose.
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  have_robot_pose_ = true;

  // Tells nav2 the terrain data is no longer live, instead of passing off an
  // old grid as fresh.
  const bool stale = cloudIsStale();
  current_ = !stale;
  if (stale != cloud_stale_) {
    cloud_stale_ = stale;
    if (stale) {
      RCLCPP_WARN(
        logger_,
        "SlopeLayer '%s': no point cloud on %s for over %.1f s -- terrain data is stale.",
        name_.c_str(), cloud_topic_.c_str(), cloud_timeout_);
    } else {
      RCLCPP_INFO(
        logger_, "SlopeLayer '%s': point cloud restored.", name_.c_str());
    }
  }

  if (!enabled_ || !has_data_) {return;}

  // Covers the sensed area, and in persist mode also the obstacle layer's
  // clearing reach, so nothing it clears is left unpainted.
  const double r = persist_ ? std::max(grid_range_, persist_update_range_) : grid_range_;
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

  // Terrain right under the rover is not painted: the camera cannot see it
  // again to correct it. The memory keeps it, since it may be real.
  const double clear_sq = robot_clear_radius_ * robot_clear_radius_;
  const bool skip_near_robot = have_robot_pose_ && robot_clear_radius_ > 0.0;

  // The memory is already in the costmap frame, so cells map straight into it.
  if (persist_) {
    if (!world_origin_set_) {return;}
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        if (skip_near_robot) {
          const double dx = wx - robot_x_, dy = wy - robot_y_;
          if (dx * dx + dy * dy < clear_sq) {
            // The rover stands here, so the ground is passable. Only unknown
            // cells are filled; a real obstacle from another layer stays.
            if (master_grid.getCost(i, j) == NO_INFORMATION) {
              master_grid.setCost(i, j, FREE_SPACE);
            }
            continue;
          }
        }
        int wgx = 0, wgy = 0;
        if (!slope_math::worldCell(
            wx, wy, world_origin_x_, world_origin_y_, grid_resolution_,
            world_w_, world_h_, wgx, wgy))
        {
          continue;
        }
        const unsigned char cost = world_cost_[static_cast<size_t>(wgy) * world_w_ + wgx];
        if (cost == NO_INFORMATION) {continue;}  // never seen, so claim nothing
        const unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost == NO_INFORMATION || cost > old_cost) {
          master_grid.setCost(i, j, cost);
        }
      }
    }
    return;
  }

  const double tx = captured_tx_, ty = captured_ty_, yaw = captured_yaw_;
  const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
  const bool mark_unknown = mark_unobserved_unknown_;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      if (skip_near_robot) {
        const double dx = wx - robot_x_, dy = wy - robot_y_;
        if (dx * dx + dy * dy < clear_sq) {
          // The rover stands here, so the ground is passable. Only unknown
          // cells are filled; a real obstacle from another layer stays.
          if (master_grid.getCost(i, j) == NO_INFORMATION) {
            master_grid.setCost(i, j, FREE_SPACE);
          }
          continue;
        }
      }

      double bx, by;
      slope_math::worldToBase(wx, wy, tx, ty, cos_yaw, sin_yaw, bx, by);

      int gx = 0, gy = 0;
      if (!slope_math::worldCell(
          bx, by, grid_origin_x_, grid_origin_y_, grid_resolution_,
          grid_w_, grid_h_, gx, gy))
      {
        continue;
      }

      const SlopeCell & c = grid_[gy * grid_w_ + gx];
      if (!c.valid) {
        // Marked unknown so a stereo hole, an occlusion or a drop-off is not
        // taken for flat ground.
        if (mark_unknown && master_grid.getCost(i, j) == FREE_SPACE) {
          master_grid.setCost(i, j, NO_INFORMATION);
        }
        continue;
      }
      if (c.near_field) {continue;}  // measured but too noisy to paint

      const unsigned char cost =
        slope_math::slopeToCost(c.slope_deg, traversable_slope_deg_, lethal_slope_deg_);

      // Only raises cost, never lowers what other layers wrote.
      const unsigned char old_cost = master_grid.getCost(i, j);
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
  // The memory goes too, so recovery can really drop a false obstacle.
  std::fill(world_cost_.begin(), world_cost_.end(), NO_INFORMATION);
  has_data_ = false;
  current_ = false;
}

}  // namespace rover_costmap_plugins

PLUGINLIB_EXPORT_CLASS(rover_costmap_plugins::SlopeLayer, nav2_costmap_2d::Layer)
