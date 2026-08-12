#pragma once

#ifndef ROVER_COSTMAP_PLUGINS__SLOPE_LAYER_HPP_
#define ROVER_COSTMAP_PLUGINS__SLOPE_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rover_costmap_plugins
{

// One cell of our own coarse "slope grid", built in base_footprint and
// re-sampled into the master costmap (which lives in odom/map) each update.
struct SlopeCell
{
  int point_count = 0;
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  double sxx = 0.0, syy = 0.0, szz = 0.0, sxy = 0.0, sxz = 0.0, syz = 0.0;
  double slope_deg = -1.0;
  bool valid = false;
};

class SlopeLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SlopeLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void recomputeGrid();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters (defaults match the Indomitus rover_navigation setup) ---
  std::string cloud_topic_;         // "/zed2i/points"
  std::string base_frame_;          // "base_footprint" -- matches nav2_params.yaml
  double grid_resolution_;
  double grid_range_;
  double min_height_, max_height_;  // relative to base_frame_
  int min_points_per_cell_;
  double traversable_slope_deg_;
  double lethal_slope_deg_;
  double roughness_std_thresh_;

  // --- Internal state (grid is stored in base_frame_ coordinates) ---
  std::mutex data_mutex_;
  std::vector<SlopeCell> grid_;
  int grid_w_ = 0, grid_h_ = 0;
  double grid_origin_x_ = 0.0, grid_origin_y_ = 0.0;
  bool has_data_ = false;
};

}  // namespace rover_costmap_plugins

#endif  // ROVER_COSTMAP_PLUGINS__SLOPE_LAYER_HPP_