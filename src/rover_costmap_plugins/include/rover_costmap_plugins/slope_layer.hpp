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
struct SlopeCell
{
  int point_count = 0;
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  double sxx = 0.0, syy = 0.0, szz = 0.0, sxy = 0.0, sxz = 0.0, syz = 0.0;
  double slope_deg = -1.0;
  double mean_z = 0.0;
  double residual = 0.0;
  bool valid = false;
};
class SlopeLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SlopeLayer();
  void onInitialize() override;
  void onFootprintChanged() override;
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
  void publishDebugCloud();
  // Maps a cell slope angle to a costmap cost using the traversable/lethal band.
  unsigned char slopeToCost(double slope_deg) const;
  // Stamps the latest per-cloud grid into the persistent world-frame memory
  // (persist_ mode only), so obstacles survive the camera looking away.
  void accumulateWorld();
  // (Re)computes the self-return exclusion box from the current footprint.
  void updateSelfFilter();
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Clock::SharedPtr clock_;
  // Cached at init so the cloud callback never has to lock the parent node's
  // weak_ptr (which can be null during shutdown/reset -> segfault).
  rclcpp::Logger logger_{rclcpp::get_logger("SlopeLayer")};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string cloud_topic_;          // input point cloud topic
  std::string base_frame_;           // frame the slope grid is built and stored in
  double grid_resolution_;           // grid cell size, meters
  double grid_range_;                // grid half-extent from robot origin, meters
  double min_height_, max_height_;   // point height filter, relative to base_frame_
  int min_points_per_cell_;          // minimum points for a cell to be considered valid
  double traversable_slope_deg_;     // slope at/below this reads as free space
  double lethal_slope_deg_;          // slope at/above this reads as lethal
  double roughness_std_thresh_;      // planar residual threshold at zero range
  double roughness_range_coeff_;     // residual threshold widening per meter of range
  double self_filter_margin_;        // margin added around the footprint for self-return exclusion
  double roughness_saturation_mult_; // residual multiple past threshold at which roughness saturates
  bool roughness_lethal_ = false;    // if false, roughness cost is capped just below lethal
  int lethal_min_support_ = 2;       // min non-traversable 8-neighbors for a cell to stay lethal
  double robot_clear_radius_ = 0.7;  // within this range of base origin, slope is never lethal
  // Persistence: when true, marks are accumulated in a fixed world-frame grid
  // that is not wiped each cloud, so an obstacle stays on the costmap after the
  // camera rotates away from it
  bool persist_ = false;
  double persist_half_extent_ = 50.0;  // world grid half-size (m) around the global-frame origin
  std::vector<unsigned char> world_cost_;  // persistent per-cell cost, NO_INFORMATION = unobserved
  int world_w_ = 0, world_h_ = 0;
  double world_origin_x_ = 0.0, world_origin_y_ = 0.0;
  // Set on reset() so the next update repaints the whole world memory once
  // (a costmap clear blanks the master; this puts remembered obstacles back).
  bool full_repaint_ = false;
  // Live robot position in the costmap frame, captured in updateBounds and used
  // in updateCosts to clear the cells under the rover (a persisted mark the
  // rover has driven onto can never be re-observed, so it would trap it).
  double robot_x_ = 0.0, robot_y_ = 0.0;
  bool have_robot_pose_ = false;
  // Grid is accumulated in base_frame_ coordinates from the latest cloud,
  // then re-sampled into the master costmap's own frame (odom/map) in
  // updateCosts.
  std::mutex data_mutex_;
  std::vector<SlopeCell> grid_;
  int grid_w_ = 0, grid_h_ = 0;
  double grid_origin_x_ = 0.0, grid_origin_y_ = 0.0;
  bool has_data_ = false;
  // Pose captured alongside grid_ in cloudCallback, reused in updateCosts
  // so grid_ is always reprojected with the pose it was built from.
  double captured_tx_ = 0.0, captured_ty_ = 0.0, captured_yaw_ = 0.0;
  // Rotation from base_frame_ to the global costmap frame, captured
  // alongside grid_ and used in recomputeGrid.
  double captured_gqw_ = 1.0, captured_gqx_ = 0.0, captured_gqy_ = 0.0, captured_gqz_ = 0.0;
  // Footprint bounding box + margin, in base_frame_. Points inside are
  // treated as rover self-returns and excluded.
  double self_filter_min_x_ = 0.0, self_filter_max_x_ = 0.0;
  double self_filter_min_y_ = 0.0, self_filter_max_y_ = 0.0;
  bool self_filter_valid_ = false;

  double tf_tolerance_ = 0.1;
};
}  // namespace rover_costmap_plugins
#endif  // ROVER_COSTMAP_PLUGINS__SLOPE_LAYER_HPP_