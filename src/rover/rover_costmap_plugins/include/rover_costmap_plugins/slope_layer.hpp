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
  // False when the cell has no usable plane: unobserved or degenerate.
  bool valid = false;
  // Cell sits within robot_clear_radius, where stereo is too noisy to trust.
  bool near_field = false;
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
  // Writes the current grid into the world memory (persist_ only).
  void accumulateWorld();
  // (Re)computes the self-return exclusion box from the current footprint.
  void updateSelfFilter();
  // True when no cloud arrived within cloud_timeout_.
  bool cloudIsStale() const;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Clock::SharedPtr clock_;
  // Cached: locking the node weak_ptr in the callback can hit a null on shutdown.
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
  int lethal_min_support_ = 1;       // min non-traversable 8-neighbors for a cell to stay lethal
  double robot_clear_radius_ = 0.7;  // radius around the rover that is not trusted or painted
  double min_plane_spread_ = 1.0e-4; // min in-plane variance (m^2) to accept a plane fit
  // Writes unobserved cells as NO_INFORMATION, keeping "no data" apart from
  // "flat". Needs track_unknown_space, else the costmap starts them free.
  bool mark_unobserved_unknown_ = false;
  // Cloud age (s) after which the layer stops calling itself current.
  double cloud_timeout_ = 2.0;
  rclcpp::Time last_cloud_time_;
  bool have_cloud_ = false;
  bool cloud_stale_ = false;
  // Keeps terrain in a world grid that is not wiped each cloud, so it survives
  // the camera turning away. reset() clears it.
  bool persist_ = false;
  double persist_half_extent_ = 50.0;  // world grid half-size (m) around its origin
  // Bounds radius in persist mode. Must cover the obstacle layer's clearing
  // reach, or that layer clears terrain this one never repaints.
  double persist_update_range_ = 6.0;
  std::vector<unsigned char> world_cost_;  // persistent per-cell cost, NO_INFORMATION = unobserved
  int world_w_ = 0, world_h_ = 0;
  // Centered on the rover's first observed pose, so the arena is covered
  // wherever it starts.
  double world_origin_x_ = 0.0, world_origin_y_ = 0.0;
  bool world_origin_set_ = false;
  // Live rover position, used to skip painting under it. A mark it stands on
  // is never seen again by the camera, so it would trap the rover.
  double robot_x_ = 0.0, robot_y_ = 0.0;
  bool have_robot_pose_ = false;
  // Built in base_frame_ from the latest cloud, then resampled into the
  // costmap frame in updateCosts.
  std::mutex data_mutex_;
  std::vector<SlopeCell> grid_;
  int grid_w_ = 0, grid_h_ = 0;
  double grid_origin_x_ = 0.0, grid_origin_y_ = 0.0;
  bool has_data_ = false;
  // Pose the grid was built at, so it is reprojected with that same pose.
  double captured_tx_ = 0.0, captured_ty_ = 0.0, captured_yaw_ = 0.0;
  // Rotation from base_frame_ to the costmap frame, used to measure slope.
  double captured_gqw_ = 1.0, captured_gqx_ = 0.0, captured_gqy_ = 0.0, captured_gqz_ = 0.0;
  // Footprint box plus margin. Points inside it are the rover seeing itself.
  double self_filter_min_x_ = 0.0, self_filter_max_x_ = 0.0;
  double self_filter_min_y_ = 0.0, self_filter_max_y_ = 0.0;
  bool self_filter_valid_ = false;

  double tf_tolerance_ = 0.1;
};
}  // namespace rover_costmap_plugins
#endif  // ROVER_COSTMAP_PLUGINS__SLOPE_LAYER_HPP_
