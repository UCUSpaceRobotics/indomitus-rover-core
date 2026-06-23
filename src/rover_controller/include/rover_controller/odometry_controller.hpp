#pragma once

#include <array>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>

#include "controller_interface/controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace rover_controller {

constexpr std::size_t ODOM_NUM_WHEELS = 4;

// Wheel indices
constexpr std::size_t ODOM_FL = 0;
constexpr std::size_t ODOM_FR = 1;
constexpr std::size_t ODOM_RL = 2;
constexpr std::size_t ODOM_RR = 3;

// ─────────────────────────────────────────────────────────────────────────────
// RoverOdometryController
//
// ros2_control controller plugin that estimates 2D pose from wheel encoders.
//
// State interfaces consumed (read-only):
//   fl/fr/bl/br  wheel_mount_joint  — position [rad]   (steering angle)
//   fl/fr/bl/br  wheel_joint        — position [rad]   (drive encoder, accumulated)
//
// Publishes:
//   /odom  [nav_msgs/Odometry]
//   TF:    odom → base_link
//
// Algorithm:
//   1. Diff drive encoder positions between cycles → Δθ_wheel → v_wheel = r·Δθ/dt
//   2. Decompose each wheel velocity into (vx_i, vy_i) using steering angle.
//   3. Solve over-determined system A·[vx, vy, wz]ᵀ = b via pseudoinverse (8×3).
//   4. Integrate chassis velocity with exact exp-map for curved paths.
// ─────────────────────────────────────────────────────────────────────────────

class RoverOdometryController : public controller_interface::ControllerInterface
{
public:
    RoverOdometryController();

    // ── ControllerInterface overrides ──────────────────────────────────────────

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

    controller_interface::CallbackReturn on_init()      override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    /// Main loop — reads encoders, estimates velocity, integrates pose, publishes.
    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // ── Parameter helpers ──────────────────────────────────────────────────────

    void declare_parameters();
    bool read_parameters();

    // ── Interface name builders ────────────────────────────────────────────────

    std::vector<std::string> steer_state_interface_names() const;
    std::vector<std::string> drive_state_interface_names() const;

    // ── Interface handle assignment ────────────────────────────────────────────

    bool assign_interfaces();

    // ── Kinematics matrix ──────────────────────────────────────────────────────

    /// Build 8×3 matrix A from wheel positions, compute and cache its pseudoinverse.
    void build_kinematics_matrix();

    // ── Publishing ─────────────────────────────────────────────────────────────

    void publish_odom(double vx, double vy, double wz, const rclcpp::Time & stamp);

    // ── Joint name storage (order: FL, FR, RL, RR) ────────────────────────────

    std::array<std::string, ODOM_NUM_WHEELS> steer_joint_names_;
    std::array<std::string, ODOM_NUM_WHEELS> drive_joint_names_;

    // ── Geometry parameters ────────────────────────────────────────────────────

    double wheelbase_{0.842};
    double track_width_{0.682};
    double wheel_radius_{0.16};

    // ── Kinematics ─────────────────────────────────────────────────────────────

    Eigen::MatrixXd A_pinv_;   // 3×8 pseudoinverse

    // ── Interface handle containers (filled in on_activate) ───────────────────

    struct SteerStateHandles {
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position;
    };
    struct DriveStateHandles {
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position;
    };

    std::optional<SteerStateHandles> steer_handles_;
    std::optional<DriveStateHandles> drive_handles_;

    // ── Encoder state ──────────────────────────────────────────────────────────

    std::array<double, ODOM_NUM_WHEELS> prev_drive_pos_{};  ///< encoder positions at last cycle
    bool first_update_{true};

    // ── Integrated pose ────────────────────────────────────────────────────────

    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};

    // ── ROS interfaces ─────────────────────────────────────────────────────────

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>         tf_broadcaster_;
};

}  // namespace rover_controller