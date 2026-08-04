#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// swerve_controller_test.hpp
//
// Experimental 4-wheel swerve controller. Same interfaces, same kinematics and
// same joint handling as RoverSwerveController — the one thing it does
// differently is *what it smooths*.
//
// RoverSwerveController runs three independent slew limiters over vx, vy and
// wz. Their ratios are what the steering angles are built from, so whenever
// the three ramp at different rates the ratios drift and the wheels twitch —
// most visibly when only the throttle moves and the geometry of the turn was
// supposed to stay put.
//
// This controller instead splits the twist into shape (theta, phi) and
// magnitude (m) — see twist_shape.hpp — and smooths those. Changing speed then
// touches m alone, and the steering angles do not move at all.
//
// Falling out of the same change:
//   · spin-in-place is reachable, as Twist(0, 0, wz) → phi = ±pi/2, with no
//     special case and a continuous sweep into and out of it
//   · a near-zero Twist is a usable "point the wheels here" command, because
//     normalising recovers the shape at any scale (see park_speed)
//   · turn radius is unbounded — nothing clamps wz
//   · no TRANSIT state machine: it exists to catch discontinuous angle
//     targets, and shape smoothing makes the targets continuous by
//     construction. The per-joint step_angle() limit and the cos⁴ alignment
//     scale remain as the anti-scrub safety net.
//
// Claimed interfaces, topic and services are identical to RoverSwerveController
// so the two are drop-in swappable via switch_controller.
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"

// SlewRateLimiter, SteerHandles, DriveHandles, NUM_WHEELS — reused as-is.
#include "rover_controller/swerve_controller.hpp"
#include "rover_controller/swerve_kinematics.hpp"
#include "rover_controller/twist_shape.hpp"

namespace rover_controller {

class RoverSwerveControllerTest : public controller_interface::ControllerInterface
{
public:
    RoverSwerveControllerTest();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    void declare_parameters();
    bool read_parameters();

    std::vector<std::string> steer_command_interface_names() const;
    std::vector<std::string> drive_command_interface_names() const;
    std::vector<std::string> steer_state_interface_names()   const;

    bool assign_interfaces();

    void read_measured_angles();
    void write_steer_commands(const WheelData & angles);
    void write_drive_commands(const WheelData & work_angles, const WheelData & speeds);

    double step_angle(double current, double target, double dt, double rate) const;

    void on_set_compact_mode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response>      response);

    bool cmd_vel_timed_out(const rclcpp::Time & now) const;

    // Joint names — order must match the FL/FR/RL/RR index constants.
    std::array<std::string, NUM_WHEELS> steer_joint_names_;
    std::array<std::string, NUM_WHEELS> drive_joint_names_;

    std::unique_ptr<SwerveKinematics> kinematics_;
    std::unique_ptr<ShapeSmoother>    shape_smoother_;

    /// Accel/decel limiter for the (signed) magnitude. The only limiter here —
    /// vx/vy/wz are never smoothed individually.
    SlewRateLimiter magnitude_limiter_{0.2, 0.5};

    // Runtime state
    WheelData current_angles_{WheelData::filled(0.0)};   ///< integrated steering command
    WheelData measured_angles_{WheelData::filled(0.0)};  ///< hardware feedback

    /// Latest twist off /cmd_vel, stored raw — nothing is clamped on the way
    /// in. Limiting wz here would silently change the turn radius, which is
    /// exactly the geometry this controller exists to keep stable.
    double raw_vx_{0.0};
    double raw_vy_{0.0};
    double raw_wz_{0.0};

    /// Latest commanded shape. Held across an empty Twist so the wheels keep
    /// their angle while the speed ramps out instead of snapping to straight.
    TwistShape target_shape_{};

    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};

    double wheelbase_{1.20};
    double track_width_{0.80};
    double wheel_radius_{0.15};
    double max_steer_{M_PI / 2.0};        // rad
    double max_steer_rate_{M_PI / 4.0};   // rad/s
    double max_linear_{0.5};              // m/s
    double max_accel_{0.2};
    double max_decel_{0.5};
    double cmd_vel_timeout_{2.0};         // s

    /// Lever arm converting wz into a linear speed. 0.0 in yaml means "auto",
    /// resolved to the wheel half-diagonal in on_configure.
    double rotation_scale_{0.0};          // m

    double max_theta_rate_{M_PI / 2.0};   // rad/s — crab direction
    double max_phi_rate_{0.55};           // rad/s — turn tightness

    /// Below this magnitude the wheels still steer but the drives are held at
    /// zero, which is what turns a near-zero Twist into an angle-only command.
    double park_speed_{1e-3};             // m/s

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr         compact_srv_;

    std::optional<SteerHandles> steer_handles_;
    std::optional<DriveHandles> drive_handles_;
};

}  // namespace rover_controller
