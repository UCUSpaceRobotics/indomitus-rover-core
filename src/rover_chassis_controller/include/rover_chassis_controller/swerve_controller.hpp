#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// rover_swerve_controller.hpp
//
// ros2_control Controller plugin for 4-wheel swerve drive.
//
// Implements:
//   controller_interface::ControllerInterface
//
// Claimed interfaces (must match <ros2_control> block in rover_description):
//   StateInterfaces  — fl/fr/rl/rr  : steer position  [rad]
//   CommandInterfaces — fl/fr/rl/rr : steer position  [rad]   (wheel_mount_joint)
//                       fl/fr/rl/rr : drive velocity  [rad/s] (wheel_joint)
//
// Subscribed topic:
//   ~/cmd_vel  [geometry_msgs/Twist]  — remappable via controller params
//
// Services:
//   ~/set_compact_mode  [std_srvs/SetBool]
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <memory>
#include <string>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_chassis_controller/swerve_kinematics.hpp"

namespace rover_chassis_controller {

constexpr std::size_t FL = 0;
constexpr std::size_t FR = 1;
constexpr std::size_t RL = 2;
constexpr std::size_t RR = 3;
constexpr std::size_t NUM_WHEELS = 4;


enum class RoverState : uint8_t {
    NORMAL,   ///< Combined translation + rotation
    ROTATE,   ///< rotate-in-place (vx, vy ~ 0, wz≠0)
    TRANSIT,  ///< Wheels re-aligning; drive output suppressed
};


class SlewRateLimiter {
public:
    explicit SlewRateLimiter(
        double max_accel,
        double max_decel,
        double initial_value = 0.0)
    : max_accel_(max_accel),
        max_decel_(max_decel),
        current_(initial_value)
    {}

    /**
     * Advance internal state toward `target` by at most rate * dt.
     * Braking (target closer to zero or sign flip) uses max_decel.
     */
    double update(double target, double dt) {
        const double diff      = target - current_;
        const bool   braking   = (std::abs(target) < std::abs(current_))
                                || (current_ * target < 0.0);
        const double rate      = braking ? max_decel_ : max_accel_;
        const double step      = clamp(diff, -rate * dt, rate * dt);
        current_ += step;
        return current_;
    }

    double current() const { return current_; }
    void   reset(double v = 0.0) { current_ = v; }

private:
    double max_accel_;
    double max_decel_;
    double current_;
};

// ─────────────────────────────────────────────────────────────────────────────
// RoverStateMachine
//
// Manages NORMAL / ROTATE / TRANSIT transitions and the speed-scaling envelope
// that prevents wheel scrub during re-alignment.
// ─────────────────────────────────────────────────────────────────────────────

class RoverStateMachine {
public:
    RoverStateMachine(
        double align_threshold,
        double scale_up_rate,
        double scale_down_rate)
    : align_threshold_(align_threshold),
        scale_up_rate_(scale_up_rate),
        scale_down_rate_(scale_down_rate),
        state_(RoverState::NORMAL),
        transit_dest_(RoverState::NORMAL),
        transit_target_(WheelData::filled(0.0)),
        transit_stopping_(false),
        current_scale_(0.0)
    {}

    // ── State access ───────────────────────────────────────────────────────────

    RoverState state()  const { return state_; }
    bool in_transit()   const { return state_ == RoverState::TRANSIT; }

    const WheelData & transit_target() const { return transit_target_; }
    bool              transit_stopping() const { return transit_stopping_; }

    // ── Called once per control cycle ─────────────────────────────────────────

    /**
     * Evaluate whether the current command requires a state change.
     * If so, kicks off a TRANSIT with drive-speed ramp-down before wheel pivot.
     *
     * @param vx/vy/wz        Smoothed chassis velocities
     * @param desired_angles  IK-derived target angles for current command
     * @param current_angles  Actual steering joint positions from hardware
     * @param logger          Optional rclcpp logger for transition events
     */
    void update_transitions(
        double vx, double vy, double wz,
        const WheelData & desired_angles,
        const WheelData & current_angles,
        rclcpp::Logger *  logger = nullptr)
    {
        const RoverState desired_dest = desired_state(vx, vy, wz);

        if (state_ != RoverState::TRANSIT) {
        if (desired_dest != state_) {
            enter_transit(desired_angles, desired_dest, logger);
        }
        } else {
        // Update target continuously while already in transit
        transit_target_ = desired_angles;
        transit_dest_   = desired_dest;

        if (!transit_stopping_ && transit_complete(current_angles)) {
            state_ = transit_dest_;
            if (logger) {
            RCLCPP_INFO(*logger, "[SwerveController] Aligned → %s", state_name(state_));
            }
        }
        }
    }

    /**
     * Compute multiplicative speed scale [0..1] for the current cycle.
     * Ramps down during TRANSIT stopping phase, then ramps up once aligned.
     *
     * @param dt              Time step [s]
     * @param work_angles     Target angles this cycle
     * @param current_angles  Actual joint positions
     */
    double update_scale(
        double dt,
        const WheelData & work_angles,
        const WheelData & current_angles,
        rclcpp::Logger * logger = nullptr)
    {
        (void)logger;

        if (state_ == RoverState::TRANSIT) {
        if (transit_stopping_) {
            current_scale_ = std::max(0.0, current_scale_ - scale_down_rate_ * dt);
            if (current_scale_ < 0.02) {
            current_scale_    = 0.0;
            transit_stopping_ = false;   // allow wheels to pivot now
            }
        } else {
            current_scale_ = 0.0;
        }
        } else {
        // Cosine-based alignment scale: full speed only when wheels are on-target
        double max_err = 0.0;
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            max_err = std::max(max_err, std::abs(work_angles[i] - current_angles[i]));
        }

        double raw_scale = std::max(0.0, std::cos(max_err));
        if (max_err > (20.0 * M_PI / 180.0)) {
            raw_scale = std::min(raw_scale, 0.3);  // hard cap during large pivots
        }

        if (raw_scale < current_scale_) {
            current_scale_ = std::max(raw_scale, current_scale_ - scale_down_rate_ * dt);
        } else {
            current_scale_ = std::min(raw_scale, current_scale_ + scale_up_rate_ * dt);
        }
        }

        return current_scale_;
    }

    void reset()
    {
        state_            = RoverState::NORMAL;
        transit_dest_     = RoverState::NORMAL;
        transit_target_   = WheelData::filled(0.0);
        transit_stopping_ = false;
        current_scale_    = 0.0;
    }

private:
    void enter_transit(
        const WheelData & target,
        RoverState dest,
        rclcpp::Logger * logger)
    {
        state_            = RoverState::TRANSIT;
        transit_dest_     = dest;
        transit_target_   = target;
        transit_stopping_ = true;

        if (logger) {
        RCLCPP_INFO(*logger, "[SwerveController] TRANSIT → %s", state_name(dest));
        }
    }

    bool transit_complete(const WheelData & current_angles) const
    {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (std::abs(transit_target_[i] - current_angles[i]) >= align_threshold_) {
            return false;
        }
        }
        return true;
    }

    static RoverState desired_state(double vx, double vy, double wz)
    {
        const bool has_translation = (std::abs(vx) > VXY_EPS) || (std::abs(vy) > VXY_EPS);
        const bool has_rotation    = std::abs(wz) > WZ_EPS;

        if (!has_translation && has_rotation) { return RoverState::ROTATE; }
        return RoverState::NORMAL;
    }

    static const char * state_name(RoverState s)
    {
        switch (s) {
        case RoverState::NORMAL:  return "NORMAL";
        case RoverState::ROTATE:  return "ROTATE";
        case RoverState::TRANSIT: return "TRANSIT";
        }
        return "UNKNOWN";
    }

    double     align_threshold_;
    double     scale_up_rate_;
    double     scale_down_rate_;
    RoverState state_;
    RoverState transit_dest_;
    WheelData  transit_target_;
    bool       transit_stopping_;
    double     current_scale_;
};

// ─────────────────────────────────────────────────────────────────────────────
// JointHandles — typed wrappers around raw interface references
//
// ros2_control passes LoanedCommandInterface / LoanedStateInterface by index.
// We name them clearly so the rest of the code never uses magic indices.
// ─────────────────────────────────────────────────────────────────────────────

struct SteerHandles
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>   position_state;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_cmd;
};

struct DriveHandles
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity_cmd;
};

// ─────────────────────────────────────────────────────────────────────────────
// RoverSwerveController — the plugin class
// ─────────────────────────────────────────────────────────────────────────────

class RoverSwerveController : public controller_interface::ControllerInterface
{
public:
    RoverSwerveController();

    // ── controller_interface::ControllerInterface overrides ───────────────────

    /**
     * Declare ROS parameters (called by controller_manager before configure).
     * All parameters have defaults — yaml overlay is optional.
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

    controller_interface::CallbackReturn on_init()      override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state)   override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state)   override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state)   override;

    /**
     * Main control loop — called by controller_manager at `control_frequency` Hz.
     * Reads state interfaces, runs kinematics + state machine, writes commands.
     */
    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // ── Parameter helpers ─────────────────────────────────────────────────────

    /// Declare all parameters with defaults; called from on_init.
    void declare_parameters();

    /// Read parameter values into member variables; called from on_configure.
    bool read_parameters();

    // ── Interface name builders ───────────────────────────────────────────────

    /// Returns "fl_wheel_mount_joint/position", "fr_wheel_mount_joint/position", …
    std::vector<std::string> steer_command_interface_names() const;

    /// Returns "fl_wheel_joint/velocity", "fr_wheel_joint/velocity", …
    std::vector<std::string> drive_command_interface_names() const;

    /// Returns "fl_wheel_mount_joint/position", … (state feedback)
    std::vector<std::string> steer_state_interface_names() const;

    // ── Interface handle assignment ───────────────────────────────────────────

    /// Fills steer_handles_ and drive_handles_ from the loaned interface vectors.
    /// Returns false if any expected interface is missing.
    bool assign_interfaces();

    // ── Control helpers ───────────────────────────────────────────────────────

    /// Read current steering positions from state interfaces → current_angles_.
    void read_current_angles();

    /// Write position commands to steer command interfaces.
    void write_steer_commands(const WheelData & angles);

    /// Write velocity commands to drive command interfaces.
    void write_drive_commands(const WheelData & work_angles, const WheelData & speeds, double scale);

    /// Advance a single steering joint toward target, respecting max_steer_rate_.
    double step_angle(double current, double target, double dt, double rate) const;

    // ── Compact-mode service callback ─────────────────────────────────────────

    void on_set_compact_mode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
        std::shared_ptr<std_srvs::srv::SetBool::Response>       response);

    // ── cmd_vel timeout helper ────────────────────────────────────────────────

    bool cmd_vel_timed_out(const rclcpp::Time & now) const;

    // Joint name arrays — order must match FL/FR/RL/RR index constants

    // Default joint names derived from rover_description URDF:
    //   fl_wheel_mount_joint, fr_wheel_mount_joint, rl_wheel_mount_joint, rr_wheel_mount_joint
    //   fl_wheel_joint,       fr_wheel_joint,       rl_wheel_joint,       rr_wheel_joint
    std::array<std::string, NUM_WHEELS> steer_joint_names_;
    std::array<std::string, NUM_WHEELS> drive_joint_names_;

    // Subsystems

    std::unique_ptr<SwerveKinematics> kinematics_;
    std::unique_ptr<RoverStateMachine> state_machine_;

    std::array<SlewRateLimiter, 3> limiters_ = {
        SlewRateLimiter{0.2, 0.5},   // vx
        SlewRateLimiter{0.2, 0.5},   // vy
        SlewRateLimiter{0.2, 0.5},   // wz
    };

    // Runtime state

    WheelData current_angles_{WheelData::filled(0.0)};
    WheelData last_work_speeds_{WheelData::filled(0.0)};

    double vx_smoothed_{0.0};
    double vy_smoothed_{0.0};
    double wz_smoothed_{0.0};

    double target_vx_{0.0};
    double target_vy_{0.0};
    double target_wz_{0.0};

    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};


    double wheelbase_{1.20};
    double track_width_{0.80};
    double wheel_radius_{0.15};
    double max_steer_{M_PI / 2.0};        // rad
    double max_steer_rate_{M_PI / 4.0};   // rad/s
    double max_linear_{0.5};              // m/s
    double max_angular_{0.5};             // rad/s
    double max_accel_{0.2};
    double max_decel_{0.5};
    double control_frequency_{20.0};      // Hz
    double cmd_vel_timeout_{2.0};         // s
    double align_threshold_{0.0873};      // rad ≈ 5°
    double scale_up_rate_{1.0};
    double scale_down_rate_{2.0};


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr         compact_srv_;

    // Loaned interface handle containers — filled in on_activate
    std::optional<SteerHandles> steer_handles_;
    std::optional<DriveHandles> drive_handles_;
};

}  // namespace rover_chassis_controller