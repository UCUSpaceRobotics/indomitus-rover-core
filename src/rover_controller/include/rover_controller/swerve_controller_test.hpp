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
// Commands, topic and services are identical to RoverSwerveController so the
// two are drop-in swappable via switch_controller. The one difference in what
// is claimed: this controller also reads the drive joints' *velocity state*,
// because standstill has to be decided on measured wheel motion — the magnitude
// limiter only reports what was asked for, and "asked for zero" is the start of
// stopping, not the end of it. Both the real hardware interface and the Gazebo
// system export those, so nothing else has to change to switch controllers.
//
// This controller is experimental and is not the default anywhere. Load it
// alongside swerve_controller and switch to it deliberately.
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"

// realtime_tools renamed its headers .h -> .hpp mid-Humble; the .h shim emits a
// deprecation #pragma. Take whichever this install actually ships.
#if defined(__has_include)
#  if __has_include("realtime_tools/realtime_buffer.hpp")
#    include "realtime_tools/realtime_buffer.hpp"
#  else
#    include "realtime_tools/realtime_buffer.h"
#  endif
#else
#  include "realtime_tools/realtime_buffer.h"
#endif

// SlewRateLimiter, SteerHandles, DriveHandles, NUM_WHEELS — reused as-is.
#include "rover_controller/swerve_controller.hpp"
#include "rover_controller/swerve_kinematics.hpp"
#include "rover_controller/twist_shape.hpp"

namespace rover_controller {

/// Drive-joint *state* handles. The steering handles carry their own state
/// vector (SteerHandles); the drive side needs one too, because standstill is
/// decided on measured wheel speed rather than on the command we just issued.
struct DriveStateHandles
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_state;
};

/// One /cmd_vel message plus the time it arrived, handed to update() as a
/// single unit. Both halves have to cross the thread boundary together: a
/// snapshot pairing a fresh Twist with a stale stamp (or the reverse) is how a
/// timeout either fires late or clears early.
struct CmdVelStamped
{
    geometry_msgs::msg::Twist twist{};
    /// RCL_ROS_TIME to match the controller node's clock, which is what
    /// update() is handed. A default-constructed rclcpp::Time is RCL_SYSTEM_TIME
    /// and subtracting the two throws.
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

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
    std::vector<std::string> drive_state_interface_names()   const;

    /// Bind the loaned interfaces into the typed handle structs. Deliberately not
    /// named assign_interfaces() — that is a public base-class method, and a
    /// private overload of it here hides the one controller_manager calls.
    bool bind_interfaces();

    /// Read both feedback channels and validate them. Non-finite readings leave
    /// the previous values in place and clear the matching *_feedback_ok_ flag,
    /// so a dead encoder degrades to "cut the drives" rather than to NaN on a
    /// hardware command interface.
    void read_feedback();

    void write_steer_commands(const WheelData & angles);
    void write_drive_commands(const WheelData & speeds);

    double step_angle(double current, double target, double dt, double rate) const;

    /// Apply a compact-mode request left by the service callback. Called from
    /// update() so the kinematics object is only ever mutated by the RT loop.
    void apply_pending_compact_mode();

    void on_set_compact_mode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response>      response);

    bool cmd_vel_timed_out(const rclcpp::Time & now, const rclcpp::Time & stamp) const;

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
    WheelData measured_angles_{WheelData::filled(0.0)};  ///< steering feedback [rad]
    WheelData measured_wheel_rates_{WheelData::filled(0.0)};  ///< drive feedback [rad/s]

    /// Cleared for the cycle when the matching feedback channel reports a
    /// non-finite value. Steering feedback drives the drive-direction decision,
    /// so losing it cuts the drives; drive feedback is what standstill is
    /// decided on, so losing it means "assume still moving".
    bool steer_feedback_ok_{false};
    bool drive_feedback_ok_{false};

    /// Steering target chosen last cycle, in the *steering frame* — that is,
    /// with the compact-mode offset taken back off. A wheel pointing at angle a
    /// with speed +s is physically identical to one at a±180 with speed −s, and
    /// the IK picks between those two afresh every cycle against the *moving*
    /// joint position — which lets the choice flip mid-manoeuvre and send a
    /// wheel the long way round with its drive reversing. Keeping the previous
    /// target as the reference instead makes the choice continuous, and keeping
    /// it offset-free means toggling compact mode does not disturb it.
    WheelData committed_angles_{WheelData::filled(0.0)};

    /// Latest twist off /cmd_vel, stored raw — nothing is clamped on the way
    /// in. Limiting wz here would silently change the turn radius, which is
    /// exactly the geometry this controller exists to keep stable.
    ///
    /// The buffer is the whole point: the subscription runs on the executor
    /// thread and update() on the control thread, so update() takes one atomic
    /// swap and works from a Twist that was published as a unit. Reading three
    /// plain doubles instead lets a cycle pair a new vx with an old wz, which
    /// is a shape the driver never asked for — and shape is what the steering
    /// angles are built from.
    realtime_tools::RealtimeBuffer<CmdVelStamped> cmd_vel_buffer_;

    /// Latest commanded shape. Held across an empty Twist until the idle timer
    /// expires, so the wheels keep their angle through the deceleration rather
    /// than pivoting while still rolling.
    TwistShape target_shape_{};

    /// Compact-mode request, written by the service callback and consumed by
    /// update(). The service thread never touches kinematics_ itself: it is
    /// read every cycle by the IK, and flipping the offsets underneath a
    /// half-finished cycle would mix pre- and post-change angles.
    std::atomic<bool> compact_mode_request_{false};

    /// Idle tracking for the return-to-zero behaviour.
    bool         idle_{false};
    rclcpp::Time idle_since_{0, 0, RCL_ROS_TIME};

    /// Standstill tracking. `at_rest_` means the rest condition held on the
    /// previous cycle; `stopped` additionally requires it to have held for
    /// standstill_hold_.
    bool         at_rest_{false};
    rclcpp::Time at_rest_since_{0, 0, RCL_ROS_TIME};

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

    /// Ground speed below which a wheel counts as not turning, measured at the
    /// rim. This is what "stopped" is decided on — the magnitude limiter only
    /// says what we asked for, and asking for zero does not make the rover
    /// stop, it starts it stopping.
    double standstill_speed_{0.02};       // m/s

    /// How long every wheel must stay under standstill_speed_ before the rover
    /// is treated as stopped. Rides out encoder noise and the moment of
    /// zero-crossing in a direction change, both of which would otherwise
    /// license a snap mid-roll.
    double standstill_hold_{0.2};         // s

    /// Seconds of continuous idle before the wheels return to zero. Negative
    /// disables homing, leaving them wherever they were last commanded.
    double idle_home_delay_{1.0};         // s

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr         compact_srv_;

    std::optional<SteerHandles>      steer_handles_;
    std::optional<DriveHandles>      drive_handles_;
    std::optional<DriveStateHandles> drive_state_handles_;
};

}  // namespace rover_controller
