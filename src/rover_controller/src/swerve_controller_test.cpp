#include "rover_controller/swerve_controller_test.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
    rover_controller::RoverSwerveControllerTest,
    controller_interface::ControllerInterface)

namespace rover_controller {

namespace {

/// Throttle interval for the "something is wrong every cycle" log lines
constexpr int kFaultLogPeriodMs = 2000;
constexpr double kMaxDt = 0.1;

bool finite_positive(double v) { return std::isfinite(v) && v > 0.0; }
bool finite_nonneg(double v)   { return std::isfinite(v) && v >= 0.0; }

}  // namespace


RoverSwerveControllerTest::RoverSwerveControllerTest()
: controller_interface::ControllerInterface()
{}


controller_interface::CallbackReturn
RoverSwerveControllerTest::on_init()
{
    try {
        declare_parameters();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[SwerveControllerTest] on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveControllerTest::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!read_parameters()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    kinematics_ = std::make_unique<SwerveKinematics>(
        wheelbase_, track_width_, wheel_radius_, max_steer_, max_linear_);

    shape_smoother_ = std::make_unique<ShapeSmoother>(max_theta_rate_, max_phi_rate_);

    magnitude_limiter_ = SlewRateLimiter{max_accel_, max_decel_};

    cmd_vel_buffer_.initRT(CmdVelStamped{
        geometry_msgs::msg::Twist{},
        get_node()->get_clock()->now()
    });

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            if (!std::isfinite(msg->linear.x) ||
                !std::isfinite(msg->linear.y) ||
                !std::isfinite(msg->angular.z))
            {
                RCLCPP_ERROR_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), kFaultLogPeriodMs,
                    "[SwerveControllerTest] Ignoring non-finite /cmd_vel "
                    "(vx=%.4f vy=%.4f wz=%.4f).",
                    msg->linear.x, msg->linear.y, msg->angular.z);
                return;
            }

            CmdVelStamped stamped;
            stamped.twist = *msg;
            stamped.stamp = get_node()->get_clock()->now();
            cmd_vel_buffer_.writeFromNonRT(stamped);
        }
    );

    compact_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/set_compact_mode",
        [this](
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response>      res)
        {
            on_set_compact_mode(req, res);
        }
    );

    RCLCPP_INFO(get_node()->get_logger(),
        "[SwerveControllerTest] Configured. wheelbase=%.3f m  track=%.3f m  "
        "r_wheel=%.3f m  max_steer=%.1f°  max_joint=%.1f°  max_v=%.2f m/s  "
        "rotation_scale=%.3f m  theta_rate=%.2f rad/s  phi_rate=%.2f rad/s  "
        "standstill=%.3f m/s held %.2f s",
        wheelbase_, track_width_, wheel_radius_,
        max_steer_ * 180.0 / M_PI, max_joint_angle_ * 180.0 / M_PI, max_linear_,
        rotation_scale_, max_theta_rate_, max_phi_rate_,
        standstill_speed_, standstill_hold_);

    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveControllerTest::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!bind_interfaces()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    // Settle compact mode first: the offsets it installs define the frame
    // committed_angles_ is seeded in, just below.
    apply_pending_compact_mode();

    // Start the integrated steering command from where the joints actually are,
    // so activation never commands a jump.
    read_feedback();
    if (!steer_feedback_ok_) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[SwerveControllerTest] Steering feedback is not finite at activation; "
            "refusing to activate rather than integrating from a bad start.");
        return controller_interface::CallbackReturn::ERROR;
    }
    current_angles_ = measured_angles_;

    const WheelData offsets = kinematics_->offset_angles();
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        committed_angles_[i] = measured_angles_[i] - offsets[i];
    }

    const rclcpp::Time now = get_node()->get_clock()->now();

    cmd_vel_buffer_.initRT(CmdVelStamped{geometry_msgs::msg::Twist{}, now});

    shape_smoother_->reset();
    magnitude_limiter_.reset(0.0);
    target_shape_ = TwistShape{};

    idle_       = false;
    idle_since_ = now;

    at_rest_       = false;
    at_rest_since_ = now;

    RCLCPP_INFO(get_node()->get_logger(), "[SwerveControllerTest] Activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveControllerTest::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (drive_handles_) {
        // Command zero velocity so motors don't coast.
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            (void)drive_handles_->velocity_cmd[i].get().set_value(0.0);
        }
    }

    steer_handles_.reset();
    drive_handles_.reset();
    drive_state_handles_.reset();

    steer_feedback_ok_ = false;
    drive_feedback_ok_ = false;

    RCLCPP_INFO(get_node()->get_logger(), "[SwerveControllerTest] Deactivated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
RoverSwerveControllerTest::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & name : steer_command_interface_names()) { cfg.names.push_back(name); }
    for (const auto & name : drive_command_interface_names()) { cfg.names.push_back(name); }
    return cfg;
}


controller_interface::InterfaceConfiguration
RoverSwerveControllerTest::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & name : steer_state_interface_names()) { cfg.names.push_back(name); }
    for (const auto & name : drive_state_interface_names()) { cfg.names.push_back(name); }
    return cfg;
}


controller_interface::return_type
RoverSwerveControllerTest::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    const double dt = std::isfinite(period.seconds())
        ? clamp(period.seconds(), 0.0, kMaxDt)
        : 0.0;

    apply_pending_compact_mode();

    read_feedback();

    const CmdVelStamped cmd = *cmd_vel_buffer_.readFromRT();

    double raw_vx = cmd.twist.linear.x;
    double raw_vy = cmd.twist.linear.y;
    double raw_wz = cmd.twist.angular.z;

    if (cmd_vel_timed_out(time, cmd.stamp)) {
        raw_vx = raw_vy = raw_wz = 0.0;
    }

    // Step 1: Split the commanded twist into shape + magnitude.
    const TwistShape incoming = decompose(raw_vx, raw_vy, raw_wz, rotation_scale_);

    const bool commanded_empty = (incoming.m <= 0.0);

    double max_rim_speed = 0.0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        max_rim_speed = std::max(
            max_rim_speed, std::abs(measured_wheel_rates_[i]) * wheel_radius_);
    }

    const bool cmd_at_rest    = std::abs(magnitude_limiter_.current()) < park_speed_;
    const bool wheels_at_rest = drive_feedback_ok_ && (max_rim_speed < standstill_speed_);

    if (cmd_at_rest && wheels_at_rest) {
        if (!at_rest_) {
            at_rest_       = true;
            at_rest_since_ = time;
        }
    } else {
        at_rest_ = false;
    }

    const bool stopped =
        at_rest_ && (time - at_rest_since_).seconds() >= standstill_hold_;

    if (commanded_empty && stopped) {
        if (!idle_) {
            idle_       = true;
            idle_since_ = time;
        }
    } else {
        idle_ = false;
    }

    const bool home_wheels =
        idle_ && idle_home_delay_ >= 0.0 &&
        (time - idle_since_).seconds() >= idle_home_delay_;

    if (incoming.m > 0.0) {
        target_shape_ = incoming;
    } else if (home_wheels) {
        target_shape_ = TwistShape{};
    } else {
        target_shape_.m = 0.0;
    }

    target_shape_.m = clamp(target_shape_.m, -max_linear_, max_linear_);

    // Step 2: Smooth. The shape slews on the sphere; the magnitude runs
    // through its own accel/decel limiter. Because the two are independent, a
    // pure throttle change leaves every steering angle untouched.
    //
    // Standing still is the exception: the shape rate limit exists to stop the
    // wheels fighting the ground, and a stopped rover has no ground fight to
    // pick. Snapping lets it pivot straight to the manoeuvre it is about to
    // start rather than tracking a sweep along a path it is not travelling.
    const TwistShape resolved = stopped
        ? shape_smoother_->snap(target_shape_)
        : shape_smoother_->step(target_shape_, dt);

    const double magnitude = magnitude_limiter_.update(resolved.m, dt);

    // Step 3: Run IK at a fixed nominal magnitude rather than the real one.
    //
    // The IK is homogeneous, so the angles this returns are the angles for the
    // current shape at *any* speed. Evaluating at a nominal magnitude keeps
    // them well-defined all the way down to a standstill — a literal
    // ik_full(1e-6, …) would trip its own near-zero guard and hand back
    // straight-ahead angles instead. Speeds are then scaled back down.
    double ux, uy, uz;
    shape_unit(resolved, ux, uy, uz);

    const double nominal = max_linear_;
    const auto ik = kinematics_->ik_full(
        ux * nominal,
        uy * nominal,
        uz * nominal / rotation_scale_,
        current_angles_);

    // Step 3b: Pick which of the two equivalent representations of each wheel
    // to actually command. (angle a, speed +s) and (angle a±180, speed −s)
    // describe the same physical wheel motion, so the choice is free — but it
    // is not free to keep *changing*.
    const WheelData offsets = kinematics_->offset_angles();

    WheelData work_angles, nominal_speeds;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double a0 = ik.angles[i] - offsets[i];
        const double s0 = ik.speeds[i];
        const double a1 = (a0 < 0.0) ? a0 + M_PI : a0 - M_PI;

        auto reachable = [&](double a) {
            return std::abs(a) <= max_steer_ + 1e-9 &&
                   std::abs(a + offsets[i]) <= max_joint_angle_ + 1e-9;
        };

        const bool a0_ok = reachable(a0);
        const bool a1_ok = reachable(a1);

        // committed_angles_ is kept in the same steering frame, so toggling
        // compact mode does not move the reference and cannot make this step
        // pick the long way round on the cycle the mode changes.
        const double reference = stopped
            ? (current_angles_[i] - offsets[i])
            : committed_angles_[i];

        bool take_alt;
        if (a0_ok && a1_ok) {
            take_alt = std::abs(a1 - reference) < std::abs(a0 - reference);
        } else {
            // Only one representation is reachable, so there is no choice.
            take_alt = a1_ok && !a0_ok;
        }

        const double chosen = take_alt ? a1 : a0;

        const double work = clamp(
            chosen + offsets[i], -max_joint_angle_, max_joint_angle_);

        committed_angles_[i] = work - offsets[i];
        work_angles[i]       = work;
        nominal_speeds[i]    = take_alt ? -s0 : s0;
    }

    // Below park_speed the wheels still track the commanded angle but the
    // drives are held at zero — this is what lets a near-zero Twist act as a
    // pure "point the wheels here" command with the rover standing still.
    const double speed_gain =
        (std::abs(magnitude) < park_speed_) ? 0.0 : (magnitude / nominal);

    // Step 4: Step the steering joints toward target, rate-limited.
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        current_angles_[i] = step_angle(
            current_angles_[i], work_angles[i], dt, max_steer_rate_);
    }

    write_steer_commands(current_angles_);

    // Step 5: Scale and orient the drives from the *measured* steering error —
    // one angle source for both decisions, deliberately.
    //
    // Both quantities are functions of the same cos(err):
    //   · cos⁴ cuts speed while a wheel is still off-target, so it doesn't
    //     scrub. Fourth power because its derivative vanishes at both ends,
    //     making the fade smooth in and out.
    //   · a wheel more than 90° from its target points roughly backwards, so it
    //     has to spin the other way to push the chassis where intended.
    //
    // Taking them from different sources — cos⁴ from feedback, the sign from
    // the integrated command — is what makes lagging feedback dangerous: the
    // sign flips at the command's 90° while cos⁴ is still near 1 at the
    // measurement's, and the drive reverses at close to full speed. Sharing
    // one source ties the flip to the exact point where the scale is zero, so
    // the drive command stays continuous through it however far behind the
    // joints are.
    //
    // Measured, not commanded, because it is the physical wheel that either
    // scrubs or pushes. The cost is that lost or lagging feedback cuts the
    // drives, which is the direction to fail in.
    double    align_scale = 1.0;
    WheelData drive_sign  = WheelData::filled(1.0);
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double cos_err = std::cos(work_angles[i] - measured_angles_[i]);
        align_scale  = std::min(align_scale, std::pow(cos_err, 4));
        drive_sign[i] = (cos_err >= 0.0) ? 1.0 : -1.0;
    }

    if (!steer_feedback_ok_) {
        // No trustworthy idea where the wheels point, so no idea which way to
        // drive them. Steering still tracks — current_angles_ is integrated,
        // not measured — so the joints keep working toward the target.
        align_scale = 0.0;
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), kFaultLogPeriodMs,
            "[SwerveControllerTest] Non-finite steering feedback; drives held at zero.");
    }

    WheelData speeds;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        speeds[i] = nominal_speeds[i] * speed_gain * align_scale * drive_sign[i];
    }

    write_drive_commands(speeds);

    return controller_interface::return_type::OK;
}


void RoverSwerveControllerTest::declare_parameters()
{
    auto node = get_node();

    auto declare_param = [node](const auto & name, const auto & default_val) {
        try {
            node->declare_parameter(name, default_val);
        } catch (const std::exception &) {
            // Parameter already declared, ignore error
        }
    };

    declare_param("wheelbase",             0.797);
    declare_param("track_width",           0.644);
    declare_param("wheel_radius",          0.156);
    declare_param("max_steer_deg",         90.0);
    declare_param("max_steer_rate_deg",    45.0);
    declare_param("max_joint_angle_deg",   270.0);
    declare_param("max_linear_speed",      0.50);
    declare_param("max_accel",             0.20);
    declare_param("max_decel",             0.50);
    declare_param("cmd_vel_timeout_s",     2.0);

    // 0.0 → auto (wheel half-diagonal, computed in read_parameters).
    declare_param("rotation_scale_length", 0.0);
    declare_param("max_theta_rate_rad",    M_PI / 2.0);
    declare_param("max_phi_rate_rad",      0.55);
    declare_param("park_speed",            0.001);
    declare_param("standstill_speed",      0.02);
    declare_param("standstill_hold_s",     0.2);
    declare_param("idle_home_delay",       1.0);

    declare_param("steer_joint_names",
        std::vector<std::string>{"fl_wheel_mount_joint", "fr_wheel_mount_joint",
                                 "bl_wheel_mount_joint", "br_wheel_mount_joint"});
    declare_param("drive_joint_names",
        std::vector<std::string>{"fl_wheel_joint", "fr_wheel_joint",
                                 "bl_wheel_joint", "br_wheel_joint"});
}


bool RoverSwerveControllerTest::read_parameters()
{
    auto node = get_node();

    wheelbase_       = node->get_parameter("wheelbase").as_double();
    track_width_     = node->get_parameter("track_width").as_double();
    wheel_radius_    = node->get_parameter("wheel_radius").as_double();
    max_steer_       = node->get_parameter("max_steer_deg").as_double()      * M_PI / 180.0;
    max_steer_rate_  = node->get_parameter("max_steer_rate_deg").as_double() * M_PI / 180.0;
    max_joint_angle_ = node->get_parameter("max_joint_angle_deg").as_double() * M_PI / 180.0;
    max_linear_      = node->get_parameter("max_linear_speed").as_double();
    max_accel_       = node->get_parameter("max_accel").as_double();
    max_decel_       = node->get_parameter("max_decel").as_double();
    cmd_vel_timeout_ = node->get_parameter("cmd_vel_timeout_s").as_double();

    rotation_scale_  = node->get_parameter("rotation_scale_length").as_double();
    max_theta_rate_  = node->get_parameter("max_theta_rate_rad").as_double();
    max_phi_rate_    = node->get_parameter("max_phi_rate_rad").as_double();
    park_speed_      = node->get_parameter("park_speed").as_double();
    standstill_speed_ = node->get_parameter("standstill_speed").as_double();
    standstill_hold_ = node->get_parameter("standstill_hold_s").as_double();
    idle_home_delay_ = node->get_parameter("idle_home_delay").as_double();

    // Validated before anything derived is computed. A NaN or a negative here
    // does not stay put: it propagates through the IK into the steering angles
    // and out to the joints, where the first sign of trouble is the rover
    // moving. Fail configuration instead — the launch will say so.
    struct Check {
        const char * name;
        double       value;
        bool         ok;
    };

    const Check checks[] = {
        {"wheelbase",             wheelbase_,        finite_positive(wheelbase_)},
        {"track_width",           track_width_,      finite_positive(track_width_)},
        {"wheel_radius",          wheel_radius_,     finite_positive(wheel_radius_)},
        {"max_steer_deg",         max_steer_ * 180.0 / M_PI,      finite_positive(max_steer_)},
        {"max_steer_rate_deg",    max_steer_rate_ * 180.0 / M_PI, finite_positive(max_steer_rate_)},
        {"max_joint_angle_deg",   max_joint_angle_ * 180.0 / M_PI,
                                  finite_positive(max_joint_angle_)},
        {"max_linear_speed",      max_linear_,       finite_positive(max_linear_)},
        {"max_accel",             max_accel_,        finite_positive(max_accel_)},
        {"max_decel",             max_decel_,        finite_positive(max_decel_)},
        {"cmd_vel_timeout_s",     cmd_vel_timeout_,  finite_positive(cmd_vel_timeout_)},
        {"max_theta_rate_rad",    max_theta_rate_,   finite_positive(max_theta_rate_)},
        {"max_phi_rate_rad",      max_phi_rate_,     finite_positive(max_phi_rate_)},
        {"park_speed",            park_speed_,       finite_nonneg(park_speed_)},
        {"standstill_speed",      standstill_speed_, finite_nonneg(standstill_speed_)},
        {"standstill_hold_s",     standstill_hold_,  finite_nonneg(standstill_hold_)},
        // Negative is meaningful here — it disables homing — so only the
        // non-finite case is rejected.
        {"idle_home_delay",       idle_home_delay_,  std::isfinite(idle_home_delay_)},
        // 0.0 is the "auto" sentinel, resolved just below.
        {"rotation_scale_length", rotation_scale_,   finite_nonneg(rotation_scale_)},
    };

    bool valid = true;
    for (const auto & check : checks) {
        if (!check.ok) {
            RCLCPP_ERROR(node->get_logger(),
                "[SwerveControllerTest] Parameter '%s' is invalid (%f).",
                check.name, check.value);
            valid = false;
        }
    }
    if (!valid) { return false; }

    if (max_steer_ + max_joint_angle_ < 2.0 * M_PI) {
        RCLCPP_WARN(node->get_logger(),
            "[SwerveControllerTest] max_steer_deg (%.1f) + max_joint_angle_deg "
            "(%.1f) is under 360; in compact mode some steering angles are "
            "reachable by neither wheel representation and will be clamped at "
            "the joint limit.",
            max_steer_ * 180.0 / M_PI, max_joint_angle_ * 180.0 / M_PI);
    }

    // Auto: the wheel half-diagonal. Picking that length makes the magnitude
    // equal the corner wheels' ground speed during a spin and the chassis speed
    // during translation, so max_accel means one physical thing in both cases.
    if (rotation_scale_ <= 0.0) {
        rotation_scale_ = std::hypot(wheelbase_ / 2.0, track_width_ / 2.0);
    }

    const auto steer_names = node->get_parameter("steer_joint_names").as_string_array();
    const auto drive_names = node->get_parameter("drive_joint_names").as_string_array();

    if (steer_names.size() != NUM_WHEELS || drive_names.size() != NUM_WHEELS) {
        RCLCPP_ERROR(node->get_logger(),
            "[SwerveControllerTest] steer_joint_names and drive_joint_names "
            "must each have exactly %zu entries.", NUM_WHEELS);
        return false;
    }

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        steer_joint_names_[i] = steer_names[i];
        drive_joint_names_[i] = drive_names[i];
    }

    return true;
}


std::vector<std::string>
RoverSwerveControllerTest::steer_command_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : steer_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return names;
}

std::vector<std::string>
RoverSwerveControllerTest::drive_command_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : drive_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return names;
}

std::vector<std::string>
RoverSwerveControllerTest::steer_state_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : steer_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return names;
}

std::vector<std::string>
RoverSwerveControllerTest::drive_state_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : drive_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return names;
}


bool RoverSwerveControllerTest::bind_interfaces()
{
    auto find_cmd = [this](const std::string & full_name)
        -> hardware_interface::LoanedCommandInterface *
    {
        for (auto & iface : command_interfaces_) {
            if (iface.get_name() == full_name) { return &iface; }
        }
        return nullptr;
    };

    auto find_state = [this](const std::string & full_name)
        -> hardware_interface::LoanedStateInterface *
    {
        for (auto & iface : state_interfaces_) {
            if (iface.get_name() == full_name) { return &iface; }
        }
        return nullptr;
    };

    SteerHandles steer;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const std::string pos_name =
            steer_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;

        auto * cmd_iface   = find_cmd(pos_name);
        auto * state_iface = find_state(pos_name);

        if (!cmd_iface || !state_iface) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[SwerveControllerTest] Missing interface: %s", pos_name.c_str());
            return false;
        }

        steer.position_cmd.emplace_back(*cmd_iface);
        steer.position_state.emplace_back(*state_iface);
    }

    DriveHandles      drive;
    DriveStateHandles drive_state;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const std::string vel_name =
            drive_joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;

        auto * cmd_iface   = find_cmd(vel_name);
        auto * state_iface = find_state(vel_name);

        if (!cmd_iface || !state_iface) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[SwerveControllerTest] Missing interface: %s", vel_name.c_str());
            return false;
        }

        drive.velocity_cmd.emplace_back(*cmd_iface);
        drive_state.velocity_state.emplace_back(*state_iface);
    }

    steer_handles_       = std::move(steer);
    drive_handles_       = std::move(drive);
    drive_state_handles_ = std::move(drive_state);
    return true;
}


namespace {

/// One reading off a loaned state interface, distro differences absorbed.
inline double state_value(hardware_interface::LoanedStateInterface & iface)
{
#if defined(JAZZY_OR_LATER)
    // value_or() picks a finite sentinel deliberately: an empty optional means
    // "no reading this cycle", and the caller's finiteness check would wave a
    // silent 0.0 through. NaN makes it visible as exactly what it is.
    return iface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
#else
    return iface.get_value();
#endif
}

}  // namespace


void RoverSwerveControllerTest::read_feedback()
{
    // Both channels are validated wholesale: a partial update would mix this
    // cycle's readings with the last one's, and the wheels are only comparable
    // to each other when they come from the same instant.
    steer_feedback_ok_ = false;
    drive_feedback_ok_ = false;

    if (steer_handles_) {
        WheelData fresh;
        bool ok = true;
        for (std::size_t i = 0; i < NUM_WHEELS && ok; ++i) {
            fresh[i] = state_value(steer_handles_->position_state[i].get());
            ok = std::isfinite(fresh[i]);
        }
        if (ok) { measured_angles_ = fresh; }
        steer_feedback_ok_ = ok;
    }

    if (drive_state_handles_) {
        WheelData fresh;
        bool ok = true;
        for (std::size_t i = 0; i < NUM_WHEELS && ok; ++i) {
            fresh[i] = state_value(drive_state_handles_->velocity_state[i].get());
            ok = std::isfinite(fresh[i]);
        }
        if (ok) { measured_wheel_rates_ = fresh; }
        drive_feedback_ok_ = ok;
    }
}


void RoverSwerveControllerTest::write_steer_commands(const WheelData & angles)
{
    if (!steer_handles_) { return; }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (!std::isfinite(angles[i])) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), kFaultLogPeriodMs,
                "[SwerveControllerTest] Non-finite steering command for wheel %zu; "
                "holding last command.", i);
            continue;   // leave the interface at whatever it last held
        }
        (void)steer_handles_->position_cmd[i].get().set_value(angles[i]);
    }
}


void RoverSwerveControllerTest::write_drive_commands(const WheelData & speeds)
{
    if (!drive_handles_) { return; }

    // Sign and scale were both settled in update(), from one angle source.
    // Last line of defence before the hardware: a non-finite command here is
    // unrecoverable, so substitute zero and say so.
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        double value = speeds[i];
        if (!std::isfinite(value)) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), kFaultLogPeriodMs,
                "[SwerveControllerTest] Non-finite drive command for wheel %zu; "
                "writing zero.", i);
            value = 0.0;
        }
        (void)drive_handles_->velocity_cmd[i].get().set_value(value);
    }
}


double RoverSwerveControllerTest::step_angle(
    double current,
    double target,
    double dt,
    double rate) const
{
    const double diff = target - current;
    const double step = clamp(diff, -rate * dt, rate * dt);
    return current + step;
}


void RoverSwerveControllerTest::apply_pending_compact_mode()
{
    if (!kinematics_) { return; }

    const bool requested = compact_mode_request_.load(std::memory_order_relaxed);
    if (requested == kinematics_->compact_mode()) { return; }

    // No TRANSIT dance needed: the offset shifts the target angles, and
    // the cos⁴ alignment scale holds the drives down until they arrive
    kinematics_->set_compact_mode(requested);

    RCLCPP_INFO(get_node()->get_logger(),
        "[SwerveControllerTest] Compact mode %s.", requested ? "enabled" : "disabled");
}


void RoverSwerveControllerTest::on_set_compact_mode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    if (!kinematics_) {
        response->success = false;
        response->message = "Controller not yet configured";
        return;
    }

    const bool previous = compact_mode_request_.exchange(
        request->data, std::memory_order_relaxed);

    response->success = true;
    response->message = (previous == request->data)
        ? "Already in requested mode"
        : std::string("Compact mode ") + (request->data ? "enabled" : "disabled") +
          " — wheels will move over at max_steer_rate";
}


bool RoverSwerveControllerTest::cmd_vel_timed_out(
    const rclcpp::Time & now,
    const rclcpp::Time & stamp) const
{
    if (now.get_clock_type() != stamp.get_clock_type()) { return true; }
    return (now - stamp).seconds() > cmd_vel_timeout_;
}

}  // namespace rover_controller
