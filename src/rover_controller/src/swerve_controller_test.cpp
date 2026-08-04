#include "rover_controller/swerve_controller_test.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
    rover_controller::RoverSwerveControllerTest,
    controller_interface::ControllerInterface)

namespace rover_controller {


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

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            last_cmd_vel_time_ = get_node()->get_clock()->now();
            raw_vx_ = msg->linear.x;
            raw_vy_ = msg->linear.y;
            raw_wz_ = msg->angular.z;
        });

    compact_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/set_compact_mode",
        [this](
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response>      res)
        {
            on_set_compact_mode(req, res);
        });

    RCLCPP_INFO(get_node()->get_logger(),
        "[SwerveControllerTest] Configured. wheelbase=%.3f m  track=%.3f m  "
        "r_wheel=%.3f m  max_steer=%.1f°  max_v=%.2f m/s  "
        "rotation_scale=%.3f m  theta_rate=%.2f rad/s  phi_rate=%.2f rad/s",
        wheelbase_, track_width_, wheel_radius_,
        max_steer_ * 180.0 / M_PI, max_linear_,
        rotation_scale_, max_theta_rate_, max_phi_rate_);

    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveControllerTest::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!assign_interfaces()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    // Start the integrated steering command from where the joints actually are,
    // so activation never commands a jump.
    read_measured_angles();
    current_angles_   = measured_angles_;
    committed_angles_ = measured_angles_;

    last_cmd_vel_time_ = get_node()->get_clock()->now();

    shape_smoother_->reset();
    magnitude_limiter_.reset(0.0);
    target_shape_ = TwistShape{};
    raw_vx_ = raw_vy_ = raw_wz_ = 0.0;

    // Activation counts as the start of an idle period, so the wheels home
    // once after idle_home_delay rather than immediately on activation.
    idle_       = false;
    idle_since_ = last_cmd_vel_time_;

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
    return cfg;
}


controller_interface::return_type
RoverSwerveControllerTest::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    const double dt = period.seconds();

    read_measured_angles();

    if (cmd_vel_timed_out(time)) {
        raw_vx_ = raw_vy_ = raw_wz_ = 0.0;
    }

    // Step 1: Split the commanded twist into shape + magnitude.
    const TwistShape incoming = decompose(raw_vx_, raw_vy_, raw_wz_, rotation_scale_);

    // "Doing nothing" means a genuinely empty twist *and* a rover that has
    // finished stopping. Both halves matter:
    //   · a near-zero twist is not empty — it is an angle-only command, and
    //     homing on it would fight the very thing it asks for
    //   · an empty twist while still rolling is a deceleration, and pivoting
    //     the wheels mid-roll would scrub them
    const bool commanded_empty = (incoming.m <= 0.0);
    const bool stopped = std::abs(magnitude_limiter_.current()) < park_speed_;

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
        // Straight-ahead shape at zero speed. The IK turns that into zero
        // steering angles (or the compact-mode offsets), and the smoother
        // walks there at the usual rates — no separate homing path needed.
        target_shape_ = TwistShape{};
    } else {
        // Stopping, but not idle long enough to home yet. Keep pointing where
        // we were and just ask for zero speed, so a brief pause between
        // commands doesn't snap the wheels straight and back.
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
    //
    // While driving, stay continuous with the target chosen last cycle. The IK
    // picks afresh each cycle against current_angles_, which is itself moving,
    // so its choice can flip part-way through a manoeuvre and send one wheel
    // 213° around with its drive reversing while the others move 39°.
    //
    // While stopped, re-pick freely against the actual joint position instead.
    // A 180° jump costs nothing when no torque is being delivered, and it buys
    // the short way round: entering a spin from a standstill becomes a 51°
    // pivot rather than a 129° sweep that ends up near the steering limit.
    WheelData work_angles, nominal_speeds;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double a0 = ik.angles[i];
        const double s0 = ik.speeds[i];
        const double a1 = (a0 < 0.0) ? a0 + M_PI : a0 - M_PI;

        const bool a0_ok = std::abs(a0) <= max_steer_ + 1e-9;
        const bool a1_ok = std::abs(a1) <= max_steer_ + 1e-9;

        const double reference = stopped ? current_angles_[i] : committed_angles_[i];

        bool take_alt;
        if (a0_ok && a1_ok) {
            take_alt = std::abs(a1 - reference) < std::abs(a0 - reference);
        } else {
            // Only one representation is reachable, so there is no choice.
            take_alt = a1_ok && !a0_ok;
        }

        work_angles[i]    = take_alt ? a1 : a0;
        nominal_speeds[i] = take_alt ? -s0 : s0;
        committed_angles_[i] = work_angles[i];
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

    // Step 5: Cut drive speed while any wheel is still off-target, so a wheel
    // that has not caught up yet doesn't scrub. cos⁴ because its derivative is
    // zero at both ends, making the fade smooth in and out.
    double align_scale = 1.0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double c4 = std::pow(std::cos(work_angles[i] - measured_angles_[i]), 4);
        align_scale = std::min(align_scale, c4);
    }

    WheelData speeds;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        speeds[i] = nominal_speeds[i] * speed_gain * align_scale;
    }

    write_drive_commands(work_angles, speeds);

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

    declare_param("wheelbase",             1.20);
    declare_param("track_width",           0.80);
    declare_param("wheel_radius",          0.15);
    declare_param("max_steer_deg",         90.0);
    declare_param("max_steer_rate_deg",    45.0);
    declare_param("max_linear_speed",      0.50);
    declare_param("max_accel",             0.20);
    declare_param("max_decel",             0.50);
    declare_param("cmd_vel_timeout_s",     2.0);

    // 0.0 → auto (wheel half-diagonal, computed in read_parameters).
    declare_param("rotation_scale_length", 0.0);
    declare_param("max_theta_rate_rad",    M_PI / 2.0);
    declare_param("max_phi_rate_rad",      0.55);
    declare_param("park_speed",            0.001);
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
    max_linear_      = node->get_parameter("max_linear_speed").as_double();
    max_accel_       = node->get_parameter("max_accel").as_double();
    max_decel_       = node->get_parameter("max_decel").as_double();
    cmd_vel_timeout_ = node->get_parameter("cmd_vel_timeout_s").as_double();

    rotation_scale_  = node->get_parameter("rotation_scale_length").as_double();
    max_theta_rate_  = node->get_parameter("max_theta_rate_rad").as_double();
    max_phi_rate_    = node->get_parameter("max_phi_rate_rad").as_double();
    park_speed_      = node->get_parameter("park_speed").as_double();
    idle_home_delay_ = node->get_parameter("idle_home_delay").as_double();

    // Auto: the wheel half-diagonal. Picking that length makes the magnitude
    // equal the corner wheels' ground speed during a spin and the chassis speed
    // during translation, so max_accel means one physical thing in both cases.
    if (rotation_scale_ <= 0.0) {
        rotation_scale_ = std::hypot(wheelbase_ / 2.0, track_width_ / 2.0);
    }

    if (rotation_scale_ <= 0.0 || max_linear_ <= 0.0 || wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(node->get_logger(),
            "[SwerveControllerTest] rotation_scale_length, max_linear_speed and "
            "wheel_radius must all be positive.");
        return false;
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


bool RoverSwerveControllerTest::assign_interfaces()
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

    DriveHandles drive;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const std::string vel_name =
            drive_joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;

        auto * cmd_iface = find_cmd(vel_name);
        if (!cmd_iface) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[SwerveControllerTest] Missing interface: %s", vel_name.c_str());
            return false;
        }

        drive.velocity_cmd.emplace_back(*cmd_iface);
    }

    steer_handles_ = std::move(steer);
    drive_handles_ = std::move(drive);
    return true;
}


void RoverSwerveControllerTest::read_measured_angles()
{
    if (!steer_handles_) { return; }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
#if defined(JAZZY_OR_LATER)
        measured_angles_[i] = steer_handles_->position_state[i].get().get_optional().value_or(0.0);
#else
        measured_angles_[i] = steer_handles_->position_state[i].get().get_value();
#endif
    }
}


void RoverSwerveControllerTest::write_steer_commands(const WheelData & angles)
{
    if (!steer_handles_) { return; }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        (void)steer_handles_->position_cmd[i].get().set_value(angles[i]);
    }
}


void RoverSwerveControllerTest::write_drive_commands(
    const WheelData & work_angles,
    const WheelData & speeds)
{
    if (!drive_handles_) { return; }

    // A wheel more than 90° from its target is pointing roughly backwards, so
    // it has to spin the other way to push the chassis the intended direction.
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double cos_err = std::cos(work_angles[i] - current_angles_[i]);
        const double sign    = (cos_err >= 0.0) ? 1.0 : -1.0;
        (void)drive_handles_->velocity_cmd[i].get().set_value(speeds[i] * sign);
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


void RoverSwerveControllerTest::on_set_compact_mode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    if (!kinematics_) {
        response->success = false;
        response->message = "Controller not yet configured";
        return;
    }

    if (request->data == kinematics_->compact_mode()) {
        response->success = true;
        response->message = "Already in requested mode";
        return;
    }

    // No TRANSIT dance needed: the offset shifts the target angles, and
    // step_angle() walks the joints over at max_steer_rate_ while the cos⁴
    // alignment scale holds the drives down until they arrive.
    kinematics_->set_compact_mode(request->data);

    response->success = true;
    response->message = std::string("Compact mode ") +
                        (kinematics_->compact_mode() ? "enabled" : "disabled");
}


bool RoverSwerveControllerTest::cmd_vel_timed_out(const rclcpp::Time & now) const
{
    return (now - last_cmd_vel_time_).seconds() > cmd_vel_timeout_;
}

}  // namespace rover_controller
