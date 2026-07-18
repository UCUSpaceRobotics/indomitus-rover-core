#include "rover_controller/swerve_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
    rover_controller::RoverSwerveController,
    controller_interface::ControllerInterface)

namespace rover_controller {


static constexpr std::array<const char *, NUM_WHEELS> kWheelPrefixes = {
    "fl", "fr", "bl", "br"
};


RoverSwerveController::RoverSwerveController()
: controller_interface::ControllerInterface()
{}


controller_interface::CallbackReturn
RoverSwerveController::on_init()
{
    try {
        declare_parameters();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[SwerveController] on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!read_parameters()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    kinematics_ = std::make_unique<SwerveKinematics>(
        wheelbase_,
        track_width_,
        wheel_radius_,
        max_steer_,
        max_linear_
    );

    state_machine_ = std::make_unique<RoverStateMachine>(
        align_threshold_,
        scale_up_rate_,
        scale_down_rate_
    );

    // for (auto & lim : limiters_) { lim.reset(0.0); }
    v_limiter_.reset(0.0);
    heading_limiter_.reset(0.0);
    curvature_limiter_.reset(0.0);
    wz_pure_limiter_.reset(0.0);

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
            last_cmd_vel_time_ = get_node()->get_clock()->now();
            target_vx_ = clamp(msg->linear.x,  -max_linear_,  max_linear_);
            target_vy_ = clamp(msg->linear.y,  -max_linear_,  max_linear_);
            target_wz_ = clamp(msg->angular.z, -max_angular_, max_angular_);
        });

    compact_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "~/set_compact_mode",
        [this](
            const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
            std::shared_ptr<std_srvs::srv::SetBool::Response>       res)
        {
            on_set_compact_mode(req, res);
        });

    RCLCPP_INFO(get_node()->get_logger(),
        "[SwerveController] Configured. wheelbase=%.3f m  track=%.3f m  "
        "r_wheel=%.3f m  max_steer=%.1f°  max_v=%.2f m/s",
        wheelbase_, track_width_, wheel_radius_,
        max_steer_ * 180.0 / M_PI, max_linear_);

    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!assign_interfaces()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    read_current_angles();

    last_cmd_vel_time_ = get_node()->get_clock()->now();

    state_machine_->reset();
    vx_smoothed_ = vy_smoothed_ = wz_smoothed_ = 0.0;
    target_vx_   = target_vy_   = target_wz_   = 0.0;
    last_work_speeds_ = WheelData::filled(0.0);

    RCLCPP_INFO(get_node()->get_logger(), "[SwerveController] Activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
RoverSwerveController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (drive_handles_) {
        // Command zero velocity so motors don't coast.
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            (void)drive_handles_->velocity_cmd[i].get().set_value(0.0);
        }
    }

    // Release optional handles
    steer_handles_.reset();
    drive_handles_.reset();

    RCLCPP_INFO(get_node()->get_logger(), "[SwerveController] Deactivated.");
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
RoverSwerveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : steer_command_interface_names()) {
        cfg.names.push_back(name);
    }
    for (const auto& name : drive_command_interface_names()) {
        cfg.names.push_back(name);
    }
    return cfg;
}


controller_interface::InterfaceConfiguration
RoverSwerveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : steer_state_interface_names()) {
        cfg.names.push_back(name);
    }
    return cfg;
}

controller_interface::return_type
RoverSwerveController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    const double dt = period.seconds();

    if (cmd_vel_timed_out(time)) {
        target_vx_ = target_vy_ = target_wz_ = 0.0;
    }

    // Step 1: Smooth chassis velocities via independent slew-rate limiters
    // vx_smoothed_ = limiters_[0].update(target_vx_, dt);
    // vy_smoothed_ = limiters_[1].update(target_vy_, dt);
    // wz_smoothed_ = limiters_[2].update(target_wz_, dt);

    static auto wrap_to_pi = [](double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    const double raw_v = std::hypot(target_vx_, target_vy_);

    if (raw_v > VXY_EPS) {
        const double heading_a = std::atan2(target_vy_, target_vx_);
        const double heading_b = wrap_to_pi(heading_a + M_PI);

        const double d_a = std::abs(wrap_to_pi(heading_a - heading_smoothed_));
        const double d_b = std::abs(wrap_to_pi(heading_b - heading_smoothed_));

        constexpr double kHysteresis = 5.0 * M_PI / 180.0;
        const bool prefer_b = (d_b + kHysteresis < d_a);

        double target_heading, target_curvature;
        if (!prefer_b) {
            target_heading   = heading_a;
            target_curvature = target_wz_ / raw_v;
            target_v_signed_ = raw_v;
        } else {
            target_heading   = heading_b;
            target_curvature = -target_wz_ / raw_v;
            target_v_signed_ = -raw_v;
        }

        heading_smoothed_   = heading_limiter_.update(target_heading, dt);
        curvature_smoothed_ = curvature_limiter_.update(target_curvature, dt);

        // Not rotating in place right now — let the pure-rotation limiter
        // decay to 0 so it's ready for a clean start next time we are.
        wz_pure_smoothed_ = wz_pure_limiter_.update(0.0, dt);
    } else {
        // Direction is undefined at ~zero commanded speed — hold heading/curvature
        // so the next nonzero command can disambiguate forward/backward correctly.
        // But the speed TARGET itself is genuinely zero here, always.
        target_v_signed_ = 0.0;
        curvature_smoothed_ = curvature_limiter_.update(0.0, dt);

        // Pure rotation-in-place: v == 0 but wz requested. curvature = wz/v is
        // undefined here, so wz is tracked directly through its own limiter
        // instead of being derived from v * curvature.
        wz_pure_smoothed_ = wz_pure_limiter_.update(target_wz_, dt);
    }

    v_smoothed_ = v_limiter_.update(target_v_signed_, dt);

    if (std::abs(v_smoothed_) < VXY_EPS && std::abs(wz_pure_smoothed_) < WZ_EPS) {
        vx_smoothed_ = vy_smoothed_ = wz_smoothed_ = 0.0;
    } else if (std::abs(v_smoothed_) < VXY_EPS) {
        // Pure rotation: no translation, wz comes from its own limiter.
        vx_smoothed_ = 0.0;
        vy_smoothed_ = 0.0;
        wz_smoothed_ = wz_pure_smoothed_;
    } else {
        vx_smoothed_ = v_smoothed_ * std::cos(heading_smoothed_);
        vy_smoothed_ = v_smoothed_ * std::sin(heading_smoothed_);
        wz_smoothed_ = v_smoothed_ * curvature_smoothed_;
    }

    const double vx = vx_smoothed_;
    const double vy = vy_smoothed_;
    const double wz = wz_smoothed_;

    // // Step 2: Read measured hardware steering positions
    read_measured_angles();

    // Step 3: Compute desired target angles for transition detection
    double dir_vx, dir_vy, dir_wz;
    if (std::abs(v_smoothed_) < VXY_EPS && std::abs(wz_pure_smoothed_) >= WZ_EPS) {
        dir_vx = 0.0;
        dir_vy = 0.0;
        dir_wz = wz_pure_smoothed_;
    } else {
        dir_vx = std::cos(heading_smoothed_);
        dir_vy = std::sin(heading_smoothed_);
        dir_wz = curvature_smoothed_;
    }

    const auto angle_result = kinematics_->ik_full(dir_vx, dir_vy, dir_wz, current_angles_);
    const WheelData & angle_target = angle_result.angles;

    const WheelData & desired_angles = angle_target;

    auto logger = get_node()->get_logger();
    state_machine_->update_transitions(
        vx, vy, wz,
        desired_angles, current_angles_,
        &logger);

    // Step 4: Compute target angles and speeds for this cycle, based on current state
    // work_angles - targeted angles for this cycle
    // work_speeds - targeted speeds for this cycle
    WheelData work_angles, work_speeds;

    switch (state_machine_->state()) {

        case RoverState::NORMAL: {
            auto [_, s] = kinematics_->ik_full(vx, vy, wz, current_angles_);
            work_angles = angle_target;
            work_speeds = s;
            break;
        }

        // case RoverState::ROTATE: {
        //     auto [a, s] = kinematics_->ik_rotate(wz, current_angles_);
        //     work_angles = a;
        //     work_speeds = s;
        //     break;
        // }

        case RoverState::TRANSIT: {
            if (state_machine_->transit_stopping()) {
                // Keep wheels where they are; fade out last commanded speed.
                work_angles = current_angles_;
                work_speeds = last_work_speeds_;
            } else {
                // Wheels have stopped — now pivot to target.
                work_angles = state_machine_->transit_target();
                work_speeds = WheelData::filled(0.0);
            }
            break;
        }
    }

    // Cache for use during the TRANSIT stopping phase on the next cycle.
    if (state_machine_->state() != RoverState::TRANSIT) {
        last_work_speeds_ = work_speeds;
    }

    // Step 5: Step steering joints toward target (rate-limited)
    const double speed_magnitude = std::hypot(vx, vy);
    const double dynamic_steer_rate = max_steer_rate_ * (1.0 - 0.5 * speed_magnitude / max_linear_);
    // const double dynamic_steer_rate = max_steer_rate_;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        current_angles_[i] = step_angle(current_angles_[i], work_angles[i], dt, dynamic_steer_rate);
    }

    // Step 6: Write steering position commands
    write_steer_commands(current_angles_);

    // Step 7: Global speed scale — cosine alignment penalty.

    double global_align_scale = 1.0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        // const double c = std::cos(work_angles[i] - current_angles_[i]);
        const double c4 = std::pow(std::cos(work_angles[i] - measured_angles_[i]), 4);
        global_align_scale = std::min(global_align_scale, c4);
        // cos^2 because it derivative in 0 and pi/2 is 0,
        // so it's smooth acceleation and deceleration
    }

    // constexpr double kHardStopAngle = M_PI * 3.0 / 8.0;  // 45°

    // double global_align_scale = 1.0;
    // bool any_wheel_misaligned = false;

    // for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    //     const double err = std::abs(work_angles[i] - current_angles_[i]);

    //     double wheel_scale;
    //     if (err >= kHardStopAngle) {
    //         wheel_scale = 0.0;
    //         any_wheel_misaligned = true;
    //     } else {
    //         // cos² normalized over [0, kHardStopAngle]:
    //         // err=0       → scale=1
    //         // err=π/4     → scale=0  (continuous, no jerk)
    //         const double t = err / kHardStopAngle;       // [0, 1]
    //         const double c4 = std::pow(std::cos(t * M_PI / 2.0), 4);  // cos(0)=1, cos(π/2)=0
    //         wheel_scale = c4;
    //     }

    //     global_align_scale = std::min(global_align_scale, wheel_scale);
    // }


    constexpr double kTransitTriggerScale = 0.005;  // tune this

    if (global_align_scale < kTransitTriggerScale && 
        state_machine_->state() == RoverState::NORMAL) {
        state_machine_->force_transit(angle_target, &logger);
    }

    const double state_scale = state_machine_->update_scale(
        dt, work_angles, current_angles_, &logger);

    const double total_scale = global_align_scale * state_scale;

    // Step 8: Write drive velocity commands
    write_drive_commands(work_angles, work_speeds, total_scale);

    const double log_target_curvature = (raw_v > VXY_EPS)
        ? (target_v_signed_ >= 0 ? target_wz_ / raw_v : -target_wz_ / raw_v)
        : 0.0;
        
    
    RCLCPP_INFO(get_node()->get_logger(),
        "[full] t_vx=%.4f t_vy=%.4f t_wz=%.4f raw_v=%.4f t_v_sgn=%.4f | "
        "v_sm=%.3e vx=%.3e vy=%.3e wz=%.3e | "
        "hd_sm=%.6f cv_sm=%.6f cv_target=%.6f cv_gap=%.6f | "
        "spd=%.4f dyn_rate=%.4f | "
        "state=%d scale_align=%.4f scale_state=%.4f scale_tot=%.4f | "
        "work=[%.4f %.4f %.4f %.4f] cur=[%.4f %.4f %.4f %.4f]",
        target_vx_, target_vy_, target_wz_, raw_v, target_v_signed_,
        v_smoothed_, vx, vy, wz,
        heading_smoothed_, curvature_smoothed_, log_target_curvature,
        std::abs(curvature_smoothed_ - log_target_curvature),
        speed_magnitude, dynamic_steer_rate,
        static_cast<int>(state_machine_->state()), global_align_scale, state_scale, total_scale,
        work_angles[0], work_angles[1], work_angles[2], work_angles[3],
        current_angles_[0], current_angles_[1], current_angles_[2], current_angles_[3]);

    return controller_interface::return_type::OK;
}

void RoverSwerveController::declare_parameters()
{
    auto node = get_node();

    // Declare parameters only if they haven't been declared already
    auto declare_param = [node](const auto& name, const auto& default_val) {
        try {
            node->declare_parameter(name, default_val);
        } catch (const std::exception&) {
            // Parameter already declared, ignore error
        }
    };

    declare_param("wheelbase",             1.20);
    declare_param("track_width",           0.80);
    declare_param("wheel_radius",          0.15);
    declare_param("max_steer_deg",         90.0);
    declare_param("max_steer_rate_deg",    45.0);
    declare_param("max_linear_speed",      0.50);
    declare_param("max_angular_speed",     0.50);
    declare_param("max_accel",             0.20);
    declare_param("max_decel",             0.50);
    declare_param("max_curvature_rate",    4.0);
    declare_param("control_frequency",     20.0);
    declare_param("cmd_vel_timeout_s",     2.0);
    declare_param("align_threshold_deg",   5.0);
    declare_param("scale_up_rate",         1.0);
    declare_param("scale_down_rate",       2.0);

    // Joint name arrays — override in yaml if your URDF uses different names.
    declare_param("steer_joint_names",
        std::vector<std::string>{"fl_wheel_mount_joint", "fr_wheel_mount_joint",
                                  "bl_wheel_mount_joint", "br_wheel_mount_joint"});
    declare_param("drive_joint_names",
        std::vector<std::string>{"fl_wheel_joint", "fr_wheel_joint",
                                  "bl_wheel_joint", "br_wheel_joint"});
}


bool RoverSwerveController::read_parameters()
{
    auto node = get_node();

    wheelbase_         = node->get_parameter("wheelbase").as_double();
    track_width_       = node->get_parameter("track_width").as_double();
    wheel_radius_      = node->get_parameter("wheel_radius").as_double();
    max_steer_         = node->get_parameter("max_steer_deg").as_double()      * M_PI / 180.0;
    max_steer_rate_    = node->get_parameter("max_steer_rate_deg").as_double() * M_PI / 180.0;
    max_linear_        = node->get_parameter("max_linear_speed").as_double();
    max_angular_       = node->get_parameter("max_angular_speed").as_double();
    max_accel_         = node->get_parameter("max_accel").as_double();
    max_decel_         = node->get_parameter("max_decel").as_double();
    max_curvature_rate_ = node->get_parameter("max_curvature_rate").as_double();
    control_frequency_ = node->get_parameter("control_frequency").as_double();
    cmd_vel_timeout_   = node->get_parameter("cmd_vel_timeout_s").as_double();
    align_threshold_   = node->get_parameter("align_threshold_deg").as_double() * M_PI / 180.0;
    scale_up_rate_     = node->get_parameter("scale_up_rate").as_double();
    scale_down_rate_   = node->get_parameter("scale_down_rate").as_double();

    // Reset slew limiters with updated accel/decel values.
    // limiters_[0] = SlewRateLimiter{max_accel_, max_decel_};
    // limiters_[1] = SlewRateLimiter{max_accel_, max_decel_};
    // limiters_[2] = SlewRateLimiter{max_accel_*2, max_decel_*2};

    // v_limiter_{max_decel_, max_accel};
    // heading_limiter_{max_steer_rate_, max_steer_rate_};
    // curvature_limiter_{max_accel*2, max_accel*2};

    v_limiter_ = SlewRateLimiter(max_decel_, max_accel_, 0.0);
    heading_limiter_ = AngularSlewRateLimiter(max_steer_rate_, heading_limiter_.current());
    curvature_limiter_ = SlewRateLimiter(max_curvature_rate_, max_curvature_rate_, curvature_limiter_.current());

    wz_pure_limiter_ = SlewRateLimiter(max_accel_ * 2.0, max_decel_ * 2.0, wz_pure_limiter_.current());

    // Joint names — must be exactly NUM_WHEELS entries each.
    const auto steer_names = node->get_parameter("steer_joint_names")
                                  .as_string_array();
    const auto drive_names = node->get_parameter("drive_joint_names")
                                  .as_string_array();

    if (steer_names.size() != NUM_WHEELS || drive_names.size() != NUM_WHEELS) {
        RCLCPP_ERROR(node->get_logger(),
            "[SwerveController] steer_joint_names and drive_joint_names "
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
RoverSwerveController::steer_command_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : steer_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return names;
}

std::vector<std::string>
RoverSwerveController::drive_command_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : drive_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return names;
}

std::vector<std::string>
RoverSwerveController::steer_state_interface_names() const
{
    std::vector<std::string> names;
    for (const auto & joint : steer_joint_names_) {
        names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return names;
}


bool RoverSwerveController::assign_interfaces()
{
    auto find_cmd = [this](const std::string & full_name)
        -> hardware_interface::LoanedCommandInterface*
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

    // Steer handles
    SteerHandles steer;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const std::string pos_name =
            steer_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;

        auto* cmd_iface   = find_cmd(pos_name);
        auto* state_iface = find_state(pos_name);

        if (!cmd_iface || !state_iface) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[SwerveController] Missing interface: %s", pos_name.c_str());
            return false;
        }

        steer.position_cmd.emplace_back(*cmd_iface);
        steer.position_state.emplace_back(*state_iface);
    }

    // Drive handles
    DriveHandles drive;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const std::string vel_name =
            drive_joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;

        auto* cmd_iface = find_cmd(vel_name);
        if (!cmd_iface) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[SwerveController] Missing interface: %s", vel_name.c_str());
            return false;
        }

        drive.velocity_cmd.emplace_back(*cmd_iface);
    }

    steer_handles_ = std::move(steer);
    drive_handles_ = std::move(drive);
    return true;
}

// Control helpers

void RoverSwerveController::read_current_angles()
{
   if (!steer_handles_) { return; }
   for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
#if defined(JAZZY_OR_LATER)
        current_angles_[i] = steer_handles_->position_state[i].get().get_optional().value_or(0.0);
#else
        current_angles_[i] = steer_handles_->position_state[i].get().get_value();
#endif
   }
}

void RoverSwerveController::read_measured_angles()
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

void RoverSwerveController::write_steer_commands(const WheelData & angles)
{
    if (!steer_handles_) { return; }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        (void)steer_handles_->position_cmd[i].get().set_value(angles[i]);
    }
}

void RoverSwerveController::write_drive_commands(
    const WheelData & work_angles,
    const WheelData & speeds,
    double scale)
{
    if (!drive_handles_) { return; }

    // Sign correction: if the alignment error is > 90°, cos goes negative.
    // We used cos² for magnitude scaling so the global_align_scale is always
    // positive — but we still need the sign of cos to reverse the wheel when
    // it is pointing roughly backwards relative to its target.
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double cos_err = std::cos(work_angles[i] - current_angles_[i]);
        const double sign    = (cos_err >= 0.0) ? 1.0 : -1.0;
        (void)drive_handles_->velocity_cmd[i].get().set_value(
            speeds[i] * sign * scale);
    }
}

double RoverSwerveController::step_angle(
    double current,
    double target,
    double dt,
    double rate) const
{
    const double diff = target - current;
    const double step = clamp(diff, -rate * dt, rate * dt);
    return current + step;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compact-mode service callback
// ─────────────────────────────────────────────────────────────────────────────

void RoverSwerveController::on_set_compact_mode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>       response)
{
    if (!kinematics_ || !state_machine_) {
        response->success = false;
        response->message = "Controller not yet configured";
        return;
    }

    if (request->data == kinematics_->compact_mode()) {
        response->success = true;
        response->message = "Already in requested mode";
        return;
    }

    kinematics_->set_compact_mode(request->data);

    // Decide the target angles for the TRANSIT that will now happen.
    // If the rover is idle we aim straight for the compact/normal offset
    // angles; if it's moving we recompute from current velocity.
    const bool is_idle =
        (std::abs(vx_smoothed_) < VXY_EPS) &&
        (std::abs(vy_smoothed_) < VXY_EPS) &&
        (std::abs(wz_smoothed_) < WZ_EPS);

    WheelData target;
    if (is_idle) {
        target = kinematics_->offset_angles();
    } else {
        const auto [a, s] =
            kinematics_->ik_full(vx_smoothed_, vy_smoothed_, wz_smoothed_, current_angles_);
        target = a;
        (void)s;
    }

    auto logger = get_node()->get_logger();
    // Re-enter transit so wheels pivot to new compact/normal position safely.
    state_machine_->update_transitions(
        vx_smoothed_, vy_smoothed_, wz_smoothed_,
        target, measured_angles_,
        &logger);

    response->success = true;
    response->message = std::string("Compact mode ") +
                        (kinematics_->compact_mode() ? "enabled" : "disabled");
}


bool RoverSwerveController::cmd_vel_timed_out(const rclcpp::Time & now) const
{
    return (now - last_cmd_vel_time_).seconds() > cmd_vel_timeout_;
}

}  // namespace rover_controller
