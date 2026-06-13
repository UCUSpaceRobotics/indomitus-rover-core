#include "rover_chassis_controller/swerve_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
    rover_chassis_controller::RoverSwerveController,
    controller_interface::ControllerInterface)

namespace rover_chassis_controller {


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

    for (auto & lim : limiters_) { lim.reset(0.0); }

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

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        current_angles_[i] =
            steer_handles_->position_state[i].get().get_optional().value_or(0.0);
    }

    last_cmd_vel_time_ = get_node()->get_clock()->now();

    // read_current_angles();

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
            std::ignore = drive_handles_->velocity_cmd[i].get().set_value(0.0);
        }
    }

    // Release optional handles — this returns loaned interfaces to controller_manager.
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

    for (const auto & name : steer_command_interface_names()) {
        cfg.names.push_back(name);
    }
    for (const auto & name : drive_command_interface_names()) {
        cfg.names.push_back(name);
    }
    return cfg;
}


controller_interface::InterfaceConfiguration
RoverSwerveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & name : steer_state_interface_names()) {
        cfg.names.push_back(name);
    }
    return cfg;
}

controller_interface::return_type
RoverSwerveController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    const double dt = period.seconds();

    if (cmd_vel_timed_out(time)) {
        target_vx_ = target_vy_ = target_wz_ = 0.0;
    }

    // Step 1: Smooth chassis velocities via independent slew-rate limiters
    vx_smoothed_ = limiters_[0].update(target_vx_, dt);
    vy_smoothed_ = limiters_[1].update(target_vy_, dt);
    wz_smoothed_ = limiters_[2].update(target_wz_, dt);

    const double vx = vx_smoothed_;
    const double vy = vy_smoothed_;
    const double wz = wz_smoothed_;

    // Step 2: Read current hardware steering positions
    read_current_angles();

    // Step 3: Compute desired target angles for transition detection
    const auto [desired_angles, desired_speeds_unused] =
        kinematics_->ik_full(vx, vy, wz, current_angles_);
    (void)desired_speeds_unused;

    auto logger = get_node()->get_logger();
    state_machine_->update_transitions(
        vx, vy, wz,
        desired_angles, current_angles_,
        &logger);

    // Step 4: Compute active execution vectors for the current state
    WheelData work_angles, work_speeds;

    switch (state_machine_->state()) {

        case RoverState::NORMAL: {
            auto [a, s] = kinematics_->ik_full(vx, vy, wz, current_angles_);
            work_angles = a;
            work_speeds = s;
            break;
        }

        case RoverState::ROTATE: {
            auto [a, s] = kinematics_->ik_rotate(wz, current_angles_);
            work_angles = a;
            work_speeds = s;
            break;
        }

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
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        current_angles_[i] = step_angle(current_angles_[i], work_angles[i], dt);

        const double before = current_angles_[i];
        current_angles_[i] = step_angle(current_angles_[i], work_angles[i], dt);

        RCLCPP_INFO_THROTTLE(get_node()->get_logger(),
            *get_node()->get_clock(), 500,
            "[steer %zu] hw=%.1f° target=%.1f° after_step=%.1f° step=%.2f°/cycle  max_step=%.2f°/cycle",
            i,
            before                  * 180.0 / M_PI,
            work_angles[i]          * 180.0 / M_PI,
            current_angles_[i]      * 180.0 / M_PI,
            (current_angles_[i] - before) * 180.0 / M_PI,
            max_steer_rate_ * dt    * 180.0 / M_PI);
    }

    // Step 6: Write steering position commands
    write_steer_commands(current_angles_);

    // Step 7: Global speed scale — cosine alignment penalty.

    double global_align_scale = 1.0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const double c = std::cos(work_angles[i] - current_angles_[i]);
        global_align_scale = std::min(global_align_scale, c * c);
    }

    const double state_scale = state_machine_->update_scale(
        dt, work_angles, current_angles_, &logger);

    const double total_scale = global_align_scale * state_scale;

    // Step 8: Write drive velocity commands
    write_drive_commands(work_angles, work_speeds, total_scale);

    return controller_interface::return_type::OK;
}

void RoverSwerveController::declare_parameters()
{
    auto node = get_node();

    node->declare_parameter("wheelbase",             1.20);
    node->declare_parameter("track_width",           0.80);
    node->declare_parameter("wheel_radius",          0.15);
    node->declare_parameter("max_steer_deg",         90.0);
    node->declare_parameter("max_steer_rate_deg",    45.0);
    node->declare_parameter("max_linear_speed",      0.50);
    node->declare_parameter("max_angular_speed",     0.50);
    node->declare_parameter("max_accel",             0.20);
    node->declare_parameter("max_decel",             0.50);
    node->declare_parameter("control_frequency",     20.0);
    node->declare_parameter("cmd_vel_timeout_s",     2.0);
    node->declare_parameter("align_threshold_deg",   5.0);
    node->declare_parameter("scale_up_rate",         1.0);
    node->declare_parameter("scale_down_rate",       2.0);

    // Joint name arrays — override in yaml if your URDF uses different names.
    node->declare_parameter("steer_joint_names",
        std::vector<std::string>{"fl_wheel_mount_joint", "fr_wheel_mount_joint",
                                  "bl_wheel_mount_joint", "br_wheel_mount_joint"});
    node->declare_parameter("drive_joint_names",
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
    control_frequency_ = node->get_parameter("control_frequency").as_double();
    cmd_vel_timeout_   = node->get_parameter("cmd_vel_timeout_s").as_double();
    align_threshold_   = node->get_parameter("align_threshold_deg").as_double() * M_PI / 180.0;
    scale_up_rate_     = node->get_parameter("scale_up_rate").as_double();
    scale_down_rate_   = node->get_parameter("scale_down_rate").as_double();

    // Reset slew limiters with updated accel/decel values.
    limiters_[0] = SlewRateLimiter{max_accel_, max_decel_};
    limiters_[1] = SlewRateLimiter{max_accel_, max_decel_};
    limiters_[2] = SlewRateLimiter{max_accel_, max_decel_};

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

    // Steer handles
    SteerHandles steer;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const std::string pos_name =
            steer_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;

        auto * cmd_iface   = find_cmd(pos_name);
        auto * state_iface = find_state(pos_name);

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

        auto * cmd_iface = find_cmd(vel_name);
        if (!cmd_iface) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "[SwerveController] Missing interface: %s", vel_name.c_str());
            return false;
        }

        drive.velocity_cmd.emplace_back(*cmd_iface);  // ← emplace_back
    }

    steer_handles_ = std::move(steer);
    drive_handles_ = std::move(drive);
    return true;
}

// Control helpers

void RoverSwerveController::read_current_angles()
{
//    if (!steer_handles_) { return; }
//    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
//        current_angles_[i] =
//            steer_handles_->position_state[i].get().get_optional().value_or(0.0);
//    }
}

void RoverSwerveController::write_steer_commands(const WheelData & angles)
{
    if (!steer_handles_) { return; }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        std::ignore = steer_handles_->position_cmd[i].get().set_value(angles[i]);
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
        std::ignore = drive_handles_->velocity_cmd[i].get().set_value(
            speeds[i] * sign * scale);
    }
}

double RoverSwerveController::step_angle(
    double current,
    double target,
    double dt) const
{
    const double diff = target - current;
    const double step = clamp(diff, -max_steer_rate_ * dt, max_steer_rate_ * dt);
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
        target, current_angles_,
        &logger);

    response->success = true;
    response->message = std::string("Compact mode ") +
                        (kinematics_->compact_mode() ? "enabled" : "disabled");
}


bool RoverSwerveController::cmd_vel_timed_out(const rclcpp::Time & now) const
{
    return (now - last_cmd_vel_time_).seconds() > cmd_vel_timeout_;
}

}  // namespace rover_swerve_controller
