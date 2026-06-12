// ─────────────────────────────────────────────────────────────────────────────
// rover_hardware_interface.cpp
// ─────────────────────────────────────────────────────────────────────────────

#include "rover_hardware_interface/rover_hardware_interface.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverHardwareInterface,
    hardware_interface::SystemInterface)

using namespace std::chrono_literals;

namespace rover_hardware_interface {

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace {

/// Read a required string parameter from HardwareInfo; throw if missing.
std::string required_param(
    const hardware_interface::HardwareInfo & info,
    const std::string & key)
{
    auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end()) {
        throw std::runtime_error("[RoverHW] Missing required parameter: " + key);
    }
    return it->second;
}

/// Read an optional string parameter; return default_val if absent.
std::string optional_param(
    const hardware_interface::HardwareInfo & info,
    const std::string & key,
    const std::string & default_val)
{
    auto it = info.hardware_parameters.find(key);
    return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// on_init — read URDF <ros2_control> hardware parameters
//
// Expected parameters in <ros2_control> block:
//   steer_ids        = "11 13 17 15"   (space-separated, FL FR RL RR)
//   drive_ids        = "10 12 16 14"
//   drive_pmax       = "12.5"
//   drive_vmax       = "50.0"
//   drive_tmax       = "20.0"
//   mst_id           = "0"
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
RoverHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ── Motor IDs ──────────────────────────────────────────────────────────────

    // Parse space-separated ID lists from URDF parameters
    auto parse_ids = [](const std::string & s, std::array<uint8_t, NUM_WHEELS> & out) {
        std::istringstream ss(s);
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            int v;
            if (!(ss >> v) || v < 0 || v > 255) {
                throw std::runtime_error(
                    "[RoverHW] Invalid motor ID '" + std::to_string(v) +
                    "' — must be 0..255");
            }
            out[i] = static_cast<uint8_t>(v);
        }
    };

    parse_ids(required_param(info, "steer_ids"), steer_ids_);
    parse_ids(required_param(info, "drive_ids"), drive_ids_);

    drive_pmax_ = std::stof(optional_param(info, "drive_pmax", "12.5"));
    drive_vmax_ = std::stof(optional_param(info, "drive_vmax", "50.0"));
    drive_tmax_ = std::stof(optional_param(info, "drive_tmax", "20.0"));
    mst_id_     = static_cast<uint32_t>(std::stoi(optional_param(info, "mst_id", "0")));

    if (drive_pmax_ <= 0.0f || drive_vmax_ <= 0.0f || drive_tmax_ <= 0.0f) {
        RCLCPP_ERROR(get_logger(), "[RoverHW] drive_pmax/vmax/tmax must be > 0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ── Joint names from URDF <joint> blocks ──────────────────────────────────
    // Joint order in URDF must be: fl_steer, fr_steer, rl_steer, rr_steer,
    //                              fl_drive,  fr_drive,  rl_drive,  rr_drive

    if (info.joints.size() != NUM_WHEELS * 2) {
        RCLCPP_ERROR(get_logger(),
            "[RoverHW] Expected %zu joints, got %zu",
            NUM_WHEELS * 2, info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        steer_joint_names_[i] = info.joints[i].name;
        drive_joint_names_[i] = info.joints[i + NUM_WHEELS].name;
    }

    // ── Zero all backing storage ───────────────────────────────────────────────
    steer_pos_.fill(0.0);
    drive_pos_.fill(0.0);
    drive_vel_.fill(0.0);
    steer_cmd_.fill(0.0);
    drive_cmd_.fill(0.0);

    RCLCPP_INFO(get_logger(),
        "[RoverHW] Initialized — "
        "steer [%d,%d,%d,%d]  drive [%d,%d,%d,%d]",
        steer_ids_[0], steer_ids_[1], steer_ids_[2], steer_ids_[3],
        drive_ids_[0], drive_ids_[1], drive_ids_[2], drive_ids_[3]);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
//
// ros2_control binds pointers to our backing arrays here.
// Order / naming must match what SwerveController requests.
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
RoverHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    ifaces.reserve(NUM_WHEELS * 3);   // steer pos + drive pos + drive vel

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        // Steering: position feedback from Steadywin
        ifaces.emplace_back(
            steer_joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &steer_pos_[i]);

        // Drive: position + velocity from Damiao MIT feedback
        ifaces.emplace_back(
            drive_joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &drive_pos_[i]);

        ifaces.emplace_back(
            drive_joint_names_[i],
            hardware_interface::HW_IF_VELOCITY,
            &drive_vel_[i]);
    }

    return ifaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
RoverHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    ifaces.reserve(NUM_WHEELS * 2);   // steer position cmd + drive velocity cmd

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        ifaces.emplace_back(
            steer_joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &steer_cmd_[i]);

        ifaces.emplace_back(
            drive_joint_names_[i],
            hardware_interface::HW_IF_VELOCITY,
            &drive_cmd_[i]);
    }

    return ifaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure — create ROS 2 publishers, subscriber, services, timers
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
RoverHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto node = get_node();

    // ── CAN I/O ────────────────────────────────────────────────────────────────
    to_can_pub_ = node->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);

    from_can_sub_ = node->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 10,
        [this](can_msgs::msg::Frame::SharedPtr msg) { on_can_frame(msg); });

    // ── Status publishers ──────────────────────────────────────────────────────
    chassis_status_pub_ = node->create_publisher<indomitus_interfaces::msg::ChassisStatus>(
        "/chassis/motor_states", 10);
    diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);

    // ── Services ───────────────────────────────────────────────────────────────
    motor_enable_srv_ = node->create_service<std_srvs::srv::SetBool>(
        "~/set_motors_enabled",
        [this](
            const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
            std::shared_ptr<std_srvs::srv::SetBool::Response>       res)
        { on_set_motors_enabled(req, res); });

    set_steer_zero_srv_ = node->create_service<indomitus_interfaces::srv::SetSteerZero>(
        "~/set_steer_zero",
        [this](
            const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request>  req,
            std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response>       res)
        { on_set_steer_zero(req, res); });

    // ── Timers ─────────────────────────────────────────────────────────────────

    // 1 Hz: poll motor status and absolute position
    status_poll_timer_ = node->create_wall_timer(1s, [this]() {
        if (!motors_enabled_) return;
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            to_can_pub_->publish(steadywin_protocol::buildStatusQueryFrame(steer_ids_[i]));
            to_can_pub_->publish(steadywin_protocol::buildAbsAngleQueryFrame(steer_ids_[i]));
            to_can_pub_->publish(damiao_protocol::buildReadRegisterFrame(drive_ids_[i], 80));
        }
    });

    // 10 Hz: publish /chassis/motor_states
    chassis_status_timer_ = node->create_wall_timer(
        100ms, [this]() { publish_chassis_status(); });

    // 1 Hz: publish /diagnostics
    diagnostics_timer_ = node->create_wall_timer(
        1s, [this]() { publish_diagnostics(); });

    // 10 Hz watchdog: if write() stops being called, zero all motors
    last_write_time_ = node->get_clock()->now();
    watchdog_timer_ = node->create_wall_timer(100ms, [this]() {
        if (!motors_enabled_) return;
        const double elapsed =
            (get_node()->get_clock()->now() - last_write_time_).seconds();
        if (elapsed < kWatchdogTimeoutSec) return;

        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            to_can_pub_->publish(
                steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f));
            to_can_pub_->publish(
                damiao_protocol::buildVelocityFrame(drive_ids_[i], 0.0f));
        }
    });

    RCLCPP_INFO(get_logger(), "[RoverHW] Configured.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate — enable motors, seed write timestamp
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
RoverHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    last_write_time_ = get_node()->get_clock()->now();

    // Attempt to put motors in a known safe state on boot.
    // If CAN bridge isn't ready yet, retry for up to 5 s.
    try_publish_boot_disable();

    send_enable_frames();

    RCLCPP_INFO(get_logger(), "[RoverHW] Activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate — graceful shutdown: zero → settle → disable
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
RoverHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    send_shutdown_frames();
    RCLCPP_INFO(get_logger(), "[RoverHW] Deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// read — copy latest CAN feedback into state interface backing storage
//
// Called by controller_manager before every controller update().
// on_can_frame() already decoded feedback into steer_state_ / drive_state_
// so here we just forward the values.
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type
RoverHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        // Steering position — prefer Steadywin absolute multi-turn feedback
        steer_pos_[i] = steer_state_[i].pos_valid
            ? static_cast<double>(steer_state_[i].pos_rad)
            : 0.0;

        // Drive position and velocity — Damiao MIT feedback
        drive_pos_[i] = drive_state_[i].valid
            ? static_cast<double>(drive_state_[i].pos)
            : 0.0;
        drive_vel_[i] = drive_state_[i].valid
            ? static_cast<double>(drive_state_[i].vel)
            : 0.0;
    }

    return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write — translate command interfaces → CAN frames
//
// Called by controller_manager after every controller update().
// steer_cmd_[i] — target steering position [rad]    → Steadywin 0xC2
// drive_cmd_[i] — target drive velocity   [rad/s]   → Damiao    0x200
//
// Left wheels (FL=0, RL=2) are physically mirrored so their drive direction
// is negated — matching the original onWheelTargets() sign convention.
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type
RoverHardwareInterface::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    last_write_time_ = get_node()->get_clock()->now();

    if (!motors_enabled_) return hardware_interface::return_type::OK;

    // Drive sign convention (mirrors original onWheelTargets):
    //   FL (0): negated   FR (1): normal   RL (2): negated   RR (3): normal
    static constexpr std::array<float, NUM_WHEELS> kDriveSign = {-1.0f, 1.0f, -1.0f, 1.0f};

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(
            steadywin_protocol::buildAbsPositionFrame(
                steer_ids_[i],
                static_cast<float>(steer_cmd_[i])));

        to_can_pub_->publish(
            damiao_protocol::buildVelocityFrame(
                drive_ids_[i],
                static_cast<float>(drive_cmd_[i]) * kDriveSign[i]));
    }

    return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_can_frame — decode incoming CAN feedback
//
// Routing logic mirrors the original ChassisDriverNode::onCanFrame() exactly.
// ─────────────────────────────────────────────────────────────────────────────

void RoverHardwareInterface::on_can_frame(const can_msgs::msg::Frame::SharedPtr msg)
{
    // ── Damiao drive: feedback at ESC_ID ──────────────────────────────────────
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (msg->id == drive_ids_[i]) {
            damiao_protocol::parseFeedback(
                msg->data, msg->dlc,
                drive_ids_[i],
                drive_pmax_, drive_vmax_, drive_tmax_,
                drive_state_[i]);
            damiao_protocol::parseRegisterResponse(
                msg->data, msg->dlc,
                drive_ids_[i],
                drive_state_[i]);
            return;
        }
    }

    // ── Damiao drive: broadcast feedback at MST_ID ────────────────────────────
    if (msg->id == mst_id_) {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            if (damiao_protocol::parseFeedback(
                    msg->data, msg->dlc,
                    drive_ids_[i],
                    drive_pmax_, drive_vmax_, drive_tmax_,
                    drive_state_[i]))
            {
                break;
            }
        }
        if (msg->dlc >= 8 && msg->data[2] == 0x33) {
            for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
                if (damiao_protocol::parseRegisterResponse(
                        msg->data, msg->dlc,
                        drive_ids_[i],
                        drive_state_[i]))
                {
                    break;
                }
            }
        }
        return;
    }

    // ── Steadywin steer: response at esc_id or 0x100|esc_id ───────────────────
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (msg->id == steer_ids_[i] ||
            msg->id == (0x100u | steer_ids_[i]))
        {
            steadywin_protocol::parseResponse(
                msg->data, msg->dlc,
                steer_state_[i]);
            return;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void RoverHardwareInterface::send_enable_frames()
{
    RCLCPP_INFO(get_logger(), "[RoverHW] Enabling all motors");

    // Steadywin: clear fault → home position to activate
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(steadywin_protocol::buildClearFaultFrame(steer_ids_[i]));
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(
            steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f));
    }

    // Damiao: TIMEOUT watchdog (reg 9, 200 ms) → velocity mode → enable
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(
            damiao_protocol::buildWriteRegisterUint32Frame(drive_ids_[i], 9, 200u));
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(damiao_protocol::buildSetModeFrame(drive_ids_[i], 3));
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(damiao_protocol::buildEnableFrame(drive_ids_[i]));
    }

    motors_enabled_ = true;
    RCLCPP_INFO(get_logger(), "[RoverHW] All motors enabled");
}

void RoverHardwareInterface::send_disable_frames()
{
    RCLCPP_INFO(get_logger(), "[RoverHW] Disabling all motors");

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(steadywin_protocol::buildDisableFrame(steer_ids_[i]));
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(damiao_protocol::buildDisableFrame(drive_ids_[i]));
    }

    motors_enabled_ = false;
    RCLCPP_INFO(get_logger(), "[RoverHW] All motors disabled");
}

void RoverHardwareInterface::send_shutdown_frames()
{
    if (!motors_enabled_) {
        send_disable_frames();
        return;
    }

    RCLCPP_INFO(get_logger(), "[RoverHW] Shutdown: zeroing commands");

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        to_can_pub_->publish(
            steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f));
        to_can_pub_->publish(
            damiao_protocol::buildVelocityFrame(drive_ids_[i], 0.0f));
    }

    RCLCPP_INFO(get_logger(), "[RoverHW] Waiting 1.5 s for motion to settle…");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    send_disable_frames();
}

// ─────────────────────────────────────────────────────────────────────────────
// Boot-time CAN subscriber detection
// ─────────────────────────────────────────────────────────────────────────────

void RoverHardwareInterface::try_publish_boot_disable()
{
    if (to_can_pub_->get_subscription_count() > 0u) {
        send_disable_frames();
        return;
    }

    boot_retry_attempts_ = 0;
    boot_retry_timer_ = get_node()->create_wall_timer(200ms, [this]() {
        if (to_can_pub_->get_subscription_count() > 0u) {
            RCLCPP_INFO(get_logger(),
                "[RoverHW] /to_can_bus subscriber detected — sending boot disable");
            send_disable_frames();
            boot_retry_timer_->cancel();
            return;
        }
        if (++boot_retry_attempts_ >= kBootRetryMax) {
            RCLCPP_WARN(get_logger(),
                "[RoverHW] No /to_can_bus subscriber after %d attempts (~%.1f s); "
                "boot disable frames may have been lost",
                boot_retry_attempts_,
                boot_retry_attempts_ * 0.2);
            boot_retry_timer_->cancel();
        }
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Services
// ─────────────────────────────────────────────────────────────────────────────

void RoverHardwareInterface::on_set_motors_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
    std::shared_ptr<std_srvs::srv::SetBool::Response>       res)
{
    if (req->data) {
        send_enable_frames();
        res->success = true;
        res->message = "All chassis motors enabled";
    } else {
        send_disable_frames();
        res->success = true;
        res->message = "All chassis motors disabled";
    }
}

void RoverHardwareInterface::on_set_steer_zero(
    const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request>  req,
    std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response>       res)
{
    if (!motors_enabled_) {
        res->success = false;
        res->message = "Motors not enabled — cannot set zero";
        return;
    }

    static constexpr std::array<const char *, NUM_WHEELS> kNames = {"FL","FR","RL","RR"};

    const bool zero_all = req->motor_ids.empty();
    std::string zeroed, unknown;

    auto zero_one = [&](std::size_t i) {
        to_can_pub_->publish(steadywin_protocol::buildSetOriginFrame(steer_ids_[i]));
        zeroed += kNames[i];
        zeroed += '(';
        zeroed += std::to_string(steer_ids_[i]);
        zeroed += ") ";
        RCLCPP_INFO(get_logger(), "[RoverHW] Set steer zero: %s (id=%d)",
            kNames[i], steer_ids_[i]);
    };

    if (zero_all) {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) { zero_one(i); }
    } else {
        for (const uint8_t req_id : req->motor_ids) {
            bool found = false;
            for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
                if (steer_ids_[i] == req_id) { zero_one(i); found = true; break; }
            }
            if (!found) {
                unknown += std::to_string(req_id);
                unknown += ' ';
            }
        }
    }

    if (!unknown.empty()) {
        res->success = false;
        res->message = "Unknown steer IDs: " + unknown +
                       "— valid: " +
                       std::to_string(steer_ids_[0]) + " " +
                       std::to_string(steer_ids_[1]) + " " +
                       std::to_string(steer_ids_[2]) + " " +
                       std::to_string(steer_ids_[3]);
        return;
    }

    res->success = true;
    res->message = "Origin set for: " + zeroed;
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostic / status publishers
// ─────────────────────────────────────────────────────────────────────────────

void RoverHardwareInterface::publish_chassis_status()
{
    indomitus_interfaces::msg::ChassisStatus msg;
    msg.header.stamp = get_node()->get_clock()->now();

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state_[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id         = steer_ids_[i];
        m.motor_type     = "steadywin";
        m.joint_name     = steer_joint_names_[i];
        m.position       = s.pos_valid ? s.pos_rad : 0.0f;
        m.velocity       = 0.0f;
        m.torque         = 0.0f;
        m.kinematic_valid = s.pos_valid;
        m.voltage        = s.voltage;
        m.current        = s.bus_current;
        m.temperature    = static_cast<float>(s.temperature);
        m.mode           = s.mode;
        m.fault_code     = s.fault_code;
        m.health_valid   = s.diag_valid;
        m.enabled        = motors_enabled_;
        msg.motors.push_back(m);
    }

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = drive_state_[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id         = drive_ids_[i];
        m.motor_type     = "damiao";
        m.joint_name     = drive_joint_names_[i];
        m.position       = s.valid ? s.pos : 0.0f;
        m.velocity       = s.valid ? s.vel : 0.0f;
        m.torque         = s.valid ? s.tor : 0.0f;
        m.kinematic_valid = s.valid;
        m.voltage        = 0.0f;
        m.current        = 0.0f;
        m.temperature    = static_cast<float>(s.t_mos);
        m.mode           = s.valid ? 3u : 0u;
        m.fault_code     = (s.valid && s.err != 0x1) ? 0x01u : 0x00u;
        m.health_valid   = s.valid;
        m.enabled        = motors_enabled_ && s.valid && s.err == 0x1;
        msg.motors.push_back(m);
    }

    chassis_status_pub_->publish(msg);
}

void RoverHardwareInterface::publish_diagnostics()
{
    static constexpr std::array<const char *, NUM_WHEELS> kNames = {"FL","FR","RL","RR"};

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = get_node()->get_clock()->now();

    auto kv = [](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue p;
        p.key = k; p.value = v;
        return p;
    };

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state_[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = std::string("steadywin/steer_") + kNames[i];
        st.hardware_id = st.name;

        if (!s.diag_valid) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "No status received";
        } else if (s.fault_code != 0) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            std::string faults;
            if (s.fault_code & 0x01) faults += "voltage ";
            if (s.fault_code & 0x02) faults += "current ";
            if (s.fault_code & 0x04) faults += "temperature ";
            if (s.fault_code & 0x08) faults += "encoder ";
            if (s.fault_code & 0x40) faults += "hardware ";
            if (s.fault_code & 0x80) faults += "software ";
            st.message = "FAULT: " + faults;
        } else {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = "OK";
        }

        st.values.push_back(kv("pos_rad",       std::to_string(s.pos_rad)));
        st.values.push_back(kv("voltage_V",     std::to_string(s.voltage)));
        st.values.push_back(kv("current_A",     std::to_string(s.bus_current)));
        st.values.push_back(kv("temperature_C", std::to_string(s.temperature)));
        st.values.push_back(kv("mode",          std::to_string(s.mode)));
        st.values.push_back(kv("fault_code",    std::to_string(s.fault_code)));
        arr.status.push_back(st);
    }

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = drive_state_[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = std::string("damiao/drive_") + kNames[i];
        st.hardware_id = st.name;

        if (!s.valid) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "No feedback received";
        } else if (s.err == 0x1) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = "Enabled";
        } else {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "Disabled or fault";
        }

        st.values.push_back(kv("pos_rad",    std::to_string(s.pos)));
        st.values.push_back(kv("vel_rad_s",  std::to_string(s.vel)));
        st.values.push_back(kv("tor_Nm",     std::to_string(s.tor)));
        st.values.push_back(kv("t_mos_C",    std::to_string(s.t_mos)));
        st.values.push_back(kv("t_rotor_C",  std::to_string(s.t_rotor)));
        st.values.push_back(kv("err_code",   std::to_string(s.err)));
        arr.status.push_back(st);
    }

    diagnostics_pub_->publish(arr);
}

}  // namespace rover_hardware_interface