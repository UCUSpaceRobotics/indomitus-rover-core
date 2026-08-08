#include "arm_hardware_interface/arm_hardware_interface.hpp"
#include "arm_hardware_interface/steadywin_protocol.hpp"
#include "arm_hardware_interface/damiao_wrist_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::ArmCanSystem,
    hardware_interface::SystemInterface)

namespace arm_hardware_interface {

namespace sw = steadywin_protocol;
namespace dm = damiao_wrist_protocol;

// Smooth 0->1 ramp (smoothstep) for stiffness ramping after enable.
static inline double smoothstep01(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

static double param_or(const std::unordered_map<std::string, std::string>& params,
                       const std::string& key, double fallback)
{
    auto it = params.find(key);
    if (it == params.end()) return fallback;
    try { return std::stod(it->second); } catch (...) { return fallback; }
}

// =============================================================================
// Lifecycle
// =============================================================================

#ifdef JAZZY_OR_LATER
hardware_interface::CallbackReturn ArmCanSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return init_from_info(params.hardware_info);
}
#else
hardware_interface::CallbackReturn ArmCanSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return init_from_info(info);
}
#endif

hardware_interface::CallbackReturn ArmCanSystem::init_from_info(
    const hardware_interface::HardwareInfo & info)
{
    if (info.joints.size() != NUM_JOINTS) {
        RCLCPP_ERROR(logger_, "Expected %zu joints, got %zu", NUM_JOINTS, info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ---- Hardware-level parameters ----
    if (info.hardware_parameters.count("can_interface")) {
        can_interface_ = info.hardware_parameters.at("can_interface");
    } else {
        RCLCPP_WARN(logger_, "can_interface param missing in URDF, defaulting to can0");
    }
    gain_ramp_secs_        = param_or(info.hardware_parameters, "gain_ramp_secs", gain_ramp_secs_);
    max_cmd_speed_rad_s_   = param_or(info.hardware_parameters, "max_cmd_speed_rad_s", max_cmd_speed_rad_s_);
    feedback_timeout_secs_ = param_or(info.hardware_parameters, "feedback_timeout_secs", feedback_timeout_secs_);

    // ---- Per-joint parameters: direction, offset, kp, kd, motor_id ----
    // These live in arm_macro.xacro so recalibration never requires recompiling.
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const auto& j = info.joints[i];
        joint_names_[i] = j.name;

        const double dir = param_or(j.parameters, "direction", joint_directions_[i]);
        if (dir != 1.0 && dir != -1.0) {
            RCLCPP_ERROR(logger_, "Joint '%s': direction must be 1 or -1 (got %f)",
                         j.name.c_str(), dir);
            return hardware_interface::CallbackReturn::ERROR;
        }
        joint_directions_[i] = dir;
        joint_offsets_[i]    = param_or(j.parameters, "offset", joint_offsets_[i]);
        joint_kp_[i]         = param_or(j.parameters, "kp", joint_kp_[i]);
        joint_kd_[i]         = param_or(j.parameters, "kd", joint_kd_[i]);
        motor_ids_[i]        = static_cast<uint8_t>(
                                   param_or(j.parameters, "motor_id",
                                            static_cast<double>(motor_ids_[i])));

        // Damiao manual: kd must not be 0 while kp > 0 or the motor oscillates.
        if (i >= NUM_STEADYWIN && joint_kp_[i] > 0.0 && joint_kd_[i] <= 0.0) {
            RCLCPP_ERROR(logger_, "Joint '%s' (Damiao): kd must be > 0 when kp > 0", j.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_,
            "Joint %zu '%s' -> motor %u  dir=%+.0f offset=%.4f rad  kp=%.1f kd=%.2f",
            i, j.name.c_str(), motor_ids_[i], joint_directions_[i],
            joint_offsets_[i], joint_kp_[i], joint_kd_[i]);
    }

    joint_position_command_.fill(0.0);
    joint_velocity_command_.fill(0.0);
    joint_position_state_.fill(0.0);
    joint_velocity_state_.fill(0.0);
    hw_position_states_.fill(0.0);
    hw_velocity_states_.fill(0.0);
    feedback_seen_.fill(false);
    dm_last_err_.fill(0x01);

    RCLCPP_INFO(logger_, "ArmCanSystem initialized (CAN: %s, ramp: %.1fs, cmd speed limit: %.2f rad/s)",
                can_interface_.c_str(), gain_ramp_secs_, max_cmd_speed_rad_s_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmCanSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_state_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmCanSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ArmCanSystem::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(logger_, "Configuring ArmCanSystem...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_activate(const rclcpp_lifecycle::State&)
{
    if (!open_can_socket()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    feedback_seen_.fill(false);
    rx_running_.store(true);
    rx_thread_ = std::thread(&ArmCanSystem::rx_thread_fn, this);

    // 1) Enable: Steadywin gets its 0xF0 limits config (does NOT apply torque —
    //    it only enters MIT mode on the first 0x4xx frame). Damiao gets 0xFC
    //    (enabled but torqueless until its first MIT command).
    send_enable_frames();

    // 2) ACTIVELY poll every motor for its true position. Motors only reply
    //    when spoken to — the old passive 200 ms wait received nothing, left
    //    the state at 0, and caused the startup jump. If any motor stays
    //    silent, we ABORT instead of guessing.
    if (!wait_for_all_feedback(feedback_timeout_secs_)) {
        RCLCPP_FATAL(logger_,
            "Not all motors reported their position — REFUSING to enable torque "
            "with an unknown arm pose. Check CAN wiring / IDs / power.");
        safe_stop();
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3) Sync commands and states to the measured pose (URDF frame),
    //    so the first MIT frame holds the arm exactly where it is.
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        joint_position_state_  = hw_position_states_;
        joint_velocity_state_  = hw_velocity_states_;
    }
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        joint_position_command_[i] = joint_position_state_[i];
        joint_velocity_command_[i] = 0.0;
        last_sent_command_[i]      = joint_position_state_[i];
        RCLCPP_INFO(logger_, "  '%s' start pose: %.4f rad (motor frame %.4f)",
                    joint_names_[i].c_str(), joint_position_state_[i],
                    urdf_to_motor(i, joint_position_state_[i]));
    }
    have_last_sent_ = true;
    ramp_started_   = false;   // ramp clock starts at the first write()
    motors_enabled_.store(true);

    RCLCPP_INFO(logger_,
        "ArmCanSystem activated. Stiffness will ramp 0 -> 100%% over %.1f s. "
        "Do not command motion until the ramp completes.", gain_ramp_secs_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_WARN(logger_,
        "Deactivating WITHOUT disabling: Steadywin motors (base/shoulder/"
        "elbow) hold their last commanded position indefinitely — they need "
        "no further CAN traffic to stay stiff. Damiao wrist motors (forearm/"
        "wrist/end-effector) WILL go limp shortly after this process stops "
        "sending frames (their own comm-loss watchdog disables them) — "
        "support the wrist/end-effector before deactivating.");
    stop_holding();
    RCLCPP_INFO(logger_, "ArmCanSystem deactivated (motors left enabled, holding last position).");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
    stop_holding();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_error(const rclcpp_lifecycle::State&)
{
    // A real fault means we no longer trust our own state (bad feedback,
    // CAN issues, etc.) — unlike a clean deactivate, holding position here
    // could mean holding a WRONG position with real torque. Fail-safe: cut
    // power instead.
    RCLCPP_ERROR(logger_, "on_error: disabling all motors (fail-safe).");
    safe_stop();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void ArmCanSystem::safe_stop()
{
    motors_enabled_.store(false);
    if (can_fd_ >= 0) {
        send_disable_frames();
    }
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    close_can_socket();
}

void ArmCanSystem::stop_holding()
{
    // Deliberately does NOT call send_disable_frames(): Steadywin keeps
    // executing its last MIT command (position/kp/kd) with no further
    // traffic required, so it stays holding after this process exits.
    // Damiao motors will drop out on their own comm-loss watchdog shortly
    // after we stop streaming — nothing we send here changes that.
    motors_enabled_.store(false);
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    close_can_socket();
}

// =============================================================================
// read / write
// =============================================================================

hardware_interface::return_type ArmCanSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    joint_position_state_ = hw_position_states_;
    joint_velocity_state_ = hw_velocity_states_;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmCanSystem::write(const rclcpp::Time&, const rclcpp::Duration& period)
{
    if (!motors_enabled_.load()) return hardware_interface::return_type::OK;

    // ---- Stiffness ramp: 0 -> 1 over gain_ramp_secs_ from the first write ----
    if (!ramp_started_) {
        ramp_start_   = std::chrono::steady_clock::now();
        ramp_started_ = true;
    }
    double ramp = 1.0;
    if (gain_ramp_secs_ > 0.05) {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ramp_start_).count();
        ramp = smoothstep01(elapsed / gain_ramp_secs_);
    }

    // ---- Command rate limiter: never step the target faster than
    //      max_cmd_speed_rad_s_, no matter what the controller asks for.
    //      This is the last line of defense against "jump to wrong pose". ----
    double dt = period.seconds();
    if (!(dt > 0.0) || dt > 0.5) dt = 0.01;
    const double max_step = max_cmd_speed_rad_s_ * dt;

    std::array<double, NUM_JOINTS> cmd{};
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        double target = joint_position_command_[i];
        if (!std::isfinite(target)) {
            target = last_sent_command_[i];   // hold on NaN/inf
        }
        const double delta = std::clamp(target - last_sent_command_[i], -max_step, max_step);
        cmd[i] = last_sent_command_[i] + delta;
        last_sent_command_[i] = cmd[i];
    }

    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        const float target_pos = static_cast<float>(urdf_to_motor(i, cmd[i]));
        double vff = joint_velocity_command_[i];
        if (!std::isfinite(vff)) vff = 0.0;
        const float target_vel = static_cast<float>(vff * joint_directions_[i]);
        const float kp = static_cast<float>(joint_kp_[i] * ramp);
        const float kd = static_cast<float>(joint_kd_[i] * ramp);

        can_msgs::msg::Frame f = (i < NUM_STEADYWIN)
            ? sw::build_mit_command_frame(motor_ids_[i], target_pos, target_vel, kp, kd, 0.0f)
            : dm::build_mit_command_frame(motor_ids_[i], target_pos, target_vel, kp, kd, 0.0f);
        send_can_frame(f.id, f.data, f.dlc);
    }

    return hardware_interface::return_type::OK;
}

// =============================================================================
// SocketCAN
// =============================================================================

bool ArmCanSystem::open_can_socket()
{
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to open CAN socket: %s", std::strerror(errno));
        return false;
    }

    // Receive timeout so rx_thread_fn can notice rx_running_ == false and the
    // deactivate path never hangs on join() (the old blocking read could).
    struct timeval tv{};
    tv.tv_sec = 0; tv.tv_usec = 100000;   // 100 ms
    setsockopt(can_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "ioctl failed for %s: %s", can_interface_.c_str(), std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "CAN bind failed: %s", std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "SocketCAN interface %s opened.", can_interface_.c_str());
    return true;
}

void ArmCanSystem::close_can_socket()
{
    if (can_fd_ >= 0) {
        close(can_fd_);
        can_fd_ = -1;
    }
}

bool ArmCanSystem::send_can_frame(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t dlc)
{
    if (can_fd_ < 0) return false;
    struct can_frame frame{};
    frame.can_id  = id;            // all IDs here fit in 11-bit standard frames
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data.data(), dlc);
    return ::write(can_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

// =============================================================================
// Motor sequences
// =============================================================================

void ArmCanSystem::send_enable_frames()
{
    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
        auto f = sw::build_config_limits_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
        auto f = dm::build_enable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void ArmCanSystem::send_disable_frames()
{
    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    // Damiao: zero-gain MIT frame first to avoid a torque discontinuity
    // (mirrors dm_disable() in the proven Python tools), then 0xFD.
    for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
        auto probe = dm::build_zero_gain_probe_frame(motor_ids_[i]);
        send_can_frame(probe.id, probe.data, probe.dlc);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
        auto f = dm::build_disable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
    for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
        auto f = sw::build_disable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
}

bool ArmCanSystem::wait_for_all_feedback(double timeout_sec)
{
    RCLCPP_INFO(logger_, "Polling all motors for their true positions (timeout %.1f s)...",
                timeout_sec);

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(can_tx_mutex_);
            for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
                // 0xF1: reads state WITHOUT entering MIT mode — zero torque.
                auto f = sw::build_read_state_frame(motor_ids_[i]);
                send_can_frame(f.id, f.data, f.dlc);
            }
            for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
                // kp=0/kd=0/tff=0 MIT frame: zero torque, elicits feedback.
                auto f = dm::build_zero_gain_probe_frame(motor_ids_[i]);
                send_can_frame(f.id, f.data, f.dlc);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        bool all = true;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            for (std::size_t i = 0; i < NUM_JOINTS; ++i) all = all && feedback_seen_[i];
        }
        if (all) {
            RCLCPP_INFO(logger_, "All %zu motors reported.", NUM_JOINTS);
            return true;
        }
    }

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        if (!feedback_seen_[i]) {
            RCLCPP_ERROR(logger_, "  motor %u ('%s') never replied",
                         motor_ids_[i], joint_names_[i].c_str());
        }
    }
    return false;
}

// =============================================================================
// RX thread
// =============================================================================

void ArmCanSystem::rx_thread_fn()
{
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    struct can_frame frame{};
    while (rx_running_.load()) {
        const ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            if (!rx_running_.load()) break;
            continue;
        }
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) continue;
        if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) continue;

        const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;

        // ---------- Steadywin: replies arrive on StdID == Dev_addr ----------
        bool is_sw = false;
        std::size_t sw_idx = 0;
        for (std::size_t i = 0; i < NUM_STEADYWIN; ++i) {
            if (raw_id == motor_ids_[i]) { is_sw = true; sw_idx = i; break; }
        }
        if (is_sw) {
            sw::Feedback fb;
            // parse_feedback filters out 0xF0/0xB1/etc. replies whose bytes
            // [1..2] are NOT a position (the old code misread the 0xF0 config
            // echo as ~92 rad and glitched the joint state).
            if (sw::parse_feedback(frame.data, frame.can_dlc, fb)) {
                if (fb.fault) {
                    RCLCPP_WARN_THROTTLE(logger_, steady_clock, 2000,
                        "Steadywin motor %u reports FAULT", motor_ids_[sw_idx]);
                }
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                hw_position_states_[sw_idx] = motor_to_urdf(sw_idx, fb.pos_rad);
                hw_velocity_states_[sw_idx] = fb.vel_rps * joint_directions_[sw_idx];
                feedback_seen_[sw_idx] = true;
            }
            continue;
        }

        // ---------- Damiao: replies arrive on that motor's own Master ID,
        //            0x400 | CAN-ID (these wrists were re-flashed away from
        //            the factory-default shared Master ID 0) ----------------
        std::size_t dm_idx = NUM_JOINTS;
        for (std::size_t i = NUM_STEADYWIN; i < NUM_JOINTS; ++i) {
            if (raw_id == dm::master_id_for(motor_ids_[i])) { dm_idx = i; break; }
        }
        if (dm_idx == NUM_JOINTS) continue;

        dm::Feedback fb;
        if (!dm::parse_feedback(frame.data, frame.can_dlc, fb)) continue;

        // The Master ID already told us who sent this; data[0]'s low nibble is
        // only a sanity check that the motor's own CAN-ID matches what we
        // expect for that slot (catches a mis-flashed / duplicated ID).
        if ((motor_ids_[dm_idx] & 0x0F) != fb.motor_id_nibble) {
            RCLCPP_WARN_THROTTLE(logger_, steady_clock, 5000,
                "Damiao reply on 0x%03X carries CAN-ID nibble %u, expected %u "
                "(motor %u) — check the motor's flashed ID/Master ID.",
                raw_id, fb.motor_id_nibble, motor_ids_[dm_idx] & 0x0F, motor_ids_[dm_idx]);
            continue;
        }

        if (fb.err >= 0x8) {
            RCLCPP_ERROR_THROTTLE(logger_, steady_clock, 2000,
                "Damiao motor %u error: %s (T_mos=%u°C T_rotor=%u°C)",
                motor_ids_[dm_idx], dm::err_to_string(fb.err), fb.t_mos_c, fb.t_rotor_c);
        } else if (fb.err == 0x0 && motors_enabled_.load() && dm_last_err_[dm_idx] == 0x1) {
            RCLCPP_WARN_THROTTLE(logger_, steady_clock, 2000,
                "Damiao motor %u dropped to DISABLED (comm-loss watchdog?)",
                motor_ids_[dm_idx]);
        }

        std::lock_guard<std::mutex> lock(feedback_mutex_);
        dm_last_err_[dm_idx] = fb.err;
        hw_position_states_[dm_idx] = motor_to_urdf(dm_idx, fb.pos_rad);
        hw_velocity_states_[dm_idx] = fb.vel_rps * joint_directions_[dm_idx];
        feedback_seen_[dm_idx] = true;
    }
}

} // namespace arm_hardware_interface