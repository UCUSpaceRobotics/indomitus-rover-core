#include "rover_hardware_interface/rover_hardware_interface.hpp"

#include <chrono>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverHardwareInterface,
    hardware_interface::SystemInterface)

using namespace std::chrono_literals;

namespace rover_hardware_interface {

namespace {

std::string required_param(
    const hardware_interface::HardwareInfo& info,
    const std::string& key)
{
    auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end()) {
        throw std::runtime_error("[RoverHW] Missing required parameter: " + key);
    }
    return it->second;
}

std::string optional_param(
    const hardware_interface::HardwareInfo & info,
    const std::string & key,
    const std::string & default_val)
{
    auto it = info.hardware_parameters.find(key);
    return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

}  // namespace


hardware_interface::CallbackReturn RoverHardwareInterface::on_init(
#ifdef JAZZY_OR_LATER
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    const auto & info = params.hardware_info;
    if (hardware_interface::SystemInterface::on_init(params) !=
#else
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
#endif
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    can_interface_ = optional_param(info, "can_interface", "can0");

    // Motor IDs
    auto parse_ids = [](const std::string& s, std::array<uint8_t, NUM_WHEELS>& out) {
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
        RCLCPP_ERROR(logger_, "[RoverHW] drive_pmax/vmax/tmax must be > 0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ── Joint names from URDF <joint> entries ─────────────────────────────────
    // Expected order: fl_steer, fr_steer, rl_steer, rr_steer,
    //                 fl_drive,  fr_drive,  rl_drive,  rr_drive
    std::size_t controllable = 0;
    for (const auto & j : info.joints) {
        if (!j.command_interfaces.empty()) ++controllable;
    }
    if (controllable != NUM_WHEELS * 2) {
        RCLCPP_ERROR(logger_,
            "[RoverHW] Expected %zu controllable joints, got %zu",
            NUM_WHEELS * 2, controllable);
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        steer_joint_names_[i] = info.joints[i].name;
        drive_joint_names_[i] = info.joints[i + NUM_WHEELS].name;
    }

    steer_pos_.fill(0.0); drive_pos_.fill(0.0); drive_vel_.fill(0.0);
    steer_cmd_.fill(0.0); drive_cmd_.fill(0.0);

    RCLCPP_INFO(logger_,
        "[RoverHW] Initialized  iface=%s  "
        "steer[%d,%d,%d,%d]  drive[%d,%d,%d,%d]",
        can_interface_.c_str(),
        steer_ids_[0], steer_ids_[1], steer_ids_[2], steer_ids_[3],
        drive_ids_[0], drive_ids_[1], drive_ids_[2], drive_ids_[3]);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// export_state_interfaces / export_command_interfaces

std::vector<hardware_interface::StateInterface>
RoverHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    ifaces.reserve(NUM_WHEELS * 3);
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        ifaces.emplace_back(steer_joint_names_[i], hardware_interface::HW_IF_POSITION, &steer_pos_[i]);
        ifaces.emplace_back(drive_joint_names_[i], hardware_interface::HW_IF_POSITION, &drive_pos_[i]);
        ifaces.emplace_back(drive_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &drive_vel_[i]);
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface>
RoverHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    ifaces.reserve(NUM_WHEELS * 2);
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        ifaces.emplace_back(steer_joint_names_[i], hardware_interface::HW_IF_POSITION, &steer_cmd_[i]);
        ifaces.emplace_back(drive_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &drive_cmd_[i]);
    }
    return ifaces;
}

// on_configure — ROS services, status timers.  No CAN socket yet.

hardware_interface::CallbackReturn
RoverHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*prev*/)
{
    hw_node_ = std::make_shared<rclcpp::Node>("rover_hardware_node");
    hw_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    hw_executor_->add_node(hw_node_);
    hw_node_thread_ = std::thread([this]() { hw_executor_->spin(); });

    auto node = hw_node_;

    // ── Status / diagnostics publishers ───────────────────────────────────────
    chassis_status_pub_ = node->create_publisher<indomitus_interfaces::msg::ChassisStatus>(
        "/chassis/motor_states", 10);
    diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);

    // ── Services ───────────────────────────────────────────────────────────────
    motor_enable_srv_ = node->create_service<std_srvs::srv::SetBool>(
        "~/set_motors_enabled",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                std::shared_ptr<std_srvs::srv::SetBool::Response> res) { on_set_motors_enabled(req, res); });

    set_steer_zero_srv_ = node->create_service<indomitus_interfaces::srv::SetSteerZero>(
        "~/set_steer_zero",
        [this](const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request> req,
                std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response> res) { on_set_steer_zero(req, res); });

    // ── 1 Hz status poll (Steadywin 0xAE + 0xA3, Damiao reg 80) ──────────────
    status_poll_timer_ = node->create_wall_timer(1s, [this]() {
        if (!motors_enabled_) return;
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            auto f1 = steadywin_protocol::buildStatusQueryFrame(steer_ids_[i]);
            auto f2 = steadywin_protocol::buildAbsAngleQueryFrame(steer_ids_[i]);
            auto f3 = damiao_protocol::buildReadRegisterFrame(drive_ids_[i], 80);
            send_can_frame(f1.id, f1.data.data(), f1.dlc);
            send_can_frame(f2.id, f2.data.data(), f2.dlc);
            // Damiao register read uses extended frame (0x7FF workaround)
            send_can_frame(f3.id, f3.data.data(), f3.dlc, f3.is_extended);
        }
    });

    // ── 10 Hz chassis status ───────────────────────────────────────────────────
    chassis_status_timer_ = node->create_wall_timer(
        100ms, [this]() { publish_chassis_status(); });

    // ── 1 Hz diagnostics ──────────────────────────────────────────────────────
    diagnostics_timer_ = node->create_wall_timer(
        1s, [this]() { publish_diagnostics(); });

    // ── 10 Hz watchdog — zero motors if write() stops being called ────────────
    last_write_time_ = node->get_clock()->now();
    watchdog_timer_ = node->create_wall_timer(100ms, [this]() {
        if (!motors_enabled_) return;
        const double elapsed =
            (clock_->now() - last_write_time_).seconds();
        if (elapsed < kWatchdogTimeoutSec) return;

        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
            "[RoverHW] write() timeout — zeroing motors");
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            auto f1 = steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f);
            auto f2 = damiao_protocol::buildVelocityFrame(drive_ids_[i], 0.0f);
            send_can_frame(f1.id, f1.data.data(), f1.dlc);
            send_can_frame(f2.id, f2.data.data(), f2.dlc);
        }
    });

    RCLCPP_INFO(logger_, "[RoverHW] Configured.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*prev*/)
{
    if (!open_can_socket()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Start receive thread before enabling motors so we don't miss early feedback
    rx_running_.store(true);
    rx_thread_ = std::thread(&RoverHardwareInterface::rx_thread_fn, this);

    last_write_time_ = clock_->now();
    send_enable_frames();

    RCLCPP_INFO(logger_, "[RoverHW] Activated on %s.", can_interface_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*prev*/)
{
    send_shutdown_frames();

    // Stop rx thread
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }

    close_can_socket();
    RCLCPP_INFO(logger_, "[RoverHW] Deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*prev*/)
{
    if (hw_executor_) hw_executor_->cancel();

    if (hw_node_thread_.joinable()) hw_node_thread_.join();

    chassis_status_pub_.reset();
    diagnostics_pub_.reset();
    motor_enable_srv_.reset();
    set_steer_zero_srv_.reset();
    status_poll_timer_.reset();
    chassis_status_timer_.reset();
    diagnostics_timer_.reset();
    watchdog_timer_.reset();

    RCLCPP_INFO(logger_, "[RoverHW] Cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /*prev*/)
{
    // just in case
    if (motors_enabled_) {
        send_shutdown_frames();
    }

    // just in case
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }

    close_can_socket();

    // just in case
    if (hw_executor_) {
        hw_executor_->cancel();
    }
    if (hw_node_thread_.joinable()) {
        hw_node_thread_.join();
    }

    hw_executor_.reset();
    hw_node_.reset();

    RCLCPP_INFO(logger_, "[RoverHW] Shutdown.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// read — copy latest feedback from steer_state_/drive_state_ → backing arrays

hardware_interface::return_type
RoverHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        steer_pos_[i] = steer_state_[i].pos_valid
            ? static_cast<double>(steer_state_[i].pos_rad) : 0.0;
        
        if (drive_state_[i].valid) {
            drive_pos_[i] = static_cast<double>(drive_state_[i].pos) * kDriveSigns[i];
            drive_vel_[i] = static_cast<double>(drive_state_[i].vel) * kDriveSigns[i];
        }
    }

    return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write — command interfaces → CAN frames, sent directly via socket
//
// Drive sign convention (matches original onWheelTargets):
//   FL(0): negated   FR(1): normal   RL(2): negated   RR(3): normal
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type
RoverHardwareInterface::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    last_write_time_ = clock_->now();

    if (!motors_enabled_) return hardware_interface::return_type::OK;

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto steer_f = steadywin_protocol::buildAbsPositionFrame(
            steer_ids_[i], static_cast<float>(steer_cmd_[i]));

        auto drive_f = damiao_protocol::buildVelocityFrame(
            drive_ids_[i],
            static_cast<float>(drive_cmd_[i]) * kDriveSigns[i]);

        send_can_frame(steer_f.id, steer_f.data.data(), steer_f.dlc);
        send_can_frame(drive_f.id, drive_f.data.data(), drive_f.dlc);
    }

    return hardware_interface::return_type::OK;
}

// SocketCAN — open / close / send

bool RoverHardwareInterface::open_can_socket()
{
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        RCLCPP_ERROR(logger_,
            "[RoverHW] socket(PF_CAN) failed: %s", std::strerror(errno));
        return false;
    }

    // Bind to the named CAN interface
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_,
            "[RoverHW] ioctl SIOCGIFINDEX failed for '%s': %s",
            can_interface_.c_str(), std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_,
            "[RoverHW] bind() failed: %s", std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    // Disable loopback so we don't receive our own TX frames
    int loopback = 0;
    setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback));

    // rx_thread uses blocking recv() — no need to set O_NONBLOCK on the fd
    // write() / send_can_frame() are non-blocking by nature for CAN frames

    RCLCPP_INFO(logger_,
        "[RoverHW] SocketCAN opened: %s (fd=%d)", can_interface_.c_str(), can_fd_);
    return true;
}

void RoverHardwareInterface::close_can_socket()
{
    if (can_fd_ >= 0) {
        close(can_fd_);
        can_fd_ = -1;
        RCLCPP_INFO(logger_, "[RoverHW] SocketCAN closed.");
    }
}

bool RoverHardwareInterface::send_can_frame(
    uint32_t id, const uint8_t * data, uint8_t dlc, bool is_extended)
{
    if (can_fd_ < 0) return false;

    struct can_frame frame{};
    frame.can_id  = is_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    std::lock_guard<std::mutex> lock(can_tx_mutex_);
    const ssize_t nbytes = ::write(can_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
            "[RoverHW] send_can_frame id=0x%X failed: %s", id, std::strerror(errno));
        return false;
    }
    return true;
}

// rx_thread_fn — blocking receive loop
//
// Runs in a dedicated thread so it doesn't block the control loop.
// Writes decoded state into steer_state_/drive_state_ under feedback_mutex_.

void RoverHardwareInterface::rx_thread_fn()
{
    struct can_frame frame{};

    while (rx_running_.load()) {
        const ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));

        if (nbytes < 0) {
            if (errno == EINTR) continue;  // interrupted by signal — retry
            if (!rx_running_.load()) break; // socket closed during shutdown
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                "[RoverHW] CAN recv error: %s", std::strerror(errno));
            continue;
        }

        if (nbytes != static_cast<ssize_t>(sizeof(frame))) continue;

        // Strip flags from CAN ID before dispatch
        const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
        frame.can_id = raw_id;

        std::lock_guard<std::mutex> lock(feedback_mutex_);
        dispatch_can_frame(frame);
    }
}

// dispatch_can_frame — route one frame to the right motor decoder

void RoverHardwareInterface::dispatch_can_frame(const struct can_frame& frame)
{
    // Build an array compatible with protocol functions
    std::array<uint8_t, 8> data{};
    std::memcpy(data.data(), frame.data, std::min<int>(frame.can_dlc, 8));
    const uint8_t dlc = frame.can_dlc;

    // ── Damiao drive: feedback at ESC_ID ──────────────────────────────────────
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (frame.can_id == drive_ids_[i]) {
            damiao_protocol::parseFeedback(
                data, dlc, drive_ids_[i],
                drive_pmax_, drive_vmax_, drive_tmax_, drive_state_[i]);
            damiao_protocol::parseRegisterResponse(
                data, dlc, drive_ids_[i], drive_state_[i]);
            return;
        }
    }

    // Damiao drive: broadcast feedback at MST_ID
    if (frame.can_id == mst_id_) {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
            if (damiao_protocol::parseFeedback(
                    data, dlc, drive_ids_[i],
                    drive_pmax_, drive_vmax_, drive_tmax_, drive_state_[i]))
            {
                break;
            }
        }
        if (dlc >= 8 && data[2] == 0x33) {
            for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
                if (damiao_protocol::parseRegisterResponse(
                        data, dlc, drive_ids_[i], drive_state_[i]))
                {
                    break;
                }
            }
        }
        return;
    }

    // Steadywin steer: response at esc_id or 0x100|esc_id
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (frame.can_id == steer_ids_[i] ||
            frame.can_id == (0x100u | steer_ids_[i]))
        {
            steadywin_protocol::parseResponse(data, dlc, steer_state_[i]);
            return;
        }
    }
}

// Motor lifecycle

void RoverHardwareInterface::send_enable_frames()
{
    RCLCPP_INFO(logger_, "[RoverHW] Enabling all motors");

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = steadywin_protocol::buildClearFaultFrame(steer_ids_[i]);
        send_can_frame(f.id, f.data.data(), f.dlc);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f);
        send_can_frame(f.id, f.data.data(), f.dlc);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildWriteRegisterUint32Frame(drive_ids_[i], 9, 200u);
        send_can_frame(f.id, f.data.data(), f.dlc, f.is_extended);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildSetModeFrame(drive_ids_[i], 3);
        send_can_frame(f.id, f.data.data(), f.dlc, f.is_extended);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildEnableFrame(drive_ids_[i]);
        send_can_frame(f.id, f.data.data(), f.dlc);
    }

    motors_enabled_ = true;
    RCLCPP_INFO(logger_, "[RoverHW] All motors enabled");
}

void RoverHardwareInterface::send_disable_frames()
{
    RCLCPP_INFO(logger_, "[RoverHW] Disabling all motors");

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = steadywin_protocol::buildDisableFrame(steer_ids_[i]);
        send_can_frame(f.id, f.data.data(), f.dlc);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildDisableFrame(drive_ids_[i]);
        send_can_frame(f.id, f.data.data(), f.dlc);
    }

    motors_enabled_ = false;
    RCLCPP_INFO(logger_, "[RoverHW] All motors disabled");
}

void RoverHardwareInterface::send_shutdown_frames()
{
    if (!motors_enabled_) {
        send_disable_frames();
        return;
    }

    send_disable_frames();
}


void RoverHardwareInterface::on_set_motors_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
    std::shared_ptr<std_srvs::srv::SetBool::Response>       res)
{
    req->data ? send_enable_frames() : send_disable_frames();
    res->success = true;
    res->message = req->data ? "All chassis motors enabled" : "All chassis motors disabled";
}

void RoverHardwareInterface::on_set_steer_zero(
    const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request>  req,
    std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response>       res)
{
    static constexpr std::array<const char *, NUM_WHEELS> kNames = {"FL","FR","RL","RR"};
    const bool zero_all = req->motor_ids.empty();
    std::string zeroed, unknown;

    auto zero_one = [&](std::size_t i) {
        auto f = steadywin_protocol::buildSetOriginFrame(steer_ids_[i]);
        send_can_frame(f.id, f.data.data(), f.dlc);
        zeroed += kNames[i]; zeroed += '(';
        zeroed += std::to_string(steer_ids_[i]); zeroed += ") ";
        RCLCPP_INFO(logger_, "[RoverHW] Set steer zero: %s (id=%d)",
            kNames[i], steer_ids_[i]);
    };

    if (zero_all) {
        for (std::size_t i = 0; i < NUM_WHEELS; ++i) zero_one(i);
    } else {
        for (const uint8_t req_id : req->motor_ids) {
            bool found = false;
            for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
                if (steer_ids_[i] == req_id) { zero_one(i); found = true; break; }
            }
            if (!found) { unknown += std::to_string(req_id); unknown += ' '; }
        }
    }

    if (!unknown.empty()) {
        res->success = false;
        res->message = "Unknown steer IDs: " + unknown + "— valid: " +
            std::to_string(steer_ids_[0]) + " " + std::to_string(steer_ids_[1]) + " " +
            std::to_string(steer_ids_[2]) + " " + std::to_string(steer_ids_[3]);
        return;
    }

    res->success = true;
    res->message = "Origin set for: " + zeroed;
}

// Diagnostics / status

void RoverHardwareInterface::publish_chassis_status()
{
    indomitus_interfaces::msg::ChassisStatus msg;
    msg.header.stamp = clock_->now();

    std::lock_guard<std::mutex> lock(feedback_mutex_);

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state_[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id = steer_ids_[i]; m.motor_type = "steadywin";
        m.joint_name = steer_joint_names_[i];
        m.position = s.pos_valid ? s.pos_rad : 0.0f;
        m.kinematic_valid = s.pos_valid;
        m.voltage = s.voltage; m.current = s.bus_current;
        m.temperature = static_cast<float>(s.temperature);
        m.mode = s.mode; m.fault_code = s.fault_code;
        m.health_valid = s.diag_valid; m.enabled = motors_enabled_;
        msg.motors.push_back(m);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = drive_state_[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id = drive_ids_[i]; m.motor_type = "damiao";
        m.joint_name = drive_joint_names_[i];
        m.position = s.valid ? s.pos : 0.0f;
        m.velocity = s.valid ? s.vel : 0.0f;
        m.torque   = s.valid ? s.tor : 0.0f;
        m.kinematic_valid = s.valid;
        m.temperature = static_cast<float>(s.t_mos);
        m.mode = s.valid ? 3u : 0u;
        m.fault_code = (s.valid && s.err != 0x1) ? 0x01u : 0x00u;
        m.health_valid = s.valid;
        m.enabled = motors_enabled_ && s.valid && s.err == 0x1;
        msg.motors.push_back(m);
    }
    chassis_status_pub_->publish(msg);
}

void RoverHardwareInterface::publish_diagnostics()
{
    static constexpr std::array<const char *, NUM_WHEELS> kNames = {"FL","FR","RL","RR"};
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = clock_->now();

    auto kv = [](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; return p;
    };

    std::lock_guard<std::mutex> lock(feedback_mutex_);

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state_[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = st.hardware_id = std::string("steadywin/steer_") + kNames[i];
        if (!s.diag_valid) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "No status received";
        } else if (s.fault_code != 0) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            std::string f;
            if (s.fault_code & 0x01) f += "voltage ";
            if (s.fault_code & 0x02) f += "current ";
            if (s.fault_code & 0x04) f += "temperature ";
            if (s.fault_code & 0x08) f += "encoder ";
            if (s.fault_code & 0x40) f += "hardware ";
            if (s.fault_code & 0x80) f += "software ";
            st.message = "FAULT: " + f;
        } else {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
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
        st.name = st.hardware_id = std::string("damiao/drive_") + kNames[i];
        if (!s.valid) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; st.message = "No feedback";
        } else if (s.err == 0x1) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;   st.message = "Enabled";
        } else {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; st.message = "Disabled or fault";
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
