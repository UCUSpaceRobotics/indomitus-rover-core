#include "rover_hardware_interface/rover_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

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

    steer_feedback_timeout_ = std::stod(optional_param(info, "steer_feedback_timeout", "3.0"));
    drive_feedback_timeout_ = std::stod(optional_param(info, "drive_feedback_timeout", "0.5"));

    if (steer_feedback_timeout_ <= 0.0 || drive_feedback_timeout_ <= 0.0) {
        RCLCPP_ERROR(logger_, "[RoverHW] feedback timeouts must be > 0");
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

    // Transient-local so a logger or operator GUI that starts late — or
    // reconnects after a comms drop — immediately receives recent fault
    // history instead of nothing. Events alone have no state for a late
    // joiner to recover from; this buys some of it back.
    {
        rclcpp::QoS qos(rclcpp::KeepLast(20));
        qos.reliable().transient_local();
        fault_event_pub_ =
            node->create_publisher<indomitus_interfaces::msg::FaultEvent>("/fault_events", qos);
    }

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
            can_bus_.send(f1.id, f1.data.data(), f1.dlc);
            can_bus_.send(f2.id, f2.data.data(), f2.dlc);
            // Damiao register read uses extended frame (0x7FF workaround)
            can_bus_.send(f3.id, f3.data.data(), f3.dlc, f3.is_extended);
        }
    });

    // ── 10 Hz chassis status ───────────────────────────────────────────────────
    chassis_status_timer_ = node->create_wall_timer(
        100ms, [this]() { publish_chassis_status(); });

    // ── 1 Hz diagnostics ──────────────────────────────────────────────────────
    diagnostics_timer_ = node->create_wall_timer(
        1s, [this]() { publish_diagnostics(); });

    // ── 20 Hz fault event drain ───────────────────────────────────────────────
    fault_event_timer_ = node->create_wall_timer(
        50ms, [this]() { publish_fault_events(); });

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
            can_bus_.send(f1.id, f1.data.data(), f1.dlc);
            can_bus_.send(f2.id, f2.data.data(), f2.dlc);
        }
    });

    RCLCPP_INFO(logger_, "[RoverHW] Configured.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*prev*/)
{
    if (!can_bus_.open(can_interface_)) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Nothing has been heard from any motor yet on this activation. Clearing
    // the arrival times (rather than carrying them over from a previous
    // activation) is what makes a repeated activate/deactivate cycle behave
    // like the first one.
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        steer_status_rx_ns_.fill(0);
        drive_feedback_rx_ns_.fill(0);
    }
    feedback_expected_since_ns_.store(steady_now_ns());

    // Start receive thread before enabling motors so we don't miss early feedback
    const bool rx_started = can_bus_.start_rx([this](const struct can_frame & frame) {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        dispatch_can_frame(frame);
    });
    if (!rx_started) {
        // Enabling motors we could never hear back from would leave the rover
        // driving with no fault detection at all.
        RCLCPP_ERROR(logger_, "[RoverHW] CAN receive thread failed to start");
        can_bus_.close();
        return hardware_interface::CallbackReturn::ERROR;
    }

    last_write_time_ = clock_->now();
    send_enable_frames();

    activated_ = true;
    RCLCPP_INFO(logger_, "[RoverHW] Activated on %s.", can_interface_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*prev*/)
{
    activated_ = false;
    send_shutdown_frames();

    // close() stops the RX thread first; the descriptor must outlive its reader.
    can_bus_.close();
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
    fault_event_timer_.reset();
    fault_event_pub_.reset();
    watchdog_timer_.reset();

    RCLCPP_INFO(logger_, "[RoverHW] Cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn
RoverHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /*prev*/)
{
    activated_ = false;

    // just in case
    if (motors_enabled_) {
        send_shutdown_frames();
    }

    // just in case
    can_bus_.close();

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
    const int64_t now_ns = steady_now_ns();
    const bool enabled = motors_enabled_.load();

    // Nothing polls the motors while they are disabled, so the timeout clock
    // only runs while we are actually asking them to answer.
    if (!enabled) feedback_expected_since_ns_.store(now_ns);

    // Snapshot, then release: everything below is arithmetic on a private copy.
    // Holding feedback_mutex_ across detection would make the RX thread wait on
    // the control loop for no reason, and the two contend eight times a cycle.
    std::array<steadywin_protocol::MotorState, NUM_WHEELS> steer;
    std::array<damiao_protocol::MotorState, NUM_WHEELS> drive;
    FeedbackFreshness fresh;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        steer = steer_state_;
        drive = drive_state_;
        fresh = feedback_freshness(now_ns);
    }

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        steer_pos_[i] = steer[i].pos_valid
            ? static_cast<double>(steer[i].pos_rad) : 0.0;

        if (drive[i].valid) {
            drive_pos_[i] = static_cast<double>(drive[i].pos) * kDriveSigns[i];
            drive_vel_[i] = static_cast<double>(drive[i].vel) * kDriveSigns[i];
        }
    }

    // Fault detection runs here rather than in the RX thread: this is the only
    // place with a consistent snapshot of all motors at a fixed rate, and a
    // motor that has gone silent produces no frames to trigger the RX path.
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        fault_detector_.detect_steer_fault(
            i, steer[i], fresh.steer[i], steer_joint_names_[i], steer_ids_[i], enabled);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        fault_detector_.detect_drive_fault(
            i, drive[i], fresh.drive[i], drive_joint_names_[i], drive_ids_[i], enabled);
    }

    fault_detector_.detect_bus_fault(can_bus_.state(), can_interface_, enabled);

    // A single motor fault must not tear down the hardware component: returning
    // ERROR here would deactivate all eight motors and the telemetry with them,
    // exactly when the operator needs both. ERROR is reserved for the CAN
    // socket itself being gone.
    //
    // Note bus-off is deliberately NOT an ERROR: the kernel recovers it on its
    // own when the interface has restart-ms set, and the motors have their own
    // failsafes. Tearing down the component on a transient bus-off would turn a
    // self-healing fault into one that needs an operator to re-activate.
    //
    // Gated on activated_: controller_manager calls read() as soon as the
    // component is merely configured (inactive), before on_activate() has
    // opened the socket. Without this gate the component would fault itself
    // out on its very first read cycle and never get a chance to activate.
    if (activated_ && !can_bus_.is_open()) {
        return hardware_interface::return_type::ERROR;
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

    std::size_t failed = 0;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto steer_f = steadywin_protocol::buildAbsPositionFrame(
            steer_ids_[i], static_cast<float>(steer_cmd_[i]));

        auto drive_f = damiao_protocol::buildVelocityFrame(
            drive_ids_[i],
            static_cast<float>(drive_cmd_[i]) * kDriveSigns[i]);

        if (can_bus_.send(steer_f.id, steer_f.data.data(), steer_f.dlc)
                != CanBus::SendResult::OK) ++failed;
        if (can_bus_.send(drive_f.id, drive_f.data.data(), drive_f.dlc)
                != CanBus::SendResult::OK) ++failed;
    }

    // A whole cycle failing to reach any motor is the signature that matters:
    // the Damiao TIMEOUT register is 200 ms, so twenty consecutive failed
    // cycles at 100 Hz is enough for every drive motor to fault itself out.
    // Reporting the cycle count makes that visible before the motors do it.
    if (failed == NUM_WHEELS * 2) {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
            "[RoverHW] no CAN frames reached the bus this cycle");
    }

    return hardware_interface::return_type::OK;
}

// Feedback freshness

int64_t RoverHardwareInterface::steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

RoverHardwareInterface::FeedbackFreshness
RoverHardwareInterface::feedback_freshness(int64_t now_ns) const
{
    // Measure the gap from the later of "last frame from this motor" and "the
    // moment we started expecting frames at all". The second term is what
    // stops a motor from being declared silent for the period it spent
    // legitimately disabled.
    const int64_t expected_since = feedback_expected_since_ns_.load();

    const auto fresh = [&](int64_t last_rx_ns, bool ever_received, double timeout_s) {
        if (!ever_received) return false;
        const int64_t reference = std::max(last_rx_ns, expected_since);
        return (now_ns - reference) <= static_cast<int64_t>(timeout_s * 1e9);
    };

    FeedbackFreshness out;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        out.steer[i] = fresh(
            steer_status_rx_ns_[i], steer_state_[i].diag_valid, steer_feedback_timeout_);
        out.drive[i] = fresh(
            drive_feedback_rx_ns_[i], drive_state_[i].valid, drive_feedback_timeout_);
    }
    return out;
}

// dispatch_can_frame — route one frame to the right motor decoder
//
// Called from CanBus's RX thread via the callback given to start_rx().
// Caller (on_activate's lambda) holds feedback_mutex_.

void RoverHardwareInterface::dispatch_can_frame(const struct can_frame& frame)
{
    // Build an array compatible with protocol functions
    std::array<uint8_t, 8> data{};
    std::memcpy(data.data(), frame.data, std::min<int>(frame.can_dlc, 8));
    const uint8_t dlc = frame.can_dlc;

    const int64_t now_ns = steady_now_ns();

    // ── Damiao drive: feedback at ESC_ID ──────────────────────────────────────
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        if (frame.can_id == drive_ids_[i]) {
            if (damiao_protocol::parseFeedback(
                    data, dlc, drive_ids_[i],
                    drive_pmax_, drive_vmax_, drive_tmax_, drive_state_[i]))
            {
                drive_feedback_rx_ns_[i] = now_ns;
            }
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
                drive_feedback_rx_ns_[i] = now_ns;
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
            // Only the 0xAE status reply refreshes health. A position reply
            // proves the motor is alive but carries no fault register, so
            // timing health off it would mask a motor that answers moves while
            // its diagnostics have gone stale.
            if (steadywin_protocol::parseStatusResponse(data, dlc, steer_state_[i])) {
                steer_status_rx_ns_[i] = now_ns;
            }
            steadywin_protocol::parsePositionResponse(data, dlc, steer_state_[i]);
            steadywin_protocol::parseRealtimeResponse(data, dlc, steer_state_[i]);
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
        can_bus_.send(f.id, f.data.data(), f.dlc);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f);
        can_bus_.send(f.id, f.data.data(), f.dlc);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildWriteRegisterUint32Frame(drive_ids_[i], 9, 200u);
        can_bus_.send(f.id, f.data.data(), f.dlc, f.is_extended);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildSetModeFrame(drive_ids_[i], 3);
        can_bus_.send(f.id, f.data.data(), f.dlc, f.is_extended);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildEnableFrame(drive_ids_[i]);
        can_bus_.send(f.id, f.data.data(), f.dlc);
    }

    // Start the feedback-timeout clock from here, not from whenever read() last
    // ran: enabling over the service while the component is idle would
    // otherwise look like every motor had been silent since boot.
    feedback_expected_since_ns_.store(steady_now_ns());
    motors_enabled_ = true;
    RCLCPP_INFO(logger_, "[RoverHW] All motors enabled");
}

void RoverHardwareInterface::send_disable_frames()
{
    RCLCPP_INFO(logger_, "[RoverHW] Disabling all motors");

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = steadywin_protocol::buildDisableFrame(steer_ids_[i]);
        can_bus_.send(f.id, f.data.data(), f.dlc);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        auto f = damiao_protocol::buildDisableFrame(drive_ids_[i]);
        can_bus_.send(f.id, f.data.data(), f.dlc);
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
        can_bus_.send(f.id, f.data.data(), f.dlc);
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

    // Copy out, then build. Composing the message under the lock would block
    // both the RX thread and read() for the length of a dozen string
    // allocations, ten times a second.
    std::array<steadywin_protocol::MotorState, NUM_WHEELS> steer_state;
    std::array<damiao_protocol::MotorState, NUM_WHEELS> drive_state;
    FeedbackFreshness fresh;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        steer_state = steer_state_;
        drive_state = drive_state_;
        fresh = feedback_freshness(steady_now_ns());
    }

    const bool enabled = motors_enabled_.load();

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id = steer_ids_[i]; m.motor_type = "steadywin";
        m.joint_name = steer_joint_names_[i];
        m.position = s.pos_valid ? s.pos_rad : kNaN;
        m.velocity = kNaN;
        m.torque   = kNaN;
        m.kinematic_valid = s.pos_valid;
        m.voltage = s.voltage; m.current = s.bus_current;
        m.temperature = static_cast<float>(s.temperature);
        m.mode = s.mode; m.fault_code = s.fault_code;
        // Freshness, not just "a frame arrived once": the vendor flag latches
        // true forever, so on its own it would report a motor that fell off
        // the bus an hour ago as healthy.
        m.health_valid = fresh.steer[i]; m.enabled = enabled;
        msg.motors.push_back(m);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = drive_state[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id = drive_ids_[i]; m.motor_type = "damiao";
        m.joint_name = drive_joint_names_[i];
        m.position = s.valid ? s.pos : kNaN;
        m.velocity = s.valid ? s.vel : kNaN;
        m.torque   = s.valid ? s.tor : kNaN;
        m.kinematic_valid = s.valid;
        m.voltage = kNaN;
        m.current = kNaN;
        m.temperature = static_cast<float>(s.t_mos);
        // Gate on the ERR nibble, not just feedback presence: a motor that has
        // faulted out is no longer running in mode 3, and reporting otherwise
        // makes a dead motor look commanded.
        m.mode = (s.valid && s.err == damiao_protocol::ERR_ENABLED) ? 3u : 0u;
        m.fault_code = (s.valid && s.err > damiao_protocol::ERR_ENABLED) ? s.err : 0x00u;
        m.health_valid = fresh.drive[i];
        m.enabled = enabled && fresh.drive[i] && s.err == damiao_protocol::ERR_ENABLED;
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

    // Snapshot under the lock; the message below is dozens of string
    // allocations that have no business blocking the RX thread.
    std::array<steadywin_protocol::MotorState, NUM_WHEELS> steer_state;
    std::array<damiao_protocol::MotorState, NUM_WHEELS> drive_state;
    FeedbackFreshness fresh;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        steer_state = steer_state_;
        drive_state = drive_state_;
        fresh = feedback_freshness(steady_now_ns());
    }

    // ── CAN transport ─────────────────────────────────────────────────────────
    // First entry deliberately: if the bus is down, every motor entry below is
    // stale by definition, and this says so.
    {
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = st.hardware_id = "can/" + can_interface_;
        const auto state = can_bus_.state();
        switch (state) {
            case CanBus::BusState::BUS_OFF:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                st.message = "BUS-OFF — no frames reach any motor";
                break;
            case CanBus::BusState::ERROR_PASSIVE:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                st.message = "Error-passive — controller degraded";
                break;
            case CanBus::BusState::ERROR_WARNING:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                st.message = "Error counters elevated";
                break;
            case CanBus::BusState::OK:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                st.message = "OK";
                break;
        }
        st.values.push_back(kv("tx_error_count", std::to_string(can_bus_.tx_error_count())));
        st.values.push_back(kv("rx_error_count", std::to_string(can_bus_.rx_error_count())));
        // Cumulative, not a rate: a value that climbs during a manoeuvre is the
        // evidence that the bus is saturating.
        st.values.push_back(kv("tx_dropped",     std::to_string(can_bus_.tx_dropped())));
        st.values.push_back(kv("rx_overflow",    std::to_string(can_bus_.rx_overflow())));
        // Non-zero means fault events were discarded before anyone published
        // them, which makes every count below it suspect.
        st.values.push_back(kv("fault_events_dropped",
            std::to_string(fault_detector_.dropped_events())));
        arr.status.push_back(st);
    }

    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = st.hardware_id = std::string("steadywin/steer_") + kNames[i];
        if (!fresh.steer[i]) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = s.diag_valid ? "Status stale — motor stopped answering"
                                      : "No status received";
        } else if (s.fault_code != 0) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            std::string f;
            if (s.fault_code & steadywin_protocol::FAULT_VOLTAGE)     f += "voltage ";
            if (s.fault_code & steadywin_protocol::FAULT_CURRENT)     f += "current ";
            if (s.fault_code & steadywin_protocol::FAULT_TEMPERATURE) f += "temperature ";
            if (s.fault_code & steadywin_protocol::FAULT_ENCODER)     f += "encoder ";
            if (s.fault_code & steadywin_protocol::FAULT_HARDWARE)    f += "hardware ";
            if (s.fault_code & steadywin_protocol::FAULT_SOFTWARE)    f += "software ";
            st.message = "FAULT: " + f;
        } else if (s.mode == steadywin_protocol::MODE_OFF) {
            // This motor sets no fault bit when it silently stops accepting
            // commands — it just drops to mode 0. Checking fault_code alone
            // would report it as OK.
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "Not enabled (mode 0)";
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
        const auto & s = drive_state[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = st.hardware_id = std::string("damiao/drive_") + kNames[i];
        const auto fault = damiao_protocol::translateFault(s.err);
        if (!fresh.drive[i]) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = s.valid ? "Feedback stale — motor stopped answering"
                                 : "No feedback";
        } else if (s.err == damiao_protocol::ERR_ENABLED) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;   st.message = "Enabled";
        } else if (s.err == damiao_protocol::ERR_DISABLED) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; st.message = "Disabled";
        } else {
            // A real fault, named. "Disabled or fault" told us nothing during
            // the field test — which of the seven codes it was is the whole
            // question when a drive motor goes red mid-run.
            st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            st.message = std::string("FAULT: ") + rover_fault::to_string(fault);
        }
        st.values.push_back(kv("fault",      rover_fault::to_string(fault)));
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

// publish_fault_events — drain the queue filled by read()
//
// Runs on hw_node_'s executor, not the control loop. Faults are rare, so this
// finds an empty queue on essentially every tick; the benefit of draining via
// FaultDetector's own internal lock (rather than feedback_mutex_) is that
// rclcpp publishing (which allocates) never runs inside read().

void RoverHardwareInterface::publish_fault_events()
{
    if (!fault_event_pub_) return;

    auto events = fault_detector_.drain_events();
    if (events.empty()) return;

    for (const auto & ev : events) {
        fault_event_pub_->publish(ev);
        if (ev.event == indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER) {
            RCLCPP_ERROR(logger_, "[RoverHW] %s FAULT: %s (raw 0x%02X)",
                ev.component.c_str(), ev.fault_name.c_str(), ev.raw_code);
        } else if (ev.event == indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR) {
            RCLCPP_INFO(logger_, "[RoverHW] %s fault cleared", ev.component.c_str());
        }
    }
}

}  // namespace rover_hardware_interface
