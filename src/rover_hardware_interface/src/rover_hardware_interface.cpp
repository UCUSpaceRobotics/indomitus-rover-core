#include "rover_hardware_interface/rover_hardware_interface.hpp"

#include <chrono>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <limits>

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/error.h>   // CAN_ERR_* classes and CAN_ERR_CRTL_* bits
#include <linux/can/raw.h>

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
    fault_event_timer_.reset();
    fault_event_pub_.reset();
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

    // Fault detection runs here rather than in the RX thread: this is the only
    // place with a consistent snapshot of all motors at a fixed rate, and a
    // motor that has gone silent produces no frames to trigger the RX path.
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & s = steer_state_[i];
        detect_fault_transition(
            i, /*is_steer=*/true,
            steadywin_protocol::translateFault(s.fault_code, s.mode),
            s.diag_valid, s.fault_code);
    }
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & d = drive_state_[i];
        detect_fault_transition(
            i, /*is_steer=*/false,
            damiao_protocol::translateFault(d.err),
            d.valid, d.err);
    }

    detect_can_bus_fault();

    // A single motor fault must not tear down the hardware component: returning
    // ERROR here would deactivate all eight motors and the telemetry with them,
    // exactly when the operator needs both. ERROR is reserved for the CAN
    // socket itself being gone.
    //
    // Note bus-off is deliberately NOT an ERROR: the kernel recovers it on its
    // own when the interface has restart-ms set, and the motors have their own
    // failsafes. Tearing down the component on a transient bus-off would turn a
    // self-healing fault into one that needs an operator to re-activate.
    if (can_fd_ < 0) {
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

// detect_can_bus_fault — edge-trigger the transport
//
// Caller must hold feedback_mutex_ (queues onto pending_fault_events_).

void RoverHardwareInterface::detect_can_bus_fault()
{
    rover_fault::Fault current = rover_fault::Fault::NONE;
    switch (bus_state_.load()) {
        case CanBusState::BUS_OFF:        current = rover_fault::Fault::CAN_BUS_OFF;       break;
        case CanBusState::ERROR_PASSIVE:  current = rover_fault::Fault::CAN_ERROR_PASSIVE; break;
        // Error-warning is a counter threshold, not a loss of function: the
        // controller is still transmitting normally. Reporting it as a fault
        // would cry wolf on a bus that is merely busy.
        case CanBusState::ERROR_WARNING:
        case CanBusState::OK:             current = rover_fault::Fault::NONE;              break;
    }

    if (bus_fault_.seen && current == bus_fault_.fault) return;

    if (bus_fault_.seen) {
        indomitus_interfaces::msg::FaultEvent ev;
        ev.header.stamp = clock_->now();
        ev.component = "chassis/" + can_interface_;
        ev.vendor    = "socketcan";
        ev.esc_id    = 0;
        ev.raw_code  = static_cast<uint8_t>(bus_state_.load());
        ev.event = !rover_fault::is_fault(bus_fault_.fault)
            ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER
            : (!rover_fault::is_fault(current)
                   ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR
                   : indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CHANGE);
        ev.fault          = static_cast<uint8_t>(current);
        ev.previous_fault = static_cast<uint8_t>(bus_fault_.fault);
        ev.fault_name     = rover_fault::to_string(current);
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(current));
        // The bus has no kinematics; the counters are what explain it, and they
        // ride in the diagnostics entry alongside.
        ev.position = ev.velocity = ev.torque = kNaN;
        ev.temperature = ev.voltage = ev.current = kNaN;
        ev.mode    = static_cast<uint8_t>(bus_state_.load());
        ev.enabled = motors_enabled_;
        pending_fault_events_.push_back(ev);
    }

    bus_fault_.fault = current;
    bus_fault_.seen  = true;
}

// detect_fault_transition — edge-trigger one motor, queue an event if changed
//
// Caller must hold feedback_mutex_.

void RoverHardwareInterface::detect_fault_transition(
    std::size_t index, bool is_steer,
    rover_fault::Fault current, bool health_valid, uint8_t raw_code)
{
    static constexpr std::array<const char *, NUM_WHEELS> kNames = {"FL","FR","RL","RR"};

    auto & tracker = is_steer ? steer_fault_[index] : drive_fault_[index];

    indomitus_interfaces::msg::FaultEvent ev;
    ev.header.stamp = clock_->now();
    ev.component = std::string(is_steer ? "chassis/steer_" : "chassis/drive_") + kNames[index];
    ev.joint_name = is_steer ? steer_joint_names_[index] : drive_joint_names_[index];
    ev.vendor     = is_steer ? "steadywin" : "damiao";
    ev.esc_id     = is_steer ? steer_ids_[index] : drive_ids_[index];
    ev.raw_code   = raw_code;

    if (is_steer) {
        const auto & s = steer_state_[index];
        ev.position    = s.pos_valid ? s.pos_rad : kNaN;
        ev.velocity    = kNaN;   // not reported by this vendor
        ev.torque      = kNaN;
        ev.temperature = static_cast<float>(s.temperature);
        ev.voltage     = s.voltage;
        ev.current     = s.bus_current;
        ev.mode        = s.mode;
    } else {
        const auto & d = drive_state_[index];
        ev.position    = d.valid ? d.pos : kNaN;
        ev.velocity    = d.valid ? d.vel : kNaN;
        ev.torque      = d.valid ? d.tor : kNaN;
        ev.temperature = static_cast<float>(d.t_mos);
        ev.voltage     = kNaN;   // not reported by this vendor
        ev.current     = kNaN;
        ev.mode        = (d.valid && d.err == damiao_protocol::ERR_ENABLED) ? 3u : 0u;
    }
    ev.enabled = motors_enabled_;

    // Feedback presence is tracked separately from faults: a motor that has
    // dropped off the bus reports no fault code at all, so without this it
    // would be indistinguishable from a healthy one.
    if (tracker.seen && health_valid != tracker.health_valid) {
        ev.event = health_valid
            ? indomitus_interfaces::msg::FaultEvent::EVENT_SIGNAL_OK
            : indomitus_interfaces::msg::FaultEvent::EVENT_SIGNAL_LOST;
        ev.fault          = static_cast<uint8_t>(tracker.fault);
        ev.previous_fault = static_cast<uint8_t>(tracker.fault);
        ev.fault_name     = rover_fault::to_string(tracker.fault);
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(tracker.fault));
        pending_fault_events_.push_back(ev);
    }
    tracker.health_valid = health_valid;

    if (!health_valid) {
        // Fault codes are meaningless without feedback. Hold the last known
        // fault so recovery is detected correctly once frames resume.
        tracker.seen = true;
        return;
    }

    if (tracker.seen && current != tracker.fault) {
        const bool was_fault = rover_fault::is_fault(tracker.fault);
        const bool is_now    = rover_fault::is_fault(current);

        // NONE <-> NOT_ENABLED is an ordinary enable/disable, not a fault edge.
        if (was_fault || is_now) {
            ev.event = !was_fault
                ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER
                : (!is_now ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR
                           : indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CHANGE);
            ev.fault          = static_cast<uint8_t>(current);
            ev.previous_fault = static_cast<uint8_t>(tracker.fault);
            ev.fault_name     = rover_fault::to_string(current);
            ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(current));
            pending_fault_events_.push_back(ev);
        }
    }

    tracker.fault = current;
    tracker.seen  = true;

    // Bound the queue. If the executor thread ever stalls while a fault is
    // flapping, read() would otherwise grow this without limit at 100 Hz.
    // Dropping the oldest keeps the most recent picture, which is what an
    // operator needs; the count of drops is not worth tracking here because
    // reaching this at all means something upstream is already broken.
    if (pending_fault_events_.size() > kMaxPendingFaultEvents) {
        pending_fault_events_.erase(pending_fault_events_.begin());
    }
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

        if (send_can_frame(steer_f.id, steer_f.data.data(), steer_f.dlc)
                != CanSendResult::OK) ++failed;
        if (send_can_frame(drive_f.id, drive_f.data.data(), drive_f.dlc)
                != CanSendResult::OK) ++failed;
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

// SocketCAN — open / close / send

void RoverHardwareInterface::on_can_error(const struct can_frame & frame)
{
    // frame.data[1] holds controller-problem bits when CAN_ERR_CRTL is set
    if (frame.can_id & CAN_ERR_BUSOFF) {
        bus_state_.store(CanBusState::BUS_OFF);
        RCLCPP_ERROR(logger_, "[RoverHW] CAN bus-off detected on %s", can_interface_.c_str());
    } else if (frame.can_id & CAN_ERR_CRTL) {
        if (frame.data[1] & CAN_ERR_CRTL_TX_PASSIVE ||
            frame.data[1] & CAN_ERR_CRTL_RX_PASSIVE) {
            bus_state_.store(CanBusState::ERROR_PASSIVE);
            RCLCPP_WARN(logger_, "[RoverHW] CAN controller error-passive");
        } else if (frame.data[1] & CAN_ERR_CRTL_TX_WARNING ||
                   frame.data[1] & CAN_ERR_CRTL_RX_WARNING) {
            bus_state_.store(CanBusState::ERROR_WARNING);
        }
        tx_error_count_.store(frame.data[6]);
        rx_error_count_.store(frame.data[7]);
    } else if (frame.can_id & CAN_ERR_RESTARTED) {
        bus_state_.store(CanBusState::OK);
        RCLCPP_INFO(logger_, "[RoverHW] CAN interface auto-restarted, bus recovered");
    }
}

bool RoverHardwareInterface::open_can_socket()
{
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        RCLCPP_ERROR(logger_,
            "[RoverHW] socket(PF_CAN) failed: %s", std::strerror(errno));
        return false;
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL |
        CAN_ERR_PROT | CAN_ERR_TRX | CAN_ERR_ACK | CAN_ERR_BUSOFF  |
        CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
    setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

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

// Return type must be qualified: a leading return type on an out-of-class
// member definition is not looked up in class scope, and CanSendResult is a
// nested type.
RoverHardwareInterface::CanSendResult RoverHardwareInterface::send_can_frame(
    uint32_t id, const uint8_t * data, uint8_t dlc, bool is_extended)
{
    // Socket not open — same practical meaning to a caller as the bus being
    // gone: there is no transport.
    if (can_fd_ < 0) return CanSendResult::BUS_DOWN;

    struct can_frame frame{};
    frame.can_id  = is_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    std::lock_guard<std::mutex> lock(can_tx_mutex_);
    const ssize_t nbytes = ::write(can_fd_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
        return CanSendResult::OK;
    }

    if (errno == ENOBUFS || errno == EAGAIN) {
        // TX queue full. Transient, but not nothing: a sustained burst of these
        // means frames are being dropped, and 200 ms of dropped frames is
        // exactly what trips the Damiao TIMEOUT register into a comm-loss
        // fault. Silence here would hide the cause of the fault we then report.
        tx_dropped_.fetch_add(1);
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
            "[RoverHW] CAN TX queue full (id=0x%X), frame dropped — %d total",
            id, tx_dropped_.load());
        return CanSendResult::WOULD_BLOCK;
    }
    if (errno == ENETDOWN) {
        bus_state_.store(CanBusState::BUS_OFF);
        return CanSendResult::BUS_DOWN;
    }
    
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "[RoverHW] send_can_frame id=0x%X failed: %s", id, std::strerror(errno));
    return CanSendResult::ERROR;
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

        if (frame.can_id & CAN_ERR_FLAG) {
            on_can_error(frame);
            continue;  // don't pass error frames to dispatch_can_frame
        }

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
        m.position = s.pos_valid ? s.pos_rad : kNaN;
        m.velocity = kNaN;
        m.torque   = kNaN;
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

    // ── CAN transport ─────────────────────────────────────────────────────────
    // First entry deliberately: if the bus is down, every motor entry below is
    // stale by definition, and this says so.
    {
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = st.hardware_id = "can/" + can_interface_;
        const auto state = bus_state_.load();
        switch (state) {
            case CanBusState::BUS_OFF:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                st.message = "BUS-OFF — no frames reach any motor";
                break;
            case CanBusState::ERROR_PASSIVE:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                st.message = "Error-passive — controller degraded";
                break;
            case CanBusState::ERROR_WARNING:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                st.message = "Error counters elevated";
                break;
            case CanBusState::OK:
                st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                st.message = "OK";
                break;
        }
        st.values.push_back(kv("tx_error_count", std::to_string(tx_error_count_.load())));
        st.values.push_back(kv("rx_error_count", std::to_string(rx_error_count_.load())));
        // Cumulative, not a rate: a value that climbs during a manoeuvre is the
        // evidence that the bus is saturating.
        st.values.push_back(kv("tx_dropped",     std::to_string(tx_dropped_.load())));
        arr.status.push_back(st);
    }

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
        const auto & s = drive_state_[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name = st.hardware_id = std::string("damiao/drive_") + kNames[i];
        const auto fault = damiao_protocol::translateFault(s.err);
        if (!s.valid) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; st.message = "No feedback";
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
// finds an empty queue on essentially every tick; the cost of checking is a
// mutex acquisition, and the benefit is that rclcpp publishing (which
// allocates) never runs inside read().

void RoverHardwareInterface::publish_fault_events()
{
    if (!fault_event_pub_) return;

    std::vector<indomitus_interfaces::msg::FaultEvent> events;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        if (pending_fault_events_.empty()) return;
        events.swap(pending_fault_events_);
    }

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
