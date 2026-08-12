#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "indomitus_interfaces/msg/chassis_status.hpp"
#include "indomitus_interfaces/msg/fault_event.hpp"
#include "indomitus_interfaces/srv/set_steer_zero.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rover_hardware_interface/can_bus.hpp"
#include "rover_hardware_interface/constants.hpp"
#include "rover_hardware_interface/damiao_protocol.hpp"
#include "rover_hardware_interface/fault_detector.hpp"
#include "rover_hardware_interface/steadywin_protocol.hpp"

namespace rover_hardware_interface {

class RoverHardwareInterface : public hardware_interface::SystemInterface
{
public:
#ifdef JAZZY_OR_LATER
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
#endif

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    // Interface export

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Control loop

    /// Called by controller_manager: decode latest CAN feedback → state interfaces
    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    /// Called by controller_manager: command interfaces → CAN frames
    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    void send_enable_frames();
    void send_disable_frames();
    void send_shutdown_frames();   ///< disable → settle → disable (called from on_deactivate)

    /// One pass of disable frames over every steer and drive motor.
    /// Caller must hold tx_sequence_mutex_.
    void send_disable_burst();

    /// Damiao clear-error frame to every drive motor, over the given socket.
    void send_clear_error_frames(CanBus & bus);

    // Service callbacks

    void on_set_motors_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>          req,
        std::shared_ptr<std_srvs::srv::SetBool::Response>               res);

    void on_clear_motor_errors(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>          req,
        std::shared_ptr<std_srvs::srv::Trigger::Response>               res);

    void on_set_steer_zero(
        const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request>  req,
        std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response>       res);

    // Diagnostic / status publishers

    void publish_chassis_status();
    void publish_diagnostics();
    void publish_fault_events();

    // Feedback freshness

    /// Monotonic nanoseconds. Deliberately not ROS time: an NTP step mid-run
    /// (routine on a rover that gets a fix minutes after boot) would otherwise
    /// register as every motor going silent at once.
    static int64_t steady_now_ns();

    /// Per-motor verdict on whether feedback is present *and* recent. The
    /// vendor "valid" flags latch true on the first frame and never clear, so
    /// on their own they can never report a motor that stopped transmitting.
    struct FeedbackFreshness {
        std::array<bool, NUM_WHEELS> steer{};
        std::array<bool, NUM_WHEELS> drive{};
    };

    /// Caller must hold feedback_mutex_.
    FeedbackFreshness feedback_freshness(int64_t now_ns) const;

    // logger_/clock_ must stay declared before can_bus_/fault_detector_:
    // members initialize in declaration order, and both are constructed
    // from these two.
    rclcpp::Logger logger_{rclcpp::get_logger("RoverHardware")};
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};

    // CAN transport (socket, RX thread, bus health) — see CanBus. Frame
    // routing to a motor's decoder stays here: that's protocol/motor
    // knowledge the transport itself shouldn't have.
    CanBus can_bus_{logger_, clock_};

    void dispatch_can_frame(const struct can_frame & frame);

    // Fault detection — edge-triggering and event queuing live in
    // FaultDetector; read() just feeds it a snapshot every cycle, because
    // that is the only place that observes every feedback update and can
    // hold a consistent view of all motors. Publishing is deferred to a
    // timer on hw_node_: rclcpp publishing allocates, and a stall in read()
    // longer than the Damiao TIMEOUT register would provoke the very
    // comm-loss fault we are reporting.
    FaultDetector fault_detector_{clock_};

    rclcpp::Node::SharedPtr hw_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr hw_executor_;
    std::thread hw_node_thread_;

    // Parameters (populated in on_init from HardwareInfo::hardware_parameters)

    std::string can_interface_{"can0"};

    std::array<uint8_t, NUM_WHEELS> steer_ids_;   ///< Steadywin motor CAN IDs [FL,FR,RL,RR]
    std::array<uint8_t, NUM_WHEELS> drive_ids_;   ///< Damiao motor CAN IDs    [FL,FR,RL,RR]

    float    drive_pmax_{12.5f};    ///< Damiao fixed-point position range [rad]
    float    drive_vmax_{50.0f};    ///< Damiao fixed-point velocity range [rad/s]
    float    drive_tmax_{20.0f};    ///< Damiao fixed-point torque range   [Nm]
    uint32_t mst_id_{0};            ///< Damiao master CAN ID for broadcast feedback

    std::array<std::string, NUM_WHEELS> steer_joint_names_;
    std::array<std::string, NUM_WHEELS> drive_joint_names_;

    /// How long feedback may be absent before the motor counts as silent.
    /// The two differ because the two channels arrive at different rates:
    /// Steadywin health comes only from the 1 Hz 0xAE poll, while Damiao
    /// feedback comes back with every command frame at the control rate.
    double steer_feedback_timeout_{3.0};   ///< [s] — three missed status polls
    double drive_feedback_timeout_{0.5};   ///< [s] — ~50 missed cycles at 100 Hz

    static constexpr std::array<double, 4> kDriveSigns = {-1.0f, 1.0f, -1.0f, 1.0f};

    // State interface backing storage
    // (ros2_control binds pointers to these — never reallocate after export

    std::array<double, NUM_WHEELS> steer_pos_{};    ///< steering joint position [rad]
    std::array<double, NUM_WHEELS> drive_pos_{};    ///< drive wheel position     [rad]
    std::array<double, NUM_WHEELS> drive_vel_{};    ///< drive wheel velocity     [rad/s]

    // Command interface backing storage

    std::array<double, NUM_WHEELS> steer_cmd_{};    ///< target steering position [rad]
    std::array<double, NUM_WHEELS> drive_cmd_{};    ///< target drive velocity    [rad/s]

    // Raw motor feedback (filled by rx_thread, read by read())

    std::mutex feedback_mutex_;
    std::array<steadywin_protocol::MotorState, NUM_WHEELS> steer_state_{};
    std::array<damiao_protocol::MotorState,    NUM_WHEELS> drive_state_{};

    /// Arrival time of the last frame that carried *health* for each motor,
    /// in monotonic ns; 0 means never. Written by the RX thread inside
    /// dispatch_can_frame(), read under feedback_mutex_ like the states.
    /// Steer tracks the 0xAE status response specifically — a position reply
    /// proves the motor is alive but says nothing about its fault register.
    std::array<int64_t, NUM_WHEELS> steer_status_rx_ns_{};
    std::array<int64_t, NUM_WHEELS> drive_feedback_rx_ns_{};

    /// Feedback is only expected while the motors are enabled: nothing polls
    /// them otherwise, so silence is the correct state, not a dropout. While
    /// disabled this tracks "now", which both suppresses false SIGNAL_LOST and
    /// grants a full timeout of grace after the motors come back up.
    std::atomic<int64_t> feedback_expected_since_ns_{0};

    // ─────────────────────────────────────────────────────────────────────────
    // ROS 2 interfaces
    // (hardware plugins share the lifecycle node provided by controller_manager)
    // ─────────────────────────────────────────────────────────────────────────

    rclcpp::Publisher<indomitus_interfaces::msg::ChassisStatus>::SharedPtr chassis_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr    diagnostics_pub_;
    rclcpp::Publisher<indomitus_interfaces::msg::FaultEvent>::SharedPtr    fault_event_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                  motor_enable_srv_;
    rclcpp::Service<indomitus_interfaces::srv::SetSteerZero>::SharedPtr set_steer_zero_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                  clear_motor_errors_srv_;

    rclcpp::TimerBase::SharedPtr status_poll_timer_;      ///< 1 Hz  — 0xAE + 0xA3 query
    rclcpp::TimerBase::SharedPtr chassis_status_timer_;   ///< 10 Hz — /chassis/motor_states
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;      ///< 1 Hz  — /diagnostics
    rclcpp::TimerBase::SharedPtr fault_event_timer_;      ///< 20 Hz — /fault_events drain
    rclcpp::TimerBase::SharedPtr watchdog_timer_;         ///< 10 Hz — cmd_vel timeout guard

    // Runtime state

    /// Written by service callbacks and the lifecycle transitions, read by the
    /// control thread in read()/write() and by three timer callbacks. Atomic
    /// because those are different threads; a plain bool here is a data race.
    std::atomic<bool> motors_enabled_{false};

    /// Serializes whole multi-frame TX sequences against each other — not
    /// individual frames, which CanBus already guards with its own tx_mutex_.
    /// What matters on disable is that no command frame reaches a motor after
    /// its disable frame: such a frame re-arms the Damiao TIMEOUT watchdog
    /// (register 9, 200 ms) on a motor nothing will talk to again, and the
    /// motor then faults itself out with COMM_LOSS (ERR 0xD). Without this,
    /// the last drive in drive_ids_ loses that race almost every time — its
    /// disable is the last frame of the disable burst and its command the last
    /// frame of the write() cycle, so any overlap at all is enough.
    std::mutex tx_sequence_mutex_;

    /// Set in on_activate(), cleared in on_deactivate()/on_shutdown(). read()
    /// is called by controller_manager as soon as the component is merely
    /// configured (inactive) — before on_activate() has ever opened the CAN
    /// socket — so "socket not open" is only a real fault once we know we
    /// should have one.
    std::atomic<bool> activated_{false};

    rclcpp::Time last_write_time_;
    static constexpr double kWatchdogTimeoutSec{0.5};  ///< zero commands if write() stalls
};

}  // namespace rover_hardware_interface
