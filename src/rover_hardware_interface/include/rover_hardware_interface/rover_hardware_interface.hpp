#pragma once

#include <array>
#include <atomic>
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

#include "rover_hardware_interface/damiao_protocol.hpp"
#include "rover_hardware_interface/steadywin_protocol.hpp"

namespace rover_hardware_interface {

constexpr std::size_t NUM_WHEELS = 4;

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
    // SocketCAN

    bool open_can_socket();
    void close_can_socket();
    bool send_can_frame(uint32_t id, const uint8_t * data, uint8_t dlc, bool is_extended = false);

    std::mutex can_tx_mutex_;

    void rx_thread_fn();
    void dispatch_can_frame(const struct can_frame & frame);

    void send_enable_frames();
    void send_disable_frames();
    void send_shutdown_frames();   ///< zero → settle → disable (called from on_deactivate)

    // Service callbacks

    void on_set_motors_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>          req,
        std::shared_ptr<std_srvs::srv::SetBool::Response>               res);

    void on_set_steer_zero(
        const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request>  req,
        std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response>       res);

    // Diagnostic / status publishers

    void publish_chassis_status();
    void publish_diagnostics();
    void publish_fault_events();

    // Fault detection
    //
    // Detection runs in read() at the control rate, because that is the only
    // place that observes every feedback update and can hold a consistent
    // snapshot of all motors. Publishing is deferred to a timer on hw_node_:
    // rclcpp publishing allocates, and a stall in read() longer than the Damiao
    // TIMEOUT register would provoke the very comm-loss fault we are reporting.

    struct FaultTracker {
        rover_fault::Fault fault{rover_fault::Fault::NONE};
        bool health_valid{true};
        bool seen{false};
    };

    /// Compare one motor's current condition against its tracker and queue an
    /// event if it changed. Caller must hold feedback_mutex_.
    void detect_fault_transition(
        std::size_t index, bool is_steer,
        rover_fault::Fault current, bool health_valid, uint8_t raw_code);

    std::array<FaultTracker, NUM_WHEELS> steer_fault_;
    std::array<FaultTracker, NUM_WHEELS> drive_fault_;

    /// Queued under feedback_mutex_ by read(), drained by publish_fault_events().
    /// Faults are rare, so this is empty on essentially every cycle.
    std::vector<indomitus_interfaces::msg::FaultEvent> pending_fault_events_;
    static constexpr std::size_t kMaxPendingFaultEvents{256};

    rclcpp::Logger logger_{rclcpp::get_logger("RoverHardware")};
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};

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

    // Concurrency & Hardware

    int can_fd_{-1};
    std::atomic<bool> rx_running_{false};
    std::thread       rx_thread_;

    // ─────────────────────────────────────────────────────────────────────────
    // ROS 2 interfaces
    // (hardware plugins share the lifecycle node provided by controller_manager)
    // ─────────────────────────────────────────────────────────────────────────

    rclcpp::Publisher<indomitus_interfaces::msg::ChassisStatus>::SharedPtr chassis_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr    diagnostics_pub_;
    rclcpp::Publisher<indomitus_interfaces::msg::FaultEvent>::SharedPtr    fault_event_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                  motor_enable_srv_;
    rclcpp::Service<indomitus_interfaces::srv::SetSteerZero>::SharedPtr set_steer_zero_srv_;

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
    rclcpp::Time last_write_time_;
    static constexpr double kWatchdogTimeoutSec{0.5};  ///< zero commands if write() stalls
};

}  // namespace rover_hardware_interface
