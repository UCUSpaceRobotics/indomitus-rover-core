#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// rover_hardware_interface.hpp
//
// ros2_control SystemInterface plugin for 4-wheel swerve rover.
//
// Claimed interfaces:
//   StateInterfaces:
//     fl/fr/rl/rr  wheel_mount_joint  — position [rad]   (Steadywin feedback)
//     fl/fr/rl/rr  wheel_joint        — position [rad]   (Damiao MIT feedback)
//     fl/fr/rl/rr  wheel_joint        — velocity [rad/s] (Damiao MIT feedback)
//
//   CommandInterfaces:
//     fl/fr/rl/rr  wheel_mount_joint  — position [rad]   → Steadywin 0xC2
//     fl/fr/rl/rr  wheel_joint        — velocity [rad/s] → Damiao 0x200
//
// CAN topics (ros2_socketcan):
//   /to_can_bus   [can_msgs/Frame]  — outgoing CAN frames
//   /from_can_bus [can_msgs/Frame]  — incoming CAN feedback
//
// Services:
//   ~/set_motors_enabled  [std_srvs/SetBool]
//   ~/set_steer_zero      [indomitus_interfaces/SetSteerZero]
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "can_msgs/msg/frame.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "indomitus_interfaces/srv/set_steer_zero.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "indomitus_interfaces/msg/chassis_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rover_hardware_interface/damiao_protocol.hpp"
#include "rover_hardware_interface/steadywin_protocol.hpp"

namespace rover_hardware_interface {

constexpr std::size_t NUM_WHEELS = 4;

class RoverHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // ── SystemInterface lifecycle ──────────────────────────────────────────────

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    // ── Interface export ───────────────────────────────────────────────────────

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ── Control loop ───────────────────────────────────────────────────────────

    /// Called by controller_manager: decode latest CAN feedback → state interfaces
    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    /// Called by controller_manager: command interfaces → CAN frames
    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // ── CAN callbacks ──────────────────────────────────────────────────────────

    void on_can_frame(const can_msgs::msg::Frame::SharedPtr msg);

    // ── Motor lifecycle helpers ────────────────────────────────────────────────

    void send_enable_frames();
    void send_disable_frames();
    void send_shutdown_frames();   ///< zero → settle → disable (called from on_deactivate)

    // ── Service callbacks ──────────────────────────────────────────────────────

    void on_set_motors_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request>          req,
        std::shared_ptr<std_srvs::srv::SetBool::Response>               res);

    void on_set_steer_zero(
        const std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Request>  req,
        std::shared_ptr<indomitus_interfaces::srv::SetSteerZero::Response>       res);

    // ── Diagnostic / status publishers ────────────────────────────────────────

    void publish_chassis_status();
    void publish_diagnostics();

    // ── Boot-time CAN subscriber detection ────────────────────────────────────

    void try_publish_boot_disable();

    // ─────────────────────────────────────────────────────────────────────────
    // Parameters (populated in on_init from HardwareInfo::hardware_parameters)
    // ─────────────────────────────────────────────────────────────────────────

    std::array<uint8_t, NUM_WHEELS> steer_ids_;   ///< Steadywin motor CAN IDs [FL,FR,RL,RR]
    std::array<uint8_t, NUM_WHEELS> drive_ids_;   ///< Damiao motor CAN IDs    [FL,FR,RL,RR]

    float    drive_pmax_{12.5f};    ///< Damiao fixed-point position range [rad]
    float    drive_vmax_{50.0f};    ///< Damiao fixed-point velocity range [rad/s]
    float    drive_tmax_{20.0f};    ///< Damiao fixed-point torque range   [Nm]
    uint32_t mst_id_{0};            ///< Damiao master CAN ID for broadcast feedback

    std::array<std::string, NUM_WHEELS> steer_joint_names_;
    std::array<std::string, NUM_WHEELS> drive_joint_names_;

    // ─────────────────────────────────────────────────────────────────────────
    // State interface backing storage
    // (ros2_control binds pointers to these — never reallocate after export)
    // ─────────────────────────────────────────────────────────────────────────

    std::array<double, NUM_WHEELS> steer_pos_{};    ///< steering joint position [rad]
    std::array<double, NUM_WHEELS> drive_pos_{};    ///< drive wheel position     [rad]
    std::array<double, NUM_WHEELS> drive_vel_{};    ///< drive wheel velocity     [rad/s]

    // ─────────────────────────────────────────────────────────────────────────
    // Command interface backing storage
    // ─────────────────────────────────────────────────────────────────────────

    std::array<double, NUM_WHEELS> steer_cmd_{};    ///< target steering position [rad]
    std::array<double, NUM_WHEELS> drive_cmd_{};    ///< target drive velocity    [rad/s]

    // ─────────────────────────────────────────────────────────────────────────
    // Raw motor feedback (filled by on_can_frame, read by read())
    // ─────────────────────────────────────────────────────────────────────────

    std::array<steadywin_protocol::MotorState, NUM_WHEELS> steer_state_{};
    std::array<damiao_protocol::MotorState,    NUM_WHEELS> drive_state_{};

    // ─────────────────────────────────────────────────────────────────────────
    // ROS 2 interfaces
    // (hardware plugins share the lifecycle node provided by controller_manager)
    // ─────────────────────────────────────────────────────────────────────────

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr           to_can_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr        from_can_sub_;

    rclcpp::Publisher<indomitus_interfaces::msg::ChassisStatus>::SharedPtr chassis_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr    diagnostics_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                          motor_enable_srv_;
    rclcpp::Service<indomitus_interfaces::srv::SetSteerZero>::SharedPtr         set_steer_zero_srv_;

    rclcpp::TimerBase::SharedPtr status_poll_timer_;      ///< 1 Hz  — 0xAE + 0xA3 query
    rclcpp::TimerBase::SharedPtr chassis_status_timer_;   ///< 10 Hz — /chassis/motor_states
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;      ///< 1 Hz  — /diagnostics
    rclcpp::TimerBase::SharedPtr watchdog_timer_;         ///< 10 Hz — cmd_vel timeout guard
    rclcpp::TimerBase::SharedPtr boot_retry_timer_;

    // ─────────────────────────────────────────────────────────────────────────
    // Runtime state
    // ─────────────────────────────────────────────────────────────────────────

    bool     motors_enabled_{false};
    rclcpp::Time last_write_time_;

    int boot_retry_attempts_{0};
    static constexpr int    kBootRetryMax{25};         ///< 25 × 200 ms = 5 s
    static constexpr double kWatchdogTimeoutSec{0.5};  ///< zero commands if write() stalls
};

}  // namespace rover_hardware_interface