#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace arm_hardware_interface {

constexpr std::size_t NUM_JOINTS = 6;
constexpr std::size_t NUM_STEADYWIN = 3;   // joints 0..2 (base, shoulder, elbow)

class ArmCanSystem : public hardware_interface::SystemInterface
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

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Shared on_init body for both ROS distro signatures
    hardware_interface::CallbackReturn init_from_info(
        const hardware_interface::HardwareInfo & info);

    // SocketCAN
    bool open_can_socket();
    void close_can_socket();
    bool send_can_frame(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t dlc);
    void rx_thread_fn();

    // Motor sequences
    void send_enable_frames();     // SW: 0xF0 limits config; DM: 0xFC enable
    void send_disable_frames();    // torqueless MIT first, then SW 0xCF / DM 0xFD
    bool wait_for_all_feedback(double timeout_sec);  // active polling, abort on missing motor
    void safe_stop();              // full disable (torqueless) — only for on_error, real faults
    void stop_holding();           // clean shutdown WITHOUT disabling — see on_deactivate

    // Static gravity compensation: per-joint holding torque (motor frame, Nm,
    // clamped to gravity_ff_max_nm_) at the current measured pose. See
    // kLinkMass/kMotorMass and the math helpers atop arm_hardware_interface.cpp.
    std::array<float, NUM_JOINTS> compute_gravity_feedforward() const;

    // Coordinate transforms (URDF <-> motor frame)
    inline double urdf_to_motor(std::size_t i, double urdf_rad) const
    { return urdf_rad * joint_directions_[i] + joint_offsets_[i]; }
    inline double motor_to_urdf(std::size_t i, double motor_rad) const
    { return (motor_rad - joint_offsets_[i]) * joint_directions_[i]; }

    rclcpp::Logger logger_{rclcpp::get_logger("ArmCanSystem")};
    std::string can_interface_{"can0"};

    // ---- Calibration & gains: loaded from URDF <ros2_control> parameters ----
    // Defaults are placeholders; real values come from arm_macro.xacro.
    std::array<double, NUM_JOINTS> joint_directions_{1, 1, 1, 1, 1, 1};
    std::array<double, NUM_JOINTS> joint_offsets_{0, 0, 0, 0, 0, 0};
    std::array<double, NUM_JOINTS> joint_kp_{30.0, 120.0, 100.0, 20.0, 30.0, 15.0};
    std::array<double, NUM_JOINTS> joint_kd_{1.5, 4.0, 3.5, 1.0, 1.0, 0.5};
    std::array<uint8_t, NUM_JOINTS> motor_ids_{20, 21, 22, 23, 24, 25};

    // ---- Safety parameters (URDF hardware params) ----
    double gain_ramp_secs_{8.0};        // stiffness ramp 0 -> full after activation
    double max_cmd_speed_rad_s_{0.3};   // rate limiter on out going position commands
    double feedback_timeout_secs_{2.0}; // abort activation if a motor never replies

    // ---- Gravity compensation (feed-forward torque, t_ff in the MIT frame) ----
    bool   gravity_ff_enabled_{false};
    double gravity_ff_max_nm_{3.0};     // hard per-joint clamp, well under T_MAX_NM=18

    std::array<std::string, NUM_JOINTS> joint_names_;

    // ros2_control buffers
    std::array<double, NUM_JOINTS> joint_position_command_{};
    std::array<double, NUM_JOINTS> joint_velocity_command_{};
    std::array<double, NUM_JOINTS> joint_position_state_{};
    std::array<double, NUM_JOINTS> joint_velocity_state_{};

    // RX-thread shadow buffers (URDF frame) + liveness flags
    std::array<double, NUM_JOINTS> hw_position_states_{};
    std::array<double, NUM_JOINTS> hw_velocity_states_{};
    std::array<bool, NUM_JOINTS>   feedback_seen_{};
    std::array<uint8_t, NUM_JOINTS> dm_last_err_{};   // Damiao error nibble per joint

    // Rate limiter memory: last position command actually sent (URDF frame)
    std::array<double, NUM_JOINTS> last_sent_command_{};
    bool have_last_sent_{false};

    // Stiffness ramp
    std::chrono::steady_clock::time_point ramp_start_;
    bool ramp_started_{false};

    int can_fd_{-1};
    std::mutex can_tx_mutex_;
    std::mutex feedback_mutex_;
    std::atomic<bool> rx_running_{false};
    std::thread rx_thread_;
    std::atomic<bool> motors_enabled_{false};
};

} // namespace arm_hardware_interface