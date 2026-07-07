#pragma once

#include <array>
#include <atomic>
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

// 6 DOF arm = 6 motors
constexpr std::size_t NUM_JOINTS = 6;

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

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // SocketCAN methods
    bool open_can_socket();
    void close_can_socket();
    bool send_can_frame(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t dlc);
    
    // Background thread for non-blocking CAN reception
    void rx_thread_fn();

    // Motor control sequences
    void send_enable_frames();
    void send_disable_frames();

    // ROS 2 parameters and logging
    rclcpp::Logger logger_{rclcpp::get_logger("ArmCanSystem")};
    std::string can_interface_{"can0"};

    // Motor IDs [Base, Shoulder, Elbow, Wrist1, Wrist2, EndEffector]
    std::array<uint8_t, NUM_JOINTS> motor_ids_{20, 21, 22, 23, 24, 25};
    std::array<std::string, NUM_JOINTS> joint_names_;

    // Hardware interface memory buffers
    // Command variables (written by MoveIt/joint_trajectory_controller)
    std::array<double, NUM_JOINTS> joint_position_command_{0.0};
    std::array<double, NUM_JOINTS> joint_velocity_command_{0.0};

    // State variables (read from CAN bus)
    std::array<double, NUM_JOINTS> joint_position_state_{0.0};
    std::array<double, NUM_JOINTS> joint_velocity_state_{0.0};

    // Shadow buffer for thread-safe data transfer from rx_thread
    std::array<double, NUM_JOINTS> hw_position_states_{0.0};

    // Concurrency and SocketCAN file descriptor
    int can_fd_{-1};
    std::mutex can_tx_mutex_;
    std::mutex feedback_mutex_;
    
    std::atomic<bool> rx_running_{false};
    std::thread rx_thread_;
    bool motors_enabled_{false};
};

} // namespace arm_hardware_interface