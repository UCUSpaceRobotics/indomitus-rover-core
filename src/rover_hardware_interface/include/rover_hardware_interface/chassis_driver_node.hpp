#pragma once
#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "indomitus_interfaces/msg/wheel_targets.hpp"
#include "indomitus_interfaces/msg/chassis_status.hpp"
#include "indomitus_interfaces/srv/set_steer_zero.hpp"
#include "rover_hardware_interface/damiao_protocol.hpp"
#include "rover_hardware_interface/steadywin_protocol.hpp"

namespace chassis_driver {

class ChassisDriverNode : public rclcpp::Node {
public:
    explicit ChassisDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

    // Graceful shutdown: zero → settle → disable all motors
    void sendShutdownFrames();

private:
    void onWheelTargets(const indomitus_interfaces::msg::WheelTargets::SharedPtr msg);
    void onCanFrame(const can_msgs::msg::Frame::SharedPtr msg);
    void onSetMotorsEnabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void publishJointStates();
    void publishChassisStatus();
    void publishDiagnostics();
    void sendEnableFrames();
    void onSetSteerZero(
        const indomitus_interfaces::srv::SetSteerZero::Request::SharedPtr req,
        indomitus_interfaces::srv::SetSteerZero::Response::SharedPtr res);
    void sendDisableFrames();
    void tryPublishBootDisable();

    // Subscriptions
    rclcpp::Subscription<indomitus_interfaces::msg::WheelTargets>::SharedPtr wheel_targets_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr from_can_sub_;

    // Services
    rclcpp::Service<indomitus_interfaces::srv::SetSteerZero>::SharedPtr set_steer_zero_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motor_enable_service_;

    // Publishers
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr to_can_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<indomitus_interfaces::msg::ChassisStatus>::SharedPtr chassis_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr status_poll_timer_;       // 1 Hz: 0xAE query to Steadywin
    rclcpp::TimerBase::SharedPtr chassis_status_timer_;    // 10 Hz: /chassis/motor_states
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;       // 1 Hz: /diagnostics
    rclcpp::TimerBase::SharedPtr boot_retry_timer_;
    int boot_retry_attempts_ = 0;
    int boot_retry_max_attempts_ = 25; // ~25 * 200ms = 5s

    // Motor IDs [FL, FR, RL, RR]
    std::array<uint8_t, 4> steer_ids_;  // Steadywin rotation motors
    std::array<uint8_t, 4> drive_ids_;  // Damiao drive motors

    // Damiao MIT feedback decoding ranges (fixed-point → float)
    float drive_pmax_, drive_vmax_, drive_tmax_;

    // Steering angle limits (rad)
    float steer_angle_min_, steer_angle_max_;
    uint32_t mst_id_;

    // Joint names
    std::vector<std::string> steer_joint_names_;
    std::vector<std::string> drive_joint_names_;

    // Motor feedback state [FL, FR, RL, RR]
    std::array<steadywin_protocol::MotorState, 4> steer_state_;
    std::array<damiao_protocol::MotorState, 4>    drive_state_;

    bool motors_enabled_ = false;

    // ====== WATCHDOG PART ======
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_wheel_targets_time_;
    static constexpr double kWheelTargetsTimeoutSec = 0.5;
    // ====== WATCHDOG PART ======
};

} // namespace chassis_driver
