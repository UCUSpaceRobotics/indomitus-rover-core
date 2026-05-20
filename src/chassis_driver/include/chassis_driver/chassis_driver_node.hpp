#pragma once
#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "indomitus_interfaces/msg/wheel_targets.hpp"
#include "indomitus_interfaces/msg/chassis_status.hpp"
#include "chassis_driver/damiao_protocol.hpp"
#include "chassis_driver/steadywin_protocol.hpp"

namespace chassis_driver {

class ChassisDriverNode : public rclcpp::Node {
public:
    explicit ChassisDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

    // Graceful shutdown: zero → settle → disable all motors
    void sendDisableFrames();

private:
    void onWheelTargets(const indomitus_interfaces::msg::WheelTargets::SharedPtr msg);
    void onCanFrame(const can_msgs::msg::Frame::SharedPtr msg);
    void publishJointStates();
    void publishChassisStatus();
    void publishDiagnostics();
    void sendEnableFrames();

    // Subscriptions
    rclcpp::Subscription<indomitus_interfaces::msg::WheelTargets>::SharedPtr wheel_targets_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr from_can_sub_;

    // Publishers
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr to_can_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<indomitus_interfaces::msg::ChassisStatus>::SharedPtr chassis_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr enable_timer_;            // one-shot, 3s: initial motor enable
    rclcpp::TimerBase::SharedPtr status_poll_timer_;       // 1 Hz: 0xAE query to Steadywin
    rclcpp::TimerBase::SharedPtr chassis_status_timer_;    // 10 Hz: /chassis/motor_states
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;       // 1 Hz: /diagnostics

    // Motor IDs [FL, FR, RL, RR]
    std::array<uint8_t, 4> steer_ids_;  // Steadywin rotation motors
    std::array<uint8_t, 4> drive_ids_;  // Damiao drive motors

    // Damiao MIT feedback decoding ranges (fixed-point → float)
    float drive_pmax_, drive_vmax_, drive_tmax_;
    uint32_t mst_id_;

    // Joint names
    std::vector<std::string> steer_joint_names_;
    std::vector<std::string> drive_joint_names_;

    // Motor feedback state [FL, FR, RL, RR]
    std::array<steadywin_protocol::MotorState, 4> steer_state_;
    std::array<damiao_protocol::MotorState, 4>    drive_state_;

    bool motors_enabled_ = false;
    rclcpp::Time last_wheel_targets_time_{0, 0, RCL_ROS_TIME};
};

} // namespace chassis_driver
