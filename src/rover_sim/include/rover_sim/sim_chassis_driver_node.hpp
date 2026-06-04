#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <indomitus_interfaces/msg/wheel_targets.hpp>

class SimChassisDriver : public rclcpp::Node {
public:
    SimChassisDriver();
    ~SimChassisDriver() = default;

private:
    void wheelTargetsCallback(const indomitus_interfaces::msg::WheelTargets::SharedPtr msg);

    rclcpp::Subscription<indomitus_interfaces::msg::WheelTargets>::SharedPtr wheel_targets_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      drive_pub_;
};