#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <indomitus_msgs/msg/wheel_targets.hpp>

class ICRController : public rclcpp::Node {
public:
    ICRController();
    ~ICRController() = default;

private:
    void wheelTargetsCallback(const indomitus_msgs::msg::WheelTargets::SharedPtr msg);

    rclcpp::Subscription<indomitus_msgs::msg::WheelTargets>::SharedPtr wheel_targets_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      drive_pub_;
};