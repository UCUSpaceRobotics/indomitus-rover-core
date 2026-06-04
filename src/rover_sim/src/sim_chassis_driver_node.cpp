#include "rover_sim/sim_chassis_driver_node.hpp"

SimChassisDriver::SimChassisDriver()
: Node("sim_chassis_driver")
{
    declare_parameter("steer_scale", 1.0);
    declare_parameter("drive_scale", 1.0);

    wheel_targets_sub_ = create_subscription<indomitus_interfaces::msg::WheelTargets>(
        "wheel_targets", 10,
        std::bind(&SimChassisDriver::wheelTargetsCallback, this, std::placeholders::_1));

    steer_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/steering_controller/commands", 10);

    drive_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/drive_controller/commands", 10);
}

void SimChassisDriver::wheelTargetsCallback(const indomitus_interfaces::msg::WheelTargets::SharedPtr msg)
{
    const double steer_scale = get_parameter("steer_scale").as_double();
    const double drive_scale = get_parameter("drive_scale").as_double();

    std_msgs::msg::Float64MultiArray steer_msg, drive_msg;
    steer_msg.data.resize(4);
    drive_msg.data.resize(4);

    // Order: FL, FR, RL, RR
    steer_msg.data[0] = msg->fl_angle * steer_scale;
    steer_msg.data[1] = msg->fr_angle * steer_scale;
    steer_msg.data[2] = msg->rl_angle * steer_scale;
    steer_msg.data[3] = msg->rr_angle * steer_scale;

    drive_msg.data[0] = msg->fl_speed * drive_scale;
    drive_msg.data[1] = msg->fr_speed * drive_scale;
    drive_msg.data[2] = msg->rl_speed * drive_scale;
    drive_msg.data[3] = msg->rr_speed * drive_scale;

    steer_pub_->publish(steer_msg);
    drive_pub_->publish(drive_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimChassisDriver>());
    rclcpp::shutdown();
    return 0;
}