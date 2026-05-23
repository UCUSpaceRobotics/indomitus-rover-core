#include "indomitus_rover_sim/sim_diff_bar_node.hpp"
#include "rclcpp/rclcpp.hpp"

SimDiffBar::SimDiffBar() : Node("sim_diff_bar") {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimDiffBar::jointStateCallback, this, std::placeholders::_1));

    r_rocker_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/r_rocker_position_controller/commands", 10);
}

void SimDiffBar::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    double l_rocker_pos = 0.0;
    bool found = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "l_rocker_joint") {
            l_rocker_pos = msg->position[i];
            found = true;
            break;
        }
    }

    if (!found) return;

    auto cmd_msg = std_msgs::msg::Float64MultiArray();
    cmd_msg.data.push_back(-l_rocker_pos);
    r_rocker_cmd_pub_->publish(cmd_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDiffBar>());
    rclcpp::shutdown();
    return 0;
}