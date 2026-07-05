#include "rover_sim/sim_diff_bar_node.hpp"
#include "rclcpp/rclcpp.hpp"

SimDiffBar::SimDiffBar() : Node("sim_diff_bar") {
    // Параметри жорсткості/демпфування диференціала — винесені назовні для тюнінгу
    this->declare_parameter<double>("k_stiffness", 400.0);
    this->declare_parameter<double>("d_damping", 10.0);
    this->declare_parameter<double>("effort_limit", 1500.0);

    k_ = this->get_parameter("k_stiffness").as_double();
    d_ = this->get_parameter("d_damping").as_double();
    effort_limit_ = this->get_parameter("effort_limit").as_double();

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimDiffBar::jointStateCallback, this, std::placeholders::_1));

    effort_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/diff_bar_effort_controller/commands", 10);
}

void SimDiffBar::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    double l_pos = 0.0, l_vel = 0.0;
    double r_pos = 0.0, r_vel = 0.0;
    bool found_l = false, found_r = false;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "l_rocker_joint") {
            l_pos = msg->position[i];
            l_vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
            found_l = true;
        } else if (msg->name[i] == "r_rocker_joint") {
            r_pos = msg->position[i];
            r_vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
            found_r = true;
        }
    }

    if (!found_l || !found_r) return;

    // Дзеркальна умова: l_pos + r_pos == 0 у ідеальному диференціалі
    double error = l_pos + r_pos;
    double error_dot = l_vel + r_vel;

    double tau = -k_ * error - d_ * error_dot;

    // Обмежуємо момент, щоб уникнути числової нестабільності при різких ударах
    tau = std::clamp(tau, -effort_limit_, effort_limit_);

    // Знаки залежать від напрямку осей — l_rocker і r_rocker мають
    // однакову вісь (0 1 0), тому момент компенсації прикладається
    // з протилежними знаками, щоб штовхати їх у дзеркальні боки
    auto cmd_msg = std_msgs::msg::Float64MultiArray();
    cmd_msg.data.push_back(tau);   // l_rocker_joint
    cmd_msg.data.push_back(tau);  // r_rocker_joint

    effort_cmd_pub_->publish(cmd_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDiffBar>());
    rclcpp::shutdown();
    return 0;
}