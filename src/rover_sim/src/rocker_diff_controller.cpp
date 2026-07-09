// rocker_diff_controller.cpp
#include "rover_sim/rocker_diff_controller.hpp"
#include <algorithm>
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rover_sim::RockerDiffController,
  controller_interface::ControllerInterface
)

namespace rover_sim {

controller_interface::InterfaceConfiguration
RockerDiffController::command_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {"l_rocker_joint/effort", "r_rocker_joint/effort"}
    };
}

controller_interface::InterfaceConfiguration
RockerDiffController::state_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {"l_rocker_joint/position", "l_rocker_joint/velocity",
        "r_rocker_joint/position", "r_rocker_joint/velocity"}
    };
}

controller_interface::CallbackReturn RockerDiffController::on_init() {
    auto_declare<double>("k_stiffness", 200.0);
    auto_declare<double>("d_damping", 10.0);
    auto_declare<double>("effort_limit", 150.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerDiffController::on_configure(const rclcpp_lifecycle::State &) {
    k_ = get_node()->get_parameter("k_stiffness").as_double();
    d_ = get_node()->get_parameter("d_damping").as_double();
    effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RockerDiffController::update(
    const rclcpp::Time &, const rclcpp::Duration &) {
#if defined(JAZZY_OR_LATER)
    double l_pos = state_interfaces_[0].get_optional().value_or(0.0);
    double l_vel = state_interfaces_[1].get_optional().value_or(0.0);
    double r_pos = state_interfaces_[2].get_optional().value_or(0.0);
    double r_vel = state_interfaces_[3].get_optional().value_or(0.0);
#else
    double l_pos = state_interfaces_[0].get_value();
    double l_vel = state_interfaces_[1].get_value();
    double r_pos = state_interfaces_[2].get_value();
    double r_vel = state_interfaces_[3].get_value();
#endif

    double error = l_pos + r_pos;
    double error_dot = l_vel + r_vel;

    double tau = std::clamp(-k_ * error - d_ * error_dot, -effort_limit_, effort_limit_);

    (void)command_interfaces_[0].set_value(tau);
    (void)command_interfaces_[1].set_value(tau);

    return controller_interface::return_type::OK;
}

}
