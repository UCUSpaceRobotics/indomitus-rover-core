#include <algorithm>
#include <optional>
#include "rover_sim/rocker_diff_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rover_sim::RockerDiffController,
  controller_interface::ControllerInterface
)

namespace {

template<typename InterfaceVec>
std::optional<std::size_t> find_interface_index(const InterfaceVec& vec, const std::string& name) {
    for (std::size_t i = 0; i < vec.size(); i++) {
        if (vec[i].get_name() == name)
            return i;
    }
    return std::nullopt;
}

} // namespace

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

controller_interface::CallbackReturn RockerDiffController::on_configure(const rclcpp_lifecycle::State&) {
    k_ = get_node()->get_parameter("k_stiffness").as_double();
    d_ = get_node()->get_parameter("d_damping").as_double();
    effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerDiffController::on_activate(const rclcpp_lifecycle::State&) {
    auto l_pos = find_interface_index(state_interfaces_, "l_rocker_joint/position");
    auto l_vel = find_interface_index(state_interfaces_, "l_rocker_joint/velocity");
    auto r_pos = find_interface_index(state_interfaces_, "r_rocker_joint/position");
    auto r_vel = find_interface_index(state_interfaces_, "r_rocker_joint/velocity");
    auto l_cmd = find_interface_index(command_interfaces_, "l_rocker_joint/effort");
    auto r_cmd = find_interface_index(command_interfaces_, "r_rocker_joint/effort");

    if (!l_pos || !l_vel || !r_pos || !r_vel || !l_cmd || !r_cmd) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "RockerDiffController: failed to resolve one or more required interfaces "
            "by name during activation.");
        return controller_interface::CallbackReturn::ERROR;
    }

    idx_l_pos_ = *l_pos;
    idx_l_vel_ = *l_vel;
    idx_r_pos_ = *r_pos;
    idx_r_vel_ = *r_vel;
    idx_l_cmd_ = *l_cmd;
    idx_r_cmd_ = *r_cmd;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RockerDiffController::update(
    const rclcpp::Time &, const rclcpp::Duration &) {
#if defined(JAZZY_OR_LATER)
    double l_pos = state_interfaces_[idx_l_pos_].get_optional().value_or(0.0);
    double l_vel = state_interfaces_[idx_l_vel_].get_optional().value_or(0.0);
    double r_pos = state_interfaces_[idx_r_pos_].get_optional().value_or(0.0);
    double r_vel = state_interfaces_[idx_r_vel_].get_optional().value_or(0.0);
#else
    double l_pos = state_interfaces_[idx_l_pos_].get_value();
    double l_vel = state_interfaces_[idx_l_vel_].get_value();
    double r_pos = state_interfaces_[idx_r_pos_].get_value();
    double r_vel = state_interfaces_[idx_r_vel_].get_value();
#endif

    double error = l_pos + r_pos;
    double error_dot = l_vel + r_vel;

    double tau = std::clamp(-k_ * error - d_ * error_dot, -effort_limit_, effort_limit_);

    (void)command_interfaces_[idx_l_cmd_].set_value(tau);
    (void)command_interfaces_[idx_r_cmd_].set_value(tau);

    return controller_interface::return_type::OK;
}

}
