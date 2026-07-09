#pragma once
#include "controller_interface/controller_interface.hpp"

namespace rover_sim {

class RockerDiffController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    double k_, d_, effort_limit_;

    std::size_t idx_l_pos_{0};
    std::size_t idx_l_vel_{0};
    std::size_t idx_r_pos_{0};
    std::size_t idx_r_vel_{0};
    std::size_t idx_l_cmd_{0};
    std::size_t idx_r_cmd_{0};
};

}