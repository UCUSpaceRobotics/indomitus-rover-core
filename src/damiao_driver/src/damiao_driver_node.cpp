#include "damiao_driver/damiao_driver_node.hpp"

#include <chrono>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using namespace std::chrono_literals;

namespace damiao_driver {

DamiaoDriverNode::DamiaoDriverNode(const rclcpp::NodeOptions& options)
: Node("damiao_driver", options)
{
    // --- Parameters ---
    declare_parameter("steer_ids",         std::vector<int64_t>{1, 2, 3, 4});
    declare_parameter("drive_ids",         std::vector<int64_t>{5, 6, 7, 8});
    declare_parameter("steer_gear_ratio",  1.0);
    declare_parameter("drive_gear_ratio",  1.0);
    declare_parameter("steer_pmax",        12.5);
    declare_parameter("steer_vmax",        50.0);
    declare_parameter("steer_tmax",        10.0);
    declare_parameter("drive_pmax",        12.5);
    declare_parameter("drive_vmax",        50.0);
    declare_parameter("drive_tmax",        20.0);
    declare_parameter("steer_velocity_cap", 10.0);
    declare_parameter("mst_id",            0);
    declare_parameter("steer_joint_names", std::vector<std::string>{
        "fl_wheel_mount_joint", "fr_wheel_mount_joint",
        "bl_wheel_mount_joint", "br_wheel_mount_joint"});
    declare_parameter("drive_joint_names", std::vector<std::string>{
        "fl_wheel_joint", "fr_wheel_joint",
        "bl_wheel_joint", "br_wheel_joint"});

    auto steer_ids_param = get_parameter("steer_ids").as_integer_array();
    auto drive_ids_param = get_parameter("drive_ids").as_integer_array();
    for (int i = 0; i < 4; i++) {
        steer_ids_[i] = static_cast<uint8_t>(steer_ids_param[i]);
        drive_ids_[i] = static_cast<uint8_t>(drive_ids_param[i]);
    }

    steer_gear_ratio_  = static_cast<float>(get_parameter("steer_gear_ratio").as_double());
    drive_gear_ratio_  = static_cast<float>(get_parameter("drive_gear_ratio").as_double());
    steer_pmax_        = static_cast<float>(get_parameter("steer_pmax").as_double());
    steer_vmax_        = static_cast<float>(get_parameter("steer_vmax").as_double());
    steer_tmax_        = static_cast<float>(get_parameter("steer_tmax").as_double());
    drive_pmax_        = static_cast<float>(get_parameter("drive_pmax").as_double());
    drive_vmax_        = static_cast<float>(get_parameter("drive_vmax").as_double());
    drive_tmax_        = static_cast<float>(get_parameter("drive_tmax").as_double());
    steer_velocity_cap_ = static_cast<float>(get_parameter("steer_velocity_cap").as_double());
    mst_id_            = static_cast<uint32_t>(get_parameter("mst_id").as_int());
    steer_joint_names_ = get_parameter("steer_joint_names").as_string_array();
    drive_joint_names_ = get_parameter("drive_joint_names").as_string_array();

    // --- Publishers ---
    to_can_pub_      = create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    diagnostics_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // --- Subscriptions ---
    wheel_targets_sub_ = create_subscription<indomitus_msgs::msg::WheelTargets>(
        "/wheel_targets", 10,
        std::bind(&DamiaoDriverNode::onWheelTargets, this, std::placeholders::_1));

    from_can_sub_ = create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 10,
        std::bind(&DamiaoDriverNode::onCanFrame, this, std::placeholders::_1));

    // Send enable frames 500 ms after startup (one-shot)
    enable_timer_ = create_wall_timer(500ms, [this]() {
        sendEnableFrames();
        enable_timer_->cancel();
    });

    // Diagnostics at 1 Hz
    diagnostics_timer_ = create_wall_timer(1s,
        std::bind(&DamiaoDriverNode::publishDiagnostics, this));

    RCLCPP_INFO(get_logger(), "damiao_driver started — steer IDs [%d,%d,%d,%d], drive IDs [%d,%d,%d,%d]",
        steer_ids_[0], steer_ids_[1], steer_ids_[2], steer_ids_[3],
        drive_ids_[0], drive_ids_[1], drive_ids_[2], drive_ids_[3]);
}

void DamiaoDriverNode::sendEnableFrames() {
    // Set modes first (register 10): steer=PositionVelocity(2), drive=Velocity(3)
    for (int i = 0; i < 4; i++) {
        to_can_pub_->publish(damiao_protocol::buildSetModeFrame(steer_ids_[i], 2));
        to_can_pub_->publish(damiao_protocol::buildSetModeFrame(drive_ids_[i], 3));
    }
    // Then enable all motors
    for (int i = 0; i < 4; i++) {
        to_can_pub_->publish(damiao_protocol::buildEnableFrame(steer_ids_[i]));
        to_can_pub_->publish(damiao_protocol::buildEnableFrame(drive_ids_[i]));
    }
    RCLCPP_INFO(get_logger(), "Mode set and enable frames sent to all 8 motors");
}

void DamiaoDriverNode::onWheelTargets(const indomitus_msgs::msg::WheelTargets::SharedPtr msg) {
    const float angles[4] = {msg->fl_angle, msg->fr_angle, msg->rl_angle, msg->rr_angle};
    const float speeds[4] = {msg->fl_speed, msg->fr_speed, msg->rl_speed, msg->rr_speed};

    for (int i = 0; i < 4; i++) {
        to_can_pub_->publish(
            damiao_protocol::buildPositionVelocityFrame(
                steer_ids_[i], angles[i] * steer_gear_ratio_, steer_velocity_cap_));
        to_can_pub_->publish(
            damiao_protocol::buildVelocityFrame(
                drive_ids_[i], speeds[i] * drive_gear_ratio_));
    }
}

void DamiaoDriverNode::onCanFrame(const can_msgs::msg::Frame::SharedPtr msg) {
    if (msg->id != mst_id_) return;

    bool updated = false;

    for (int i = 0; i < 4; i++) {
        if (damiao_protocol::parseFeedback(
                msg->data, msg->dlc, steer_ids_[i],
                steer_pmax_, steer_vmax_, steer_tmax_, steer_state_[i])) {
            updated = true;
            break;
        }
    }

    if (!updated) {
        for (int i = 0; i < 4; i++) {
            if (damiao_protocol::parseFeedback(
                    msg->data, msg->dlc, drive_ids_[i],
                    drive_pmax_, drive_vmax_, drive_tmax_, drive_state_[i])) {
                break;
            }
        }
    }

    publishJointStates();
}

void DamiaoDriverNode::publishJointStates() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();

    for (int i = 0; i < 4; i++) {
        js.name.push_back(steer_joint_names_[i]);
        js.position.push_back(steer_state_[i].valid ? steer_state_[i].pos / steer_gear_ratio_ : 0.0);
        js.velocity.push_back(steer_state_[i].valid ? steer_state_[i].vel / steer_gear_ratio_ : 0.0);
        js.effort.push_back(steer_state_[i].valid ? steer_state_[i].tor : 0.0);
    }

    for (int i = 0; i < 4; i++) {
        js.name.push_back(drive_joint_names_[i]);
        js.position.push_back(drive_state_[i].valid ? drive_state_[i].pos / drive_gear_ratio_ : 0.0);
        js.velocity.push_back(drive_state_[i].valid ? drive_state_[i].vel / drive_gear_ratio_ : 0.0);
        js.effort.push_back(drive_state_[i].valid ? drive_state_[i].tor : 0.0);
    }

    joint_states_pub_->publish(js);
}

void DamiaoDriverNode::publishDiagnostics() {
    static const char* wheel_names[4] = {"FL", "FR", "RL", "RR"};

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now();

    auto make_status = [&](const char* label, const damiao_protocol::MotorState& s) {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name    = label;
        status.hardware_id = label;

        if (!s.valid) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "No feedback received";
        } else if (s.err == 0x1) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Enabled";
        } else if (s.err == 0x0) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Disabled";
        } else {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Motor fault";
        }

        auto kv = [](const std::string& k, const std::string& v) {
            diagnostic_msgs::msg::KeyValue p;
            p.key   = k;
            p.value = v;
            return p;
        };

        status.values.push_back(kv("pos_rad",  std::to_string(s.pos)));
        status.values.push_back(kv("vel_rad_s", std::to_string(s.vel)));
        status.values.push_back(kv("tor_Nm",   std::to_string(s.tor)));
        status.values.push_back(kv("t_mos_C",  std::to_string(s.t_mos)));
        status.values.push_back(kv("t_rotor_C", std::to_string(s.t_rotor)));
        status.values.push_back(kv("err_code",  std::to_string(s.err)));

        return status;
    };

    for (int i = 0; i < 4; i++) {
        std::string steer_label = std::string("damiao/steer_") + wheel_names[i];
        std::string drive_label = std::string("damiao/drive_") + wheel_names[i];
        diag_array.status.push_back(make_status(steer_label.c_str(), steer_state_[i]));
        diag_array.status.push_back(make_status(drive_label.c_str(), drive_state_[i]));
    }

    diagnostics_pub_->publish(diag_array);
}

} // namespace damiao_driver

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<damiao_driver::DamiaoDriverNode>());
    rclcpp::shutdown();
    return 0;
}
