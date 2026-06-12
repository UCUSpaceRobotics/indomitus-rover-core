#include "rover_hardware_interface/chassis_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace chassis_driver {

ChassisDriverNode::ChassisDriverNode(const rclcpp::NodeOptions& options)
: Node("chassis_driver", options)
{
    // --- Parameters ---
    // steer_ids → Steadywin rotation motors, drive_ids → Damiao drive motors
    declare_parameter("steer_ids",          std::vector<int64_t>{11, 13, 17, 15});
    declare_parameter("drive_ids",          std::vector<int64_t>{10, 12, 16, 14});
    declare_parameter("drive_pmax",         12.5);
    declare_parameter("drive_vmax",         50.0);
    declare_parameter("drive_tmax",         20.0);
    declare_parameter("mst_id",             0);
    declare_parameter("steer_angle_min",    -1.5707963);  // -π/2 rad = -90°
    declare_parameter("steer_angle_max",     1.5707963);  //  π/2 rad =  90°
    declare_parameter("steer_joint_names",  std::vector<std::string>{
        "fl_wheel_mount_joint", "fr_wheel_mount_joint",
        "bl_wheel_mount_joint", "br_wheel_mount_joint"});
    declare_parameter("drive_joint_names",  std::vector<std::string>{
        "fl_wheel_joint", "fr_wheel_joint",
        "bl_wheel_joint", "br_wheel_joint"});

    auto steer_ids_param = get_parameter("steer_ids").as_integer_array();
    auto drive_ids_param = get_parameter("drive_ids").as_integer_array();

    // Validate parameter array sizes (prevent OOB crash from misconfigured YAML)
    if (steer_ids_param.size() < 4) {
        throw std::invalid_argument(
            "Parameter 'steer_ids' must have at least 4 entries (FL, FR, BL, BR), got " +
            std::to_string(steer_ids_param.size()));
    }
    if (drive_ids_param.size() < 4) {
        throw std::invalid_argument(
            "Parameter 'drive_ids' must have at least 4 entries (FL, FR, BL, BR), got " +
            std::to_string(drive_ids_param.size()));
    }

    for (int i = 0; i < 4; i++) {
        if (steer_ids_param[i] < 0 || steer_ids_param[i] > 255) {
            throw std::invalid_argument(
                "steer_ids[" + std::to_string(i) + "] = " +
                std::to_string(steer_ids_param[i]) + " is out of range 0..255"
            );
        }
        if (drive_ids_param[i] < 0 || drive_ids_param[i] > 255) {
            throw std::invalid_argument(
                "drive_ids[" + std::to_string(i) + "] = " + 
                std::to_string(drive_ids_param[i]) + " is out of range 0..255"
            );
        }

        steer_ids_[i] = static_cast<uint8_t>(steer_ids_param[i]);
        drive_ids_[i] = static_cast<uint8_t>(drive_ids_param[i]);
    }

    drive_pmax_       = static_cast<float>(get_parameter("drive_pmax").as_double());
    drive_vmax_       = static_cast<float>(get_parameter("drive_vmax").as_double());
    drive_tmax_       = static_cast<float>(get_parameter("drive_tmax").as_double());
    mst_id_           = static_cast<uint32_t>(get_parameter("mst_id").as_int());
    steer_angle_min_  = static_cast<float>(get_parameter("steer_angle_min").as_double());
    steer_angle_max_  = static_cast<float>(get_parameter("steer_angle_max").as_double());
    steer_joint_names_ = get_parameter("steer_joint_names").as_string_array();
    drive_joint_names_ = get_parameter("drive_joint_names").as_string_array();

    // Validate Damiao feedback decoding parameters (prevent divide-by-zero in fixed-point conversion)
    if (drive_pmax_ <= 0.0f || drive_vmax_ <= 0.0f || drive_tmax_ <= 0.0f) {
        throw std::invalid_argument(
            "Damiao feedback parameters must be positive: "
            "drive_pmax=" + std::to_string(drive_pmax_) + ", "
            "drive_vmax=" + std::to_string(drive_vmax_) + ", "
            "drive_tmax=" + std::to_string(drive_tmax_));
    }

    // --- Services ---
    set_steer_zero_srv_ = create_service<indomitus_interfaces::srv::SetSteerZero>(
        "~/set_steer_zero",
        std::bind(&ChassisDriverNode::onSetSteerZero, this,
                  std::placeholders::_1, std::placeholders::_2));

    // --- Publishers ---
    to_can_pub_          = create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
    joint_states_pub_    = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    chassis_status_pub_  = create_publisher<indomitus_interfaces::msg::ChassisStatus>(
                               "/chassis/motor_states", 10);
    diagnostics_pub_     = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // --- Subscriptions ---
    wheel_targets_sub_ = create_subscription<indomitus_interfaces::msg::WheelTargets>(
        "/wheel_targets", 10,
        std::bind(&ChassisDriverNode::onWheelTargets, this, std::placeholders::_1));


    // ====== WATCHDOG PART ======
    last_wheel_targets_time_ = now();

    // Watchdog: if /wheel_targets stops arriving, zero all commands
    watchdog_timer_ = create_wall_timer(100ms, [this]() {
        if (!motors_enabled_) return;
        if ((now() - last_wheel_targets_time_).seconds() < kWheelTargetsTimeoutSec) return;

        for (int i = 0; i < 4; i++) {
            to_can_pub_->publish(steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f));
            to_can_pub_->publish(damiao_protocol::buildVelocityFrame(drive_ids_[i], 0.0f));
        }
    });
    // ====== WATCHDOG PART ======

    from_can_sub_ = create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 10,
        std::bind(&ChassisDriverNode::onCanFrame, this, std::placeholders::_1));

    motor_enable_service_ = create_service<std_srvs::srv::SetBool>(
        "/chassis/set_motors_enabled",
        std::bind(
            &ChassisDriverNode::onSetMotorsEnabled,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // --- Timers ---


    // --- Boot disable handling ---
    // Start in a known safe state: attempt to publish disable frames, but if
    // the CAN bridge isn't yet subscribed to `/to_can_bus` retry for a short
    // window to avoid losing the safety-critical disable frames.
    tryPublishBootDisable();

    // 1 Hz: poll all motors for status feedback
    status_poll_timer_ = create_wall_timer(1s, [this]() {
        if (!motors_enabled_) return;
        // Steadywin steer: 0xAE status (voltage/temp/fault) + 0xA3 position (multi-turn counts)
        for (int i = 0; i < 4; i++)
            to_can_pub_->publish(steadywin_protocol::buildStatusQueryFrame(steer_ids_[i]));
        for (int i = 0; i < 4; i++)
            to_can_pub_->publish(steadywin_protocol::buildAbsAngleQueryFrame(steer_ids_[i]));

        // Damiao drive: read register 80 (p_m, motor position float) — also triggers MIT feedback
        for (int i = 0; i < 4; i++)
            to_can_pub_->publish(damiao_protocol::buildReadRegisterFrame(drive_ids_[i], 80));
    });

    // 10 Hz: publish /chassis/motor_states
    chassis_status_timer_ = create_wall_timer(100ms,
        std::bind(&ChassisDriverNode::publishChassisStatus, this));

    // 1 Hz: publish /diagnostics
    diagnostics_timer_ = create_wall_timer(1s,
        std::bind(&ChassisDriverNode::publishDiagnostics, this));

    RCLCPP_INFO(get_logger(),
        "chassis_driver started — "
        "Steadywin steer [%d,%d,%d,%d]  Damiao drive [%d,%d,%d,%d]",
        steer_ids_[0], steer_ids_[1], steer_ids_[2], steer_ids_[3],
        drive_ids_[0], drive_ids_[1], drive_ids_[2], drive_ids_[3]);
}

void ChassisDriverNode::tryPublishBootDisable() {
    // If there's already a subscriber on /to_can_bus, publish immediately.
    if (to_can_pub_->get_subscription_count() > 0u) {
        sendDisableFrames();
        return;
    }

    // Otherwise, start a retry timer (non-blocking) that fires every 200ms
    // and publishes once a subscriber appears. Give up after boot_retry_max_attempts_.
    boot_retry_attempts_ = 0;
    boot_retry_timer_ = create_wall_timer(200ms, [this]() {
        if (to_can_pub_->get_subscription_count() > 0u) {
            RCLCPP_INFO(this->get_logger(), "/to_can_bus subscriber detected — publishing boot disable frames");
            sendDisableFrames();
            boot_retry_timer_->cancel();
            return;
        }

        ++boot_retry_attempts_;
        if (boot_retry_attempts_ >= boot_retry_max_attempts_) {
            RCLCPP_WARN(get_logger(), "No /to_can_bus subscriber after %d attempts (~%.1fs); initial disable frames may have been lost",
                boot_retry_attempts_, (boot_retry_attempts_ * 0.2));
            boot_retry_timer_->cancel();
        }
    });
}

// ── Hardware lifecycle ────────────────────────────────────────────────────────

void ChassisDriverNode::sendEnableFrames() {
    RCLCPP_INFO(get_logger(), "Motor init: enabling all chassis motors");

    // Steadywin steer: clear fault → position=0 to activate
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(steadywin_protocol::buildClearFaultFrame(steer_ids_[i]));
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f));

    // Damiao drive: set TIMEOUT watchdog (reg 9, 200ms) → setMode(3=velocity) → enable(0xFC)
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(damiao_protocol::buildWriteRegisterUint32Frame(drive_ids_[i], 9, 200u));
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(damiao_protocol::buildSetModeFrame(drive_ids_[i], 3));
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(damiao_protocol::buildEnableFrame(drive_ids_[i]));

    motors_enabled_ = true;
    RCLCPP_INFO(get_logger(), "All motors enabled");
}

void ChassisDriverNode::sendDisableFrames() {
    RCLCPP_INFO(get_logger(), "Motor command: disabling all chassis motors");

    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(steadywin_protocol::buildDisableFrame(steer_ids_[i]));
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(damiao_protocol::buildDisableFrame(drive_ids_[i]));

    motors_enabled_ = false;
    RCLCPP_INFO(get_logger(), "All motors disabled");
}

// ── Service handlers ──────────────────────────────────────────────────────────

void ChassisDriverNode::onSetSteerZero(
    const indomitus_interfaces::srv::SetSteerZero::Request::SharedPtr req,
    indomitus_interfaces::srv::SetSteerZero::Response::SharedPtr res)
{
    if (!motors_enabled_) {
        res->success = false;
        res->message = "Motors not enabled — cannot set zero";
        return;
    }

    static const char* names[4] = {"FL", "FR", "RL", "RR"};

    // Empty list → zero all steering motors
    bool zero_all = req->motor_ids.empty();

    std::string zeroed;
    std::string unknown;

    for (const uint8_t req_id : req->motor_ids) {
        bool found = false;
        for (int i = 0; i < 4; i++) {
            if (steer_ids_[i] == req_id) {
                to_can_pub_->publish(steadywin_protocol::buildSetOriginFrame(steer_ids_[i]));
                zeroed += names[i];
                zeroed += '(';
                zeroed += std::to_string(steer_ids_[i]);
                zeroed += ") ";
                RCLCPP_INFO(get_logger(),
                    "Set steer zero: motor %s (id=%d)", names[i], steer_ids_[i]);
                found = true;
                break;
            }
        }
        if (!found) {
            unknown += std::to_string(req_id);
            unknown += ' ';
        }
    }

    if (zero_all) {
        for (int i = 0; i < 4; i++) {
            to_can_pub_->publish(steadywin_protocol::buildSetOriginFrame(steer_ids_[i]));
            zeroed += names[i];
            zeroed += '(';
            zeroed += std::to_string(steer_ids_[i]);
            zeroed += ") ";
            RCLCPP_INFO(get_logger(),
                "Set steer zero: motor %s (id=%d)", names[i], steer_ids_[i]);
        }
    }

    if (!unknown.empty()) {
        res->success = false;
        res->message = "Unknown steer IDs: " + unknown +
                       "— valid IDs: " +
                       std::to_string(steer_ids_[0]) + " " +
                       std::to_string(steer_ids_[1]) + " " +
                       std::to_string(steer_ids_[2]) + " " +
                       std::to_string(steer_ids_[3]);
        return;
    }

    res->success = true;
    res->message = "Origin set for: " + zeroed;
}


void ChassisDriverNode::sendShutdownFrames() {
    if (!motors_enabled_) {
        sendDisableFrames();
        return;
    }

    RCLCPP_INFO(get_logger(), "Motor shutdown: zeroing commands");

    // Send zero/home targets before disabling so the rover stops cleanly.
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], 0.0f));
    for (int i = 0; i < 4; i++)
        to_can_pub_->publish(damiao_protocol::buildVelocityFrame(drive_ids_[i], 0.0f));

    RCLCPP_INFO(get_logger(), "Waiting 1.5s for motion to settle...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    sendDisableFrames();
}

void ChassisDriverNode::onSetMotorsEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        sendEnableFrames();
        response->success = true;
        response->message = "All chassis motors enabled";
        return;
    }

    sendDisableFrames();
    response->success = true;
    response->message = "All chassis motors disabled";
}

// ── Command handling ──────────────────────────────────────────────────────────

void ChassisDriverNode::onWheelTargets(const indomitus_interfaces::msg::WheelTargets::SharedPtr msg) {
    last_wheel_targets_time_ = now();

    if (!motors_enabled_) return;

    const float raw_angles[4] = {msg->fl_angle, msg->fr_angle, msg->rl_angle, msg->rr_angle};
    const float speeds[4]     = {-msg->fl_speed, msg->fr_speed, -msg->rl_speed, msg->rr_speed};

    for (int i = 0; i < 4; i++) {
        float angle = raw_angles[i];

        // Steadywin steer: 0xC2 absolute position (rad), clamped to [steer_angle_min_, steer_angle_max_]
        to_can_pub_->publish(
            steadywin_protocol::buildAbsPositionFrame(steer_ids_[i], angle));

        // Damiao drive: float velocity (rad/s) — firmware handles gear reduction internally
        to_can_pub_->publish(
            damiao_protocol::buildVelocityFrame(drive_ids_[i], speeds[i]));
    }
}

// ── Feedback handling ─────────────────────────────────────────────────────────

void ChassisDriverNode::onCanFrame(const can_msgs::msg::Frame::SharedPtr msg) {
    // Damiao drive: feedback uses CAN Message ID (frame header) as the primary motor identifier.
    // Each motor sends feedback with CAN ID = its ESC_ID or MST_ID (Register 7, default 0).
    // Route by CAN ID: check if frame came from mst_id or any drive motor's ESC_ID.
    // This works for any motor ID (0-255); the 4-bit payload ID field is just a sanity check.
    int drive_index = -1;
    for (int i = 0; i < 4; i++) {
        if (msg->id == drive_ids_[i]) {
            drive_index = i;
            break;
        }
    }

    if (drive_index >= 0) {
        damiao_protocol::parseFeedback(msg->data, msg->dlc, drive_ids_[drive_index],
            drive_pmax_, drive_vmax_, drive_tmax_, drive_state_[drive_index]);
        damiao_protocol::parseRegisterResponse(msg->data, msg->dlc, drive_ids_[drive_index],
            drive_state_[drive_index]);
        publishJointStates();
        return;
    }

    if (msg->id == mst_id_) {
        for (int i = 0; i < 4; i++) {
            if (damiao_protocol::parseFeedback(msg->data, msg->dlc, drive_ids_[i],
                    drive_pmax_, drive_vmax_, drive_tmax_, drive_state_[i])) {
                break;
            }
        }
        if (msg->dlc >= 8 && msg->data[2] == 0x33) {
            for (int i = 0; i < 4; i++) {
                if (damiao_protocol::parseRegisterResponse(msg->data, msg->dlc, drive_ids_[i], drive_state_[i])) {
                    break;
                }
            }
        }
        publishJointStates();
        return;
    }

    // Steadywin steer: 0xAE status and 0xA3/0xC2 position responses at motor's own address
    for (int i = 0; i < 4; i++) {
        if (msg->id == steer_ids_[i] || msg->id == (0x100u | steer_ids_[i])) {
            steadywin_protocol::parseResponse(msg->data, msg->dlc, steer_state_[i]);
            publishJointStates();
            return;
        }
    }
}

// ── Publishers ────────────────────────────────────────────────────────────────

void ChassisDriverNode::publishJointStates() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();

    for (int i = 0; i < 4; i++) {
        js.name.push_back(steer_joint_names_[i]);
        js.position.push_back(steer_state_[i].pos_valid ? steer_state_[i].pos_rad : 0.0);
        js.velocity.push_back(0.0);
        js.effort.push_back(0.0);
    }

    for (int i = 0; i < 4; i++) {
        js.name.push_back(drive_joint_names_[i]);
        js.position.push_back(drive_state_[i].valid ? drive_state_[i].pos : 0.0);
        js.velocity.push_back(drive_state_[i].valid ? drive_state_[i].vel : 0.0);
        js.effort.push_back(drive_state_[i].valid ? drive_state_[i].tor : 0.0);
    }

    joint_states_pub_->publish(js);
}

void ChassisDriverNode::publishChassisStatus() {
    // static const char* wheel_names[4] = {"FL", "FR", "RL", "RR"};

    indomitus_interfaces::msg::ChassisStatus msg;
    msg.header.stamp = now();

    // Steadywin steer motors
    for (int i = 0; i < 4; i++) {
        const auto& s = steer_state_[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id     = steer_ids_[i];
        m.motor_type = "steadywin";
        m.joint_name = steer_joint_names_[i];
        // std::string label = std::string(wheel_names[i]) + "_steer";
        // (void)label;

        m.position         = s.pos_valid ? s.pos_rad : 0.0f;
        m.velocity         = 0.0f;
        m.torque           = 0.0f;
        m.kinematic_valid  = s.pos_valid;

        m.voltage          = s.voltage;
        m.current          = s.bus_current;
        m.temperature      = static_cast<float>(s.temperature);
        m.mode             = s.mode;
        m.fault_code       = s.fault_code;
        m.health_valid     = s.diag_valid;

        m.enabled          = motors_enabled_;

        msg.motors.push_back(m);
    }

    // Damiao drive motors
    for (int i = 0; i < 4; i++) {
        const auto& s = drive_state_[i];
        indomitus_interfaces::msg::MotorStatus m;
        m.esc_id     = drive_ids_[i];
        m.motor_type = "damiao";
        m.joint_name = drive_joint_names_[i];

        m.position         = s.valid ? s.pos : 0.0f;
        m.velocity         = s.valid ? s.vel : 0.0f;
        m.torque           = s.valid ? s.tor : 0.0f;
        m.kinematic_valid  = s.valid;

        m.voltage          = 0.0f;
        m.current          = 0.0f;
        m.temperature      = static_cast<float>(s.t_mos);
        m.mode             = s.valid ? 3u : 0u;  // 3 = speed mode
        m.fault_code       = (s.valid && s.err != 0x1) ? 0x01u : 0x00u;
        m.health_valid     = s.valid;

        m.enabled          = motors_enabled_ && s.valid && s.err == 0x1;

        msg.motors.push_back(m);
    }

    chassis_status_pub_->publish(msg);
}

void ChassisDriverNode::publishDiagnostics() {
    static const char* wheel_names[4] = {"FL", "FR", "RL", "RR"};

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now();

    auto kv = [](const std::string& k, const std::string& v) {
        diagnostic_msgs::msg::KeyValue p;
        p.key = k; p.value = v;
        return p;
    };

    for (int i = 0; i < 4; i++) {
        const auto& s = steer_state_[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = std::string("steadywin/steer_") + wheel_names[i];
        st.hardware_id = st.name;

        if (!s.diag_valid) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "No status received";
        } else if (s.fault_code != 0) {
            st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            std::string faults;
            if (s.fault_code & 0x01) faults += "voltage ";
            if (s.fault_code & 0x02) faults += "current ";
            if (s.fault_code & 0x04) faults += "temperature ";
            if (s.fault_code & 0x08) faults += "encoder ";
            if (s.fault_code & 0x40) faults += "hardware ";
            if (s.fault_code & 0x80) faults += "software ";
            st.message = "FAULT: " + faults;
        } else {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = "OK";
        }

        st.values.push_back(kv("pos_rad",        std::to_string(s.pos_rad)));
        st.values.push_back(kv("voltage_V",      std::to_string(s.voltage)));
        st.values.push_back(kv("current_A",      std::to_string(s.bus_current)));
        st.values.push_back(kv("temperature_C",  std::to_string(s.temperature)));
        st.values.push_back(kv("mode",           std::to_string(s.mode)));
        st.values.push_back(kv("fault_code",     std::to_string(s.fault_code)));
        diag_array.status.push_back(st);
    }

    for (int i = 0; i < 4; i++) {
        const auto& s = drive_state_[i];
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name        = std::string("damiao/drive_") + wheel_names[i];
        st.hardware_id = st.name;

        if (!s.valid) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "No feedback received";
        } else if (s.err == 0x1) {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
            st.message = "Enabled";
        } else {
            st.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            st.message = "Disabled or fault";
        }

        st.values.push_back(kv("pos_rad",    std::to_string(s.pos)));
        st.values.push_back(kv("vel_rad_s",  std::to_string(s.vel)));
        st.values.push_back(kv("tor_Nm",     std::to_string(s.tor)));
        st.values.push_back(kv("t_mos_C",   std::to_string(s.t_mos)));
        st.values.push_back(kv("t_rotor_C", std::to_string(s.t_rotor)));
        st.values.push_back(kv("err_code",  std::to_string(s.err)));
        diag_array.status.push_back(st);
    }

    diagnostics_pub_->publish(diag_array);
}

} // namespace chassis_driver

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<chassis_driver::ChassisDriverNode>();
    rclcpp::spin(node);
    // spin() returns on SIGINT — perform graceful motor shutdown before exit
    node->sendShutdownFrames();
    rclcpp::shutdown();
    return 0;
}
