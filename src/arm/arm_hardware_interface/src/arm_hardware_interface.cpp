#include "arm_hardware_interface/arm_hardware_interface.hpp"
#include "arm_hardware_interface/steadywin_protocol.hpp"
#include "arm_hardware_interface/damiao_wrist_protocol.hpp"

#include <cmath>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::ArmCanSystem,
    hardware_interface::SystemInterface)

namespace arm_hardware_interface {

// =============================================================================
// CALIBRATION PARAMETERS
// Update these arrays after finding the physical zero points of your arm.
// =============================================================================

// 1.0 = Normal rotation direction (matches URDF)
// -1.0 = Inverted rotation direction
//
// Array order MUST match URDF joint order (same as info.joints[i] / joint_names_[i]):
//   [0] arm_mount_base_joint
//   [1] arm_base_shoulder_joint
//   [2] arm_shoulder_forearm_joint
//   [3] arm_wrist_1_wrist_2_joint
//   [4] arm_forearm_wrist_1_joint
//   [5] arm_wrist_2_end_effector_joint
// NOTE: the previous version of this file had [3] and [4] swapped
// (values/comments for wrist_1_wrist_2 and forearm_wrist_1 were transposed).
// Fixed below.
const double JOINT_DIRECTIONS[NUM_JOINTS] = {
    -1.0, -1.0, 1.0, -1.0, 1.0, 1.0
};
 
// Offsets calculated based on your physical calibration data
const double JOINT_OFFSETS[NUM_JOINTS] = {
    -0.8437,   // arm_mount_base_joint
     0.6512,   // arm_base_shoulder_joint
     2.8576,   // arm_shoulder_forearm_joint
     2.2192,   // arm_forearm_wrist_1_joint
    -1.6767,   // arm_wrist_1_wrist_2_joint
    -3.1122    // arm_wrist_2_end_effector_joint
};


#ifdef JAZZY_OR_LATER
hardware_interface::CallbackReturn ArmCanSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto & info = params.hardware_info;
#else
hardware_interface::CallbackReturn ArmCanSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
#endif

    // Parse joint names from URDF (expected 6 joints)
    if (info.joints.size() != NUM_JOINTS) {
        RCLCPP_ERROR(logger_, "Expected %zu joints, got %zu", NUM_JOINTS, info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Read can_interface parameter from URDF
    if (info.hardware_parameters.count("can_interface") > 0) {
        can_interface_ = info.hardware_parameters.at("can_interface");
        RCLCPP_INFO(logger_, "Using CAN interface from URDF: %s", can_interface_.c_str());
    } else {
        can_interface_ = "can0"; // Fallback
        RCLCPP_WARN(logger_, "can_interface param missing in URDF, defaulting to can0");
    }

    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        joint_names_[i] = info.joints[i].name;
    }

    // Initialize all buffers to 0.0
    joint_position_command_.fill(0.0);
    joint_velocity_command_.fill(0.0);
    joint_position_state_.fill(0.0);
    joint_velocity_state_.fill(0.0);
    hw_position_states_.fill(0.0); // Initialize shadow buffer

    RCLCPP_INFO(logger_, "ArmCanSystem initialized successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmCanSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_state_[i]);
        state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmCanSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
        // Also exporting velocity to handle advanced trajectory features if needed
        command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ArmCanSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Configuring ArmCanSystem...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!open_can_socket()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Start background thread for reading CAN bus
    rx_running_.store(true);
    rx_thread_ = std::thread(&ArmCanSystem::rx_thread_fn, this);

    // Give the RX thread time to receive the initial state frames from all motors
    // before we synchronize the commands.
    RCLCPP_INFO(logger_, "Waiting for initial CAN frames to establish physical zero...");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Force an immediate read to populate joint_position_state_ with actual hardware values
    read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));

    // Synchronize initial command with actual hardware state (prevent startup jumps)
    for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
        joint_position_command_[i] = joint_position_state_[i];
    }

    send_enable_frames();
    motors_enabled_ = true;

    RCLCPP_INFO(logger_, "ArmCanSystem successfully activated. Motors enabled.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmCanSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    motors_enabled_ = false;
    send_disable_frames();

    // Stop RX thread safely
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }

    close_can_socket();

    RCLCPP_INFO(logger_, "ArmCanSystem deactivated. Motors disabled.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmCanSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Thread-safe copy from the background RX thread buffer to the state interface buffer
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    joint_position_state_ = hw_position_states_; 
    
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type ArmCanSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!motors_enabled_) return hardware_interface::return_type::OK;

    // Safety check: Prevent jumping to 0.0 if the controller sends an uninitialized command
    static bool first_write = true;
    if (first_write) {
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            // If the command is exactly 0.0 but the physical state is not, override the command
            if (joint_position_command_[i] == 0.0 && std::abs(joint_position_state_[i]) > 0.01) {
                joint_position_command_[i] = joint_position_state_[i];
            }
        }
        first_write = false;
    }

    /*
     * ======================= CONTROL GAINS =======================
     */
    const float KP_STEADYWIN[3] = {0.0f, 0.0f, 0.0f}; 
    const float KD_STEADYWIN[3] = {0.0f, 0.0f, 0.0f}; 

    const float KP_DAMIAO[3]    = {0.0f, 0.0f, 0.0f}; 
    const float KD_DAMIAO[3]    = {0.0f, 0.0f, 0.0f}; 

    std::lock_guard<std::mutex> lock(can_tx_mutex_);

    // 1. Send commands to Steadywin (Base, Shoulder, Elbow -> indices 0, 1, 2)
    for (std::size_t i = 0; i < 3; ++i) {
        float target_pos = static_cast<float>((joint_position_command_[i] * JOINT_DIRECTIONS[i]) + JOINT_OFFSETS[i]);
        float target_vel = static_cast<float>(joint_velocity_command_[i] * JOINT_DIRECTIONS[i]);

        can_msgs::msg::Frame f = steadywin_protocol::build_mit_command_frame(
            motor_ids_[i], 
            target_pos, 
            target_vel, 
            KP_STEADYWIN[i], KD_STEADYWIN[i], 0.0f);
        
        send_can_frame(f.id, f.data, f.dlc);
    }

    // 2. Send commands to Damiao Wrist (Indices 3, 4, 5)
    for (std::size_t i = 3; i < 6; ++i) {
        float target_pos = static_cast<float>((joint_position_command_[i] * JOINT_DIRECTIONS[i]) + JOINT_OFFSETS[i]);
        float target_vel = static_cast<float>(joint_velocity_command_[i] * JOINT_DIRECTIONS[i]);

        can_msgs::msg::Frame f = damiao_wrist_protocol::build_mit_command_frame(
            motor_ids_[i], 
            target_pos, 
            target_vel, 
            KP_DAMIAO[i - 3], KD_DAMIAO[i - 3], 0.0f);
        
        send_can_frame(f.id, f.data, f.dlc);
    }

    return hardware_interface::return_type::OK;
}


// -----------------------------------------------------------------------------
// SocketCAN Internals
// -----------------------------------------------------------------------------

bool ArmCanSystem::open_can_socket()
{
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to open CAN socket: %s", std::strerror(errno));
        return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "ioctl failed for %s: %s", can_interface_.c_str(), std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "CAN bind failed: %s", std::strerror(errno));
        close(can_fd_); can_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(logger_, "SocketCAN interface %s opened successfully.", can_interface_.c_str());
    return true;
}

void ArmCanSystem::close_can_socket()
{
    if (can_fd_ >= 0) {
        close(can_fd_);
        can_fd_ = -1;
    }
}

bool ArmCanSystem::send_can_frame(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t dlc)
{
    if (can_fd_ < 0) return false;

    struct can_frame frame{};
    frame.can_id = id; // Eff flag not needed for these particular IDs
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data.data(), dlc);

    if (::write(can_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        return false;
    }
    return true;
}

void ArmCanSystem::send_enable_frames()
{
    std::lock_guard<std::mutex> lock(can_tx_mutex_);
    
    // Enable Steadywin (requires limits config before enabling)
    uint16_t sw_pos = 955;  // 95.5 * 10
    uint16_t sw_vel = 4500; // 45.0 * 100
    uint16_t sw_tmx = 1800; // 18.0 * 100
    
    for (std::size_t i = 0; i < 3; ++i) {
        struct can_frame f{};
        f.can_id = 0x100 | motor_ids_[i];
        f.can_dlc = 7;
        f.data[0] = 0xF0;
        f.data[1] = sw_pos & 0xFF; f.data[2] = (sw_pos >> 8) & 0xFF;
        f.data[3] = sw_vel & 0xFF; f.data[4] = (sw_vel >> 8) & 0xFF;
        f.data[5] = sw_tmx & 0xFF; f.data[6] = (sw_tmx >> 8) & 0xFF;
        ::write(can_fd_, &f, sizeof(f));
    }

    // Enable Damiao
    for (std::size_t i = 3; i < 6; ++i) {
        auto f = damiao_wrist_protocol::build_enable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
}

void ArmCanSystem::send_disable_frames()
{
    std::lock_guard<std::mutex> lock(can_tx_mutex_);
    
    for (std::size_t i = 0; i < 3; ++i) {
        auto f = steadywin_protocol::build_disable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
    
    for (std::size_t i = 3; i < 6; ++i) {
        auto f = damiao_wrist_protocol::build_disable_frame(motor_ids_[i]);
        send_can_frame(f.id, f.data, f.dlc);
    }
}

void ArmCanSystem::rx_thread_fn()
{
    struct can_frame frame{};
    while (rx_running_.load()) {
        ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EINTR) continue;
            if (!rx_running_.load()) break;
            continue;
        }
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) continue;

        uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
        std::lock_guard<std::mutex> lock(feedback_mutex_);

        // Parse Steadywin (IDs 20, 21, 22)
        if (raw_id >= 20 && raw_id <= 22) {
            uint16_t pos_uint = (frame.data[1] << 8) | frame.data[2];
            float pos_rad = steadywin_protocol::uint_to_float(
                pos_uint, -steadywin_protocol::P_MAX_RAD, steadywin_protocol::P_MAX_RAD, 16);
            
            size_t idx = raw_id - 20;
            // Clean physical motor position back to URDF/MoveIt coordinates
            hw_position_states_[idx] = (static_cast<double>(pos_rad) - JOINT_OFFSETS[idx]) * JOINT_DIRECTIONS[idx];
        }
        // Parse Damiao (IDs 23, 24, 25 or 0)
        else if ((raw_id >= 23 && raw_id <= 25) || raw_id == 0) {
            uint8_t mid = raw_id;
            
            // Damiao can reply on ID 0 with the actual motor ID mod 16 in the first byte
            if (raw_id == 0) {
                uint8_t mod = frame.data[0] & 0x0F;
                for (uint8_t dm_id = 23; dm_id <= 25; ++dm_id) {
                    if (dm_id % 16 == mod) { mid = dm_id; break; }
                }
            }
            
            if (mid >= 23 && mid <= 25) {
                uint16_t pos_uint = (frame.data[1] << 8) | frame.data[2];
                float pos_rad = damiao_wrist_protocol::uint_to_float(
                    pos_uint, -damiao_wrist_protocol::P_MAX_RAD, damiao_wrist_protocol::P_MAX_RAD, 16);
                
                size_t idx = mid - 20;
                // Clean physical motor position back to URDF/MoveIt coordinates
                hw_position_states_[idx] = (static_cast<double>(pos_rad) - JOINT_OFFSETS[idx]) * JOINT_DIRECTIONS[idx];
            }
        }
    }
}

} // namespace arm_hardware_interface