#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace arm_hardware_interface {
namespace damiao_wrist_protocol {

// Hardware limits for Damiao DM-J4340-2EC (Wrist IDs 23, 24, 25)
constexpr float P_MAX_RAD = 12.5f;
constexpr float V_MAX_RPS = 45.0f;
constexpr float T_MAX_NM  = 18.0f;

/**
 * @brief Converts a float value to a scaled unsigned integer based on hardware limits.
 */
inline uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = (1 << bits) - 1.0f;
    float clamped_x = std::clamp(x, x_min, x_max);
    return static_cast<uint16_t>((clamped_x - x_min) * span / (x_max - x_min));
}

/**
 * @brief Converts a received unsigned integer back to a float value.
 */
inline float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
    float span = (1 << bits) - 1.0f;
    return (static_cast<float>(x_int) * (x_max - x_min) / span) + x_min;
}

/**
 * @brief Packs MIT mode control parameters into an 8-byte payload using Damiao limits.
 */
inline std::array<uint8_t, 8> pack_mit_8bytes(
    float pos_rad, float vel_rps, float kp, float kd, float t_ff)
{
    uint16_t p_int = float_to_uint(pos_rad, -P_MAX_RAD, P_MAX_RAD, 16);
    uint16_t v_int = float_to_uint(vel_rps, -V_MAX_RPS, V_MAX_RPS, 12);
    uint16_t k_int = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint16_t d_int = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint16_t t_int = float_to_uint(t_ff, -T_MAX_NM, T_MAX_NM, 12);

    return {{
        static_cast<uint8_t>((p_int >> 8) & 0xFF),
        static_cast<uint8_t>(p_int & 0xFF),
        static_cast<uint8_t>((v_int >> 4) & 0xFF),
        static_cast<uint8_t>(((v_int & 0xF) << 4) | ((k_int >> 8) & 0xF)),
        static_cast<uint8_t>(k_int & 0xFF),
        static_cast<uint8_t>((d_int >> 4) & 0xFF),
        static_cast<uint8_t>(((d_int & 0xF) << 4) | ((t_int >> 8) & 0xF)),
        static_cast<uint8_t>(t_int & 0xFF)
    }};
}

/**
 * @brief Builds the CAN frame to send MIT control commands to the Damiao motor.
 */
inline can_msgs::msg::Frame build_mit_command_frame(
    uint8_t motor_id, float pos_rad, float vel_rps, float kp, float kd, float t_ff)
{
    can_msgs::msg::Frame frame;
    // Damiao uses the direct motor ID for control
    frame.id = motor_id;
    frame.dlc = 8;
    frame.data = pack_mit_8bytes(pos_rad, vel_rps, kp, kd, t_ff);
    return frame;
}

/**
 * @brief Builds the CAN frame to enable the Damiao motor (Motor Run Mode).
 */
inline can_msgs::msg::Frame build_enable_frame(uint8_t motor_id)
{
    can_msgs::msg::Frame frame;
    frame.id = motor_id;
    frame.dlc = 8;
    frame.data.fill(0xFF);
    frame.data[7] = 0xFC;
    return frame;
}

/**
 * @brief Builds the CAN frame to disable the Damiao motor (Motor Stop Mode).
 */
inline can_msgs::msg::Frame build_disable_frame(uint8_t motor_id)
{
    can_msgs::msg::Frame frame;
    frame.id = motor_id;
    frame.dlc = 8;
    frame.data.fill(0xFF);
    frame.data[7] = 0xFD;
    return frame;
}

/**
 * @brief Builds the CAN frame to set the current position as zero.
 */
inline can_msgs::msg::Frame build_set_zero_frame(uint8_t motor_id)
{
    can_msgs::msg::Frame frame;
    frame.id = motor_id;
    frame.dlc = 8;
    frame.data.fill(0xFF);
    frame.data[7] = 0xFE;
    return frame;
}

} // namespace damiao_wrist_protocol
} // namespace arm_hardware_interface