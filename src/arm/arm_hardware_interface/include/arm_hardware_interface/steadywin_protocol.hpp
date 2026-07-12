#pragma once
//
// Steadywin custom CAN protocol (自定义CAN通信协议 Rev.3.06b0) — MIT motion mode.
// Verified against the vendor PDF and against the proven-working Python tools
// (arm_control.py / play_arm_waypoints.py).
//
// Key facts from the manual:
//  * MIT command frame:  StdID = 0x400 | (0x100 | Dev_addr)   (bit10 set = MIT cmd)
//    payload = classic MIT packing: pos16 | vel12 | kp12 | kd12 | tff12
//    Receiving an MIT frame IMMEDIATELY switches the motor into motion-control
//    mode and executes it — there is no separate "enable". The first MIT frame
//    you send must therefore be harmless (kp≈0, kd≈0, tff=0, pos=measured).
//  * Feedback / reply frame: StdID = Dev_addr, DLC = 7,
//    data[0] = echoed command code (0xF1 for reads and MIT replies),
//    data[1..2] = pos16 (big-endian, maps to ±Pos_Max),
//    data[3..4hi] = vel12 (±Vel_Max), data[4lo..5] = torque12 (±T_Max),
//    data[6] = status (bit0: in MIT mode, bit1: fault).
//  * 0xF1 (DLC=1) reads pos/vel/torque WITHOUT changing mode — safe startup read.
//  * 0xF0 (DLC=7) configures Pos_Max (0.1 rad), Vel_Max (0.01 rad/s),
//    T_Max (0.01 Nm), little-endian, saved to ROM. Its reply carries those
//    values in data[1..6] — it must NOT be parsed as a position frame!
//  * 0xCF disables the motor (free / coast). 0xB1 saves current position as
//    origin in ROM (do this only during calibration, arm held at reference pose).
//
#include <algorithm>
#include <array>
#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace arm_hardware_interface {
namespace steadywin_protocol {

// These MUST match what is configured in the motor via the 0xF0 command.
// Factory defaults are 95.5 rad / 45.00 rad/s / 18.00 Nm — we re-send them
// on every activation so the scaling is always consistent.
constexpr float P_MAX_RAD = 95.5f;
constexpr float V_MAX_RPS = 45.0f;
constexpr float T_MAX_NM  = 18.0f;

constexpr float KP_MAX = 500.0f;
constexpr float KD_MAX = 5.0f;

// Reply command codes we may see with the motor's Dev_addr as StdID.
constexpr uint8_t CMD_READ_STATE = 0xF1;  // also echoed in MIT-command replies
constexpr uint8_t CMD_CONFIG_MAX = 0xF0;  // reply payload = Pos/Vel/T max, NOT position!
constexpr uint8_t CMD_SET_ZERO   = 0xB1;  // reply payload = mechanical angle offset
constexpr uint8_t CMD_DISABLE    = 0xCF;

inline uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    const float span = static_cast<float>((1 << bits) - 1);
    const float clamped = std::clamp(x, x_min, x_max);
    return static_cast<uint16_t>((clamped - x_min) * span / (x_max - x_min));
}

inline float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
    const float span = static_cast<float>((1 << bits) - 1);
    return static_cast<float>(x_int) * (x_max - x_min) / span + x_min;
}

inline std::array<uint8_t, 8> pack_mit_8bytes(
    float pos_rad, float vel_rps, float kp, float kd, float t_ff)
{
    const uint16_t p = float_to_uint(pos_rad, -P_MAX_RAD, P_MAX_RAD, 16);
    const uint16_t v = float_to_uint(vel_rps, -V_MAX_RPS, V_MAX_RPS, 12);
    const uint16_t k = float_to_uint(kp, 0.0f, KP_MAX, 12);
    const uint16_t d = float_to_uint(kd, 0.0f, KD_MAX, 12);
    const uint16_t t = float_to_uint(t_ff, -T_MAX_NM, T_MAX_NM, 12);
    return {{
        static_cast<uint8_t>((p >> 8) & 0xFF),
        static_cast<uint8_t>(p & 0xFF),
        static_cast<uint8_t>((v >> 4) & 0xFF),
        static_cast<uint8_t>(((v & 0xF) << 4) | ((k >> 8) & 0xF)),
        static_cast<uint8_t>(k & 0xFF),
        static_cast<uint8_t>((d >> 4) & 0xFF),
        static_cast<uint8_t>(((d & 0xF) << 4) | ((t >> 8) & 0xF)),
        static_cast<uint8_t>(t & 0xFF)
    }};
}

/// MIT motion command. Sending this puts the motor in MIT mode and executes it.
inline can_msgs::msg::Frame build_mit_command_frame(
    uint8_t motor_id, float pos_rad, float vel_rps, float kp, float kd, float t_ff)
{
    can_msgs::msg::Frame frame;
    frame.id  = 0x400u | 0x100u | motor_id;   // bit10 set → MIT command
    frame.dlc = 8;
    frame.data = pack_mit_8bytes(pos_rad, vel_rps, kp, kd, t_ff);
    return frame;
}

/// 0xF1 — read pos/vel/torque/status WITHOUT entering MIT mode. The safe way
/// to learn where the arm is before applying any torque.
inline can_msgs::msg::Frame build_read_state_frame(uint8_t motor_id)
{
    can_msgs::msg::Frame frame;
    frame.id  = 0x100u | motor_id;
    frame.dlc = 1;
    frame.data.fill(0);
    frame.data[0] = CMD_READ_STATE;
    return frame;
}

/// 0xF0 (DLC=7) — configure Pos_Max/Vel_Max/T_Max (little-endian; saved to ROM).
/// Keeps motor firmware scaling in sync with the constants above.
inline can_msgs::msg::Frame build_config_limits_frame(uint8_t motor_id)
{
    const uint16_t pos = static_cast<uint16_t>(P_MAX_RAD * 10.0f + 0.5f);   // 0.1 rad
    const uint16_t vel = static_cast<uint16_t>(V_MAX_RPS * 100.0f + 0.5f);  // 0.01 rad/s
    const uint16_t tmx = static_cast<uint16_t>(T_MAX_NM * 100.0f + 0.5f);   // 0.01 Nm

    can_msgs::msg::Frame frame;
    frame.id  = 0x100u | motor_id;
    frame.dlc = 7;
    frame.data.fill(0);
    frame.data[0] = CMD_CONFIG_MAX;
    frame.data[1] = pos & 0xFF;  frame.data[2] = (pos >> 8) & 0xFF;
    frame.data[3] = vel & 0xFF;  frame.data[4] = (vel >> 8) & 0xFF;
    frame.data[5] = tmx & 0xFF;  frame.data[6] = (tmx >> 8) & 0xFF;
    return frame;
}

/// 0xCF — disable output; motor coasts freely (also the power-on state).
inline can_msgs::msg::Frame build_disable_frame(uint8_t motor_id)
{
    can_msgs::msg::Frame frame;
    frame.id  = 0x100u | motor_id;
    frame.dlc = 1;
    frame.data.fill(0);
    frame.data[0] = CMD_DISABLE;
    return frame;
}

/// 0xB1 — save current position as origin (ROM). CALIBRATION ONLY.
inline can_msgs::msg::Frame build_set_zero_frame(uint8_t motor_id)
{
    can_msgs::msg::Frame frame;
    frame.id  = 0x100u | motor_id;
    frame.dlc = 1;
    frame.data.fill(0);
    frame.data[0] = CMD_SET_ZERO;
    return frame;
}

struct Feedback {
    float pos_rad{0.0f};
    float vel_rps{0.0f};
    float torque_nm{0.0f};
    bool  in_mit_mode{false};
    bool  fault{false};
};

/// Parse a frame received on StdID == Dev_addr. Returns false if this frame is
/// not a state/MIT reply (e.g. it is the 0xF0 config echo, whose data[1..2]
/// would otherwise be misread as a position of ~92 rad — the bug that
/// corrupted the old implementation's state).
inline bool parse_feedback(const uint8_t* data, uint8_t dlc, Feedback& out)
{
    if (dlc < 7) return false;
    // Only 0xF1-style replies carry pos/vel/torque in this layout. Config,
    // zero-set and parameter replies echo other codes with different payloads.
    if (data[0] != CMD_READ_STATE && data[0] != CMD_DISABLE) {
        // 0xCF reply also carries the state snapshot per the manual's example.
        return false;
    }
    const uint16_t p = static_cast<uint16_t>((data[1] << 8) | data[2]);
    const uint16_t v = static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4));
    const uint16_t t = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);
    out.pos_rad     = uint_to_float(p, -P_MAX_RAD, P_MAX_RAD, 16);
    out.vel_rps     = uint_to_float(v, -V_MAX_RPS, V_MAX_RPS, 12);
    out.torque_nm   = uint_to_float(t, -T_MAX_NM, T_MAX_NM, 12);
    out.in_mit_mode = (data[6] & 0x01) != 0;
    out.fault       = (data[6] & 0x02) != 0;
    return true;
}

} // namespace steadywin_protocol
} // namespace arm_hardware_interface