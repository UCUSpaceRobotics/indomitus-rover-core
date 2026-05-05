#pragma once
#include <cstdint>
#include <cstring>
#include "can_msgs/msg/frame.hpp"

namespace steadywin_protocol {

// Steadywin custom CAN protocol V3.06b0
// 16384 counts = 1 full motor revolution = 360 degrees

constexpr float COUNTS_PER_REV = 16384.0f;
constexpr float TWO_PI = 6.28318530f;

struct MotorState {
    float pos_rad       = 0.0f;   // multi-turn absolute position in radians
    bool  pos_valid     = false;

    float   voltage      = 0.0f;  // V
    float   bus_current  = 0.0f;  // A
    uint8_t temperature  = 0;     // °C
    uint8_t mode         = 0;     // 0=off 1=voltage 2=Iq 3=speed 4=position
    uint8_t fault_code   = 0;     // Bit0=voltage Bit1=current Bit2=temp Bit3=encoder Bit6=hw Bit7=sw
    bool    diag_valid   = false;
};

inline int32_t radToCounts(float angle_rad) {
    return static_cast<int32_t>(angle_rad * COUNTS_PER_REV / TWO_PI);
}

// Absolute position command (0xC2): int32 counts, little-endian
// Motor activates on receiving this (no separate enable needed)
inline can_msgs::msg::Frame buildAbsPositionFrame(uint8_t esc_id, float angle_rad) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 5; f.data.fill(0);
    f.data[0] = 0xC2;
    int32_t counts = radToCounts(angle_rad);
    std::memcpy(f.data.data() + 1, &counts, 4);
    return f;
}

// Free/disable motor (0xCF) — motor enters uncontrolled free-spin state
inline can_msgs::msg::Frame buildDisableFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xCF;
    return f;
}

// Clear fault (0xAF) — send before first command after power-on or fault
inline can_msgs::msg::Frame buildClearFaultFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xAF;
    return f;
}

// Status query (0xAE) — motor responds with 8-byte status frame at its own address
inline can_msgs::msg::Frame buildStatusQueryFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xAE;
    return f;
}

// Parse 0xAE status response (DLC=8, data[0]=0xAE)
// Payload: voltage(2u 0.01V LE), current(2u 0.01A LE), temp(1u °C), mode(1u), fault(1u)
inline bool parseStatusResponse(
    const std::array<uint8_t, 8>& data, uint8_t dlc, MotorState& out)
{
    if (dlc < 8 || data[0] != 0xAE) return false;
    uint16_t v_raw = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
    uint16_t i_raw = static_cast<uint16_t>(data[3]) | (static_cast<uint16_t>(data[4]) << 8);
    out.voltage     = v_raw * 0.01f;
    out.bus_current = i_raw * 0.01f;
    out.temperature = data[5];
    out.mode        = data[6];
    out.fault_code  = data[7];
    out.diag_valid  = true;
    return true;
}

// Parse position response (0xA3 query response or 0xC2/0xC3 command echo, DLC>=7)
// data[1-2]: single-turn uint16, data[3-6]: multi-turn int32, unit: counts (16384/rev)
inline bool parsePositionResponse(
    const std::array<uint8_t, 8>& data, uint8_t dlc, MotorState& out)
{
    if (dlc < 7) return false;
    if (data[0] != 0xA3 && data[0] != 0xC2 && data[0] != 0xC3) return false;
    int32_t multi_counts;
    std::memcpy(&multi_counts, data.data() + 3, 4);
    out.pos_rad   = static_cast<float>(multi_counts) * TWO_PI / COUNTS_PER_REV;
    out.pos_valid = true;
    return true;
}

// Dispatch any frame from a steer motor's address
inline void parseResponse(
    const std::array<uint8_t, 8>& data, uint8_t dlc, MotorState& out)
{
    parseStatusResponse(data, dlc, out);
    parsePositionResponse(data, dlc, out);
}

} // namespace steadywin_protocol
