#pragma once
#include <cstdint>
#include <cstring>
#include "can_msgs/msg/frame.hpp"

namespace damiao_protocol {

struct MotorState {
    float pos    = 0.0f;   // rad, motor shaft
    float vel    = 0.0f;   // rad/s, motor shaft
    float tor    = 0.0f;   // Nm
    uint8_t t_mos    = 0;  // °C, MOSFET temperature
    uint8_t t_rotor  = 0;  // °C, rotor temperature
    uint8_t err      = 0;  // error nibble (0x1 = Enabled/normal)
    bool valid   = false;
};

inline uint16_t floatToUint(float x, float xmin, float xmax, int bits) {
    if (x < xmin) x = xmin;
    if (x > xmax) x = xmax;
    float span = xmax - xmin;
    return static_cast<uint16_t>((x - xmin) * static_cast<float>((1 << bits) - 1) / span);
}

inline float uintToFloat(uint16_t x, float xmin, float xmax, int bits) {
    float span = xmax - xmin;
    return static_cast<float>(x) * span / static_cast<float>((1 << bits) - 1) + xmin;
}

// CAN ID = 0x200 + esc_id, 4-byte IEEE 754 float (motor shaft rad/s)
inline can_msgs::msg::Frame buildVelocityFrame(uint8_t esc_id, float v_rad_s) {
    can_msgs::msg::Frame f;
    f.id  = 0x200u + esc_id;
    f.dlc = 4;
    std::memcpy(f.data.data(), &v_rad_s, 4);
    return f;
}

// CAN ID = 0x100 + esc_id, bytes 0-3: position (rad), bytes 4-7: velocity cap (rad/s)
inline can_msgs::msg::Frame buildPositionVelocityFrame(uint8_t esc_id, float p_rad, float v_max) {
    can_msgs::msg::Frame f;
    f.id  = 0x100u + esc_id;
    f.dlc = 8;
    std::memcpy(f.data.data() + 0, &p_rad, 4);
    std::memcpy(f.data.data() + 4, &v_max, 4);
    return f;
}

// CAN ID = 0x7FF, register 10 write — sets motor control mode
// mode: 1=MIT, 2=PositionVelocity, 3=Velocity
inline can_msgs::msg::Frame buildSetModeFrame(uint8_t esc_id, uint8_t mode) {
    can_msgs::msg::Frame f;
    f.id  = 0x7FFu;
    f.dlc = 8;
    f.data.fill(0x00);
    f.data[0] = esc_id & 0xFFu;
    f.data[1] = (esc_id >> 8) & 0xFFu;
    f.data[2] = 0x55;   // write command
    f.data[3] = 10;     // register 10 = CTRL_MODE
    f.data[4] = mode;
    return f;
}

// CAN ID = esc_id, payload FF FF FF FF FF FF FF FC
inline can_msgs::msg::Frame buildEnableFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id  = esc_id;
    f.dlc = 8;
    f.data.fill(0xFF);
    f.data[7] = 0xFC;
    
    return f;
}

// CAN ID = esc_id, payload FF FF FF FF FF FF FF FD
inline can_msgs::msg::Frame buildDisableFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id  = esc_id;
    f.dlc = 8;
    f.data.fill(0xFF);
    f.data[7] = 0xFD;
    return f;
}

// Decode 8-byte Damiao feedback frame into MotorState.
// Returns false if dlc < 8 or motor ID in D[0] doesn't match esc_id.
inline bool parseFeedback(
    const std::array<uint8_t, 8>& data, uint8_t dlc,
    uint8_t esc_id,
    float pmax, float vmax, float tmax,
    MotorState& out)
{
    if (dlc < 8) return false;
    if ((data[0] & 0x0F) != esc_id) return false;

    uint16_t p_int = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    uint16_t v_int = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    uint16_t t_int = (static_cast<uint16_t>(data[4] & 0x0Fu) << 8) | data[5];

    out.err     = (data[0] >> 4) & 0x0Fu;
    out.pos     = uintToFloat(p_int, -pmax,  pmax,  16);
    out.vel     = uintToFloat(v_int, -vmax,  vmax,  12);
    out.tor     = uintToFloat(t_int, -tmax,  tmax,  12);
    out.t_mos   = data[6];
    out.t_rotor = data[7];
    out.valid   = true;
    return true;
}

} // namespace damiao_protocol
