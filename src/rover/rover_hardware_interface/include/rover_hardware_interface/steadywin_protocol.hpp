#pragma once
#include <cstdint>
#include <cstring>
#include "can_msgs/msg/frame.hpp"
#include "rover_hardware_interface/motor_fault.hpp"

namespace steadywin_protocol {

// Operating mode (0xAE byte[6]). Mode 0 is how this motor reports that it has
// stopped accepting commands — it sets no fault bit for that condition, so a
// check on fault_code alone would read a dead motor as healthy.
constexpr uint8_t MODE_OFF = 0;

// Fault bitmask (0xAE byte[7], protocol V3.06b0 p.5)
constexpr uint8_t FAULT_VOLTAGE     = 0x01;
constexpr uint8_t FAULT_CURRENT     = 0x02;
constexpr uint8_t FAULT_TEMPERATURE = 0x04;
constexpr uint8_t FAULT_ENCODER     = 0x08;
constexpr uint8_t FAULT_HARDWARE    = 0x40;
constexpr uint8_t FAULT_SOFTWARE    = 0x80;

// Bits 4-5 are not documented in V3.06b0. They must never be set; if firmware
// starts using them, the value has a meaning we do not know.
constexpr uint8_t FAULT_KNOWN_MASK    = FAULT_VOLTAGE | FAULT_CURRENT | FAULT_TEMPERATURE |
                                        FAULT_ENCODER | FAULT_HARDWARE | FAULT_SOFTWARE;
constexpr uint8_t FAULT_RESERVED_MASK = static_cast<uint8_t>(~FAULT_KNOWN_MASK);

/// Map the fault bitmask + operating mode onto the vendor-neutral taxonomy.
/// Several bits can be set at once; the most severe wins, since the caller
/// gets a single Fault and recovery policy must key off the worst condition.
/// Bit0 covers both over- and undervoltage — the protocol does not
/// distinguish, so it maps to the deliberately ambiguous VOLTAGE value.
/// Reserved bits map to UNKNOWN rather than NONE, matching the Damiao
/// treatment of its reserved codes: an undocumented bit is a fault we cannot
/// name, and reporting it as healthy is the one answer that is certainly wrong.
inline rover_fault::Fault translateFault(uint8_t fault_code, uint8_t mode)
{
    // Documented bits first: when a reserved bit is set alongside a known one,
    // the known one is the more actionable diagnosis.
    if (fault_code & FAULT_HARDWARE)      return rover_fault::Fault::HARDWARE;
    if (fault_code & FAULT_SOFTWARE)      return rover_fault::Fault::SOFTWARE;
    if (fault_code & FAULT_ENCODER)       return rover_fault::Fault::ENCODER;
    if (fault_code & FAULT_TEMPERATURE)   return rover_fault::Fault::OVERTEMP;
    if (fault_code & FAULT_CURRENT)       return rover_fault::Fault::OVERCURRENT;
    if (fault_code & FAULT_VOLTAGE)       return rover_fault::Fault::VOLTAGE;
    if (fault_code & FAULT_RESERVED_MASK) return rover_fault::Fault::UNKNOWN;
    if (mode == MODE_OFF)                 return rover_fault::Fault::NOT_ENABLED;
    return rover_fault::Fault::NONE;
}

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

// Int32 serialization to/from little-endian byte arrays (protocol-defined, portable)
inline void serializeInt32LE(int32_t value, uint8_t* out) {
    out[0] = static_cast<uint8_t>((value >>  0) & 0xFFu);
    out[1] = static_cast<uint8_t>((value >>  8) & 0xFFu);
    out[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    out[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

inline int32_t deserializeInt32LE(const uint8_t* data) {
    return (static_cast<int32_t>(data[0]) <<  0) |
           (static_cast<int32_t>(data[1]) <<  8) |
           (static_cast<int32_t>(data[2]) << 16) |
           (static_cast<int32_t>(data[3]) << 24);
}

// Absolute position command (0xC2): int32 counts, little-endian
// Motor activates on receiving this (no separate enable needed)
inline can_msgs::msg::Frame buildAbsPositionFrame(uint8_t esc_id, float angle_rad) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 5; f.data.fill(0);
    f.data[0] = 0xC2;
    int32_t counts = radToCounts(angle_rad);
    serializeInt32LE(counts, f.data.data() + 1);
    return f;
}

// Set origin (0xB1) — saves current position as zero; persists across power cycles
// Motor responds with DLC=3: [0xB1, mechanical_angle_offset_low, mechanical_angle_offset_high]
inline can_msgs::msg::Frame buildSetOriginFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xB1;
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

// Status query (0xAE) — motor responds with voltage/current/temp/mode/fault at esc_id
inline can_msgs::msg::Frame buildStatusQueryFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xAE;
    return f;
}

// Absolute angle query (0xA3) — motor responds at 0x100|esc_id with single+multi-turn counts
inline can_msgs::msg::Frame buildAbsAngleQueryFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = 0x100u | esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xA3;
    return f;
}

// MIT real-time query (0xF1) — motor responds at 0x100|esc_id with mechanical position in bytes [1-2]
inline can_msgs::msg::Frame buildRealtimeQueryFrame(uint8_t esc_id) {
    can_msgs::msg::Frame f;
    f.id = 0x100u | esc_id; f.dlc = 1; f.data.fill(0);
    f.data[0] = 0xF1;
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

// Parse 0xA3 position response (DLC>=7)
// data[1-2]: single-turn uint16 counts, data[3-6]: multi-turn int32 counts (16384/rev)
inline bool parsePositionResponse(
    const std::array<uint8_t, 8>& data, uint8_t dlc, MotorState& out)
{
    if (dlc < 7) return false;
    if (data[0] != 0xA3 && data[0] != 0xC2 && data[0] != 0xC3) return false;
    int32_t multi_counts = deserializeInt32LE(data.data() + 3);
    out.pos_rad   = static_cast<float>(multi_counts) * TWO_PI / COUNTS_PER_REV;
    out.pos_valid = true;
    return true;
}

// Parse 0xF1 MIT real-time response (DLC>=3)
// data[1-2]: mechanical position uint16 counts (single-turn, 0-16384)
inline bool parseRealtimeResponse(
    const std::array<uint8_t, 8>& data, uint8_t dlc, MotorState& out)
{
    if (dlc < 3 || data[0] != 0xF1) return false;
    uint16_t counts = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
    out.pos_rad   = static_cast<float>(counts) * TWO_PI / COUNTS_PER_REV;
    out.pos_valid = true;
    return true;
}

// Dispatch any frame from a steer motor's address
inline void parseResponse(
    const std::array<uint8_t, 8>& data, uint8_t dlc, MotorState& out)
{
    parseStatusResponse(data, dlc, out);
    parsePositionResponse(data, dlc, out);
    parseRealtimeResponse(data, dlc, out);
}

} // namespace steadywin_protocol
