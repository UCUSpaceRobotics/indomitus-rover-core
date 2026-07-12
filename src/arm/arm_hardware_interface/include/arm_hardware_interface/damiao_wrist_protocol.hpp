#pragma once
//
// Damiao DM-J4340-2EC — MIT mode over CAN @ 1 Mbps.
// Verified against the DM-J4340-2EC user manual and the proven-working
// Python tools (arm_control.py / play_arm_waypoints.py).
//
// Key facts from the manual:
//  * MIT control frame: StdID = CAN_ID (the raw motor id).
//    payload = pos16 | vel12 | kp12 | kd12 | tff12  (same packing as MIT).
//    Kp range [0,500], Kd range [0,5]. P/V/T ranges (P_MAX, V_MAX, T_MAX) are
//    CONFIGURABLE in the Damiao debug assistant — the constants below MUST
//    match what is stored in the motor, or scaling is silently wrong.
//  * Feedback frame: StdID = Master ID (default 0!), DLC = 8:
//      D[0] = motor_ID_low_nibble | (ERR << 4)
//      D[1..2] = POS16 (big-endian), D[3]=VEL[11:4], D[4]=VEL[3:0]|T[11:8],
//      D[5]=T[7:0], D[6]=T_MOS °C, D[7]=T_Rotor °C
//    ERR: 0=disabled, 1=enabled, 8=overvolt, 9=undervolt, A=overcurrent,
//         B=MOS overtemp, C=coil overtemp, D=comm loss, E=overload.
//  * Special commands (DLC=8): FF FF FF FF FF FF FF FC = enable,
//    ...FD = disable, ...FE = save position zero (ROM), ...FB = clear error.
//  * The motor auto-disables on CAN silence (comm-loss watchdog) — you must
//    stream commands continuously while it is enabled.
//  * WARNING (manual): when position-controlling, Kd must NOT be 0 while
//    Kp > 0, or the motor oscillates / loses control.
//
#include <algorithm>
#include <array>
#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace arm_hardware_interface {
namespace damiao_wrist_protocol {

// !!! These must match the values configured in each motor via the Damiao
// debug assistant. The previously-working Python scripts used exactly these,
// so your motors are presumably configured this way. If a wrist joint reads
// or moves a scaled-wrong amount, verify PMAX/VMAX/TMAX in the assistant.
constexpr float P_MAX_RAD = 12.5f;
constexpr float V_MAX_RPS = 45.0f;
constexpr float T_MAX_NM  = 18.0f;

constexpr float KP_MAX = 500.0f;
constexpr float KD_MAX = 5.0f;

enum class ErrCode : uint8_t {
    DISABLED      = 0x0,
    ENABLED       = 0x1,
    OVERVOLTAGE   = 0x8,
    UNDERVOLTAGE  = 0x9,
    OVERCURRENT   = 0xA,
    MOS_OVERTEMP  = 0xB,
    COIL_OVERTEMP = 0xC,
    COMM_LOSS     = 0xD,
    OVERLOAD      = 0xE,
};

inline const char* err_to_string(uint8_t err)
{
    switch (err) {
        case 0x0: return "disabled";
        case 0x1: return "enabled";
        case 0x8: return "OVERVOLTAGE";
        case 0x9: return "UNDERVOLTAGE";
        case 0xA: return "OVERCURRENT";
        case 0xB: return "MOS OVERTEMP";
        case 0xC: return "COIL OVERTEMP";
        case 0xD: return "COMM LOSS";
        case 0xE: return "OVERLOAD";
        default:  return "unknown";
    }
}

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

/// MIT motion command (only obeyed while the motor is enabled).
inline can_msgs::msg::Frame build_mit_command_frame(
    uint8_t motor_id, float pos_rad, float vel_rps, float kp, float kd, float t_ff)
{
    can_msgs::msg::Frame frame;
    frame.id  = motor_id;              // Damiao uses the raw CAN ID for MIT mode
    frame.dlc = 8;
    frame.data = pack_mit_8bytes(pos_rad, vel_rps, kp, kd, t_ff);
    return frame;
}

/// A "torqueless probe": kp=0, kd=0, tff=0. Produces zero torque even when
/// the motor is enabled, and the motor answers with a feedback frame — the
/// safe way to read the true position before any stiffness is applied.
inline can_msgs::msg::Frame build_zero_gain_probe_frame(uint8_t motor_id)
{
    return build_mit_command_frame(motor_id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

inline can_msgs::msg::Frame make_special(uint8_t motor_id, uint8_t last_byte)
{
    can_msgs::msg::Frame frame;
    frame.id  = motor_id;
    frame.dlc = 8;
    frame.data.fill(0xFF);
    frame.data[7] = last_byte;
    return frame;
}

/// FF..FC — enable. The motor holds zero torque until it receives its first
/// MIT command, so enabling by itself does not move the arm.
inline can_msgs::msg::Frame build_enable_frame(uint8_t motor_id)  { return make_special(motor_id, 0xFC); }

/// FF..FD — disable (coast). Send a zero-gain MIT frame first to avoid a
/// torque discontinuity, as the proven Python tools do.
inline can_msgs::msg::Frame build_disable_frame(uint8_t motor_id) { return make_special(motor_id, 0xFD); }

/// FF..FE — save current position as zero (ROM). CALIBRATION ONLY.
inline can_msgs::msg::Frame build_set_zero_frame(uint8_t motor_id){ return make_special(motor_id, 0xFE); }

/// FF..FB — clear error.
inline can_msgs::msg::Frame build_clear_error_frame(uint8_t motor_id) { return make_special(motor_id, 0xFB); }

struct Feedback {
    uint8_t motor_id_nibble{0};  // low 4 bits of the motor CAN ID
    uint8_t err{0};              // see ErrCode
    float pos_rad{0.0f};
    float vel_rps{0.0f};
    float torque_nm{0.0f};
    uint8_t t_mos_c{0};
    uint8_t t_rotor_c{0};
};

/// Parse a Damiao feedback frame. The frame arrives on StdID == Master ID
/// (default 0) — the motor is identified by the low nibble of data[0].
inline bool parse_feedback(const uint8_t* data, uint8_t dlc, Feedback& out)
{
    if (dlc < 6) return false;
    out.motor_id_nibble = data[0] & 0x0F;
    out.err             = (data[0] >> 4) & 0x0F;
    const uint16_t p = static_cast<uint16_t>((data[1] << 8) | data[2]);
    const uint16_t v = static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4));
    const uint16_t t = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);
    out.pos_rad   = uint_to_float(p, -P_MAX_RAD, P_MAX_RAD, 16);
    out.vel_rps   = uint_to_float(v, -V_MAX_RPS, V_MAX_RPS, 12);
    out.torque_nm = uint_to_float(t, -T_MAX_NM, T_MAX_NM, 12);
    out.t_mos_c   = (dlc >= 7) ? data[6] : 0;
    out.t_rotor_c = (dlc >= 8) ? data[7] : 0;
    return true;
}

} // namespace damiao_wrist_protocol
} // namespace arm_hardware_interface