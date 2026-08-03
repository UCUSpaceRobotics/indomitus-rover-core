#pragma once
#include <cstdint>

// Vendor-neutral fault taxonomy.
//
// Damiao packs state and fault into one 4-bit nibble (0x0 disabled, 0x1 enabled,
// 0x8-0xE faults); Steadywin uses an independent fault bitmask plus a separate
// operating-mode byte. Downstream consumers must never have to know which.
// Each vendor protocol header provides a translateFault() that maps its own
// encoding onto the Fault enum below; nothing above that layer branches on
// vendor.

namespace rover_fault {

enum class Fault : uint8_t {
    NONE         = 0,
    NOT_ENABLED  = 1,   ///< cleanly out of enabled mode — not a failure
    COMM_LOSS    = 2,
    UNDERVOLTAGE = 3,
    OVERVOLTAGE  = 4,
    VOLTAGE      = 5,   ///< over/under indistinguishable (Steadywin reports one bit)
    OVERCURRENT  = 6,
    OVERTEMP     = 7,
    OVERLOAD     = 8,
    ENCODER      = 9,
    HARDWARE     = 10,
    SOFTWARE     = 11,
    UNKNOWN      = 12,  ///< reserved/undocumented code — surfaced, never ignored
    // Transport-level faults. Not attached to any one motor: when the bus is
    // down every motor is unreachable, so these are reported against the CAN
    // interface itself rather than eight times over.
    CAN_ERROR_PASSIVE = 13,  ///< controller degraded, still transmitting
    CAN_BUS_OFF       = 14,  ///< controller off the bus, nothing gets through
};

/// How a fault manager is permitted to respond. Recovery policy is a property
/// of the fault class, not of the vendor that reported it.
enum class Recovery : uint8_t {
    NONE      = 0,  ///< healthy
    IMMEDIATE = 1,  ///< retry at once; cheap and safe
    LIMITED   = 2,  ///< retry with backoff, capped attempts
    BACKOFF   = 3,  ///< retry slowly, low cap
    THERMAL   = 4,  ///< retry only once temperature has dropped
    LATCH     = 5,  ///< never auto-retry; operator decision only
};

constexpr bool is_fault(Fault f)
{
    return f != Fault::NONE && f != Fault::NOT_ENABLED;
}

constexpr Recovery recovery_for(Fault f)
{
    switch (f) {
        case Fault::NONE:         return Recovery::NONE;
        case Fault::NOT_ENABLED:
        case Fault::COMM_LOSS:    return Recovery::IMMEDIATE;
        // Voltage faults are a power-system symptom: clearing them is safe, but
        // repeating without bound would just mask the underlying supply problem.
        case Fault::UNDERVOLTAGE:
        case Fault::OVERVOLTAGE:
        case Fault::VOLTAGE:      return Recovery::LIMITED;
        case Fault::OVERCURRENT:
        case Fault::OVERLOAD:     return Recovery::BACKOFF;
        case Fault::OVERTEMP:     return Recovery::THERMAL;
        // Nothing the software can do resolves these, and retrying into them
        // risks the hardware.
        case Fault::ENCODER:
        case Fault::HARDWARE:
        case Fault::SOFTWARE:
        case Fault::UNKNOWN:      return Recovery::LATCH;
        // Both clear themselves: error-passive lifts once the controller's
        // error counters fall, and bus-off is recovered by the kernel when the
        // interface is configured with restart-ms. Latching would keep the
        // rover down through a fault the driver stack already handles.
        case Fault::CAN_ERROR_PASSIVE:
        case Fault::CAN_BUS_OFF:  return Recovery::LIMITED;
    }
    return Recovery::LATCH;  // unreachable; fail safe
}

constexpr const char * to_string(Fault f)
{
    switch (f) {
        case Fault::NONE:         return "none";
        case Fault::NOT_ENABLED:  return "not_enabled";
        case Fault::COMM_LOSS:    return "communication_loss";
        case Fault::UNDERVOLTAGE: return "undervoltage";
        case Fault::OVERVOLTAGE:  return "overvoltage";
        case Fault::VOLTAGE:      return "voltage";
        case Fault::OVERCURRENT:  return "overcurrent";
        case Fault::OVERTEMP:     return "overtemperature";
        case Fault::OVERLOAD:     return "overload";
        case Fault::ENCODER:      return "encoder";
        case Fault::HARDWARE:     return "hardware";
        case Fault::SOFTWARE:     return "software";
        case Fault::UNKNOWN:      return "unknown";
        case Fault::CAN_ERROR_PASSIVE: return "can_error_passive";
        case Fault::CAN_BUS_OFF:       return "can_bus_off";
    }
    return "unknown";
}

}  // namespace rover_fault
