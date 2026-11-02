#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "indomitus_interfaces/msg/fault_event.hpp"

#include "rover_hardware_interface/can_bus.hpp"
#include "rover_hardware_interface/constants.hpp"
#include "rover_hardware_interface/damiao_protocol.hpp"
#include "rover_hardware_interface/motor_fault.hpp"
#include "rover_hardware_interface/steadywin_protocol.hpp"

namespace rover_hardware_interface {

// Detection is edge-triggered bookkeeping only — no I/O, no ROS publishing.

/// Edge-triggers fault/health-change events for the eight motors and the CAN
/// bus itself, and queues them for a publisher to drain.
class FaultDetector
{
public:
    explicit FaultDetector(rclcpp::Clock::SharedPtr clock);

    struct FreezeFrame {
        static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

        float position    = kNaN;
        float velocity    = kNaN;
        float torque      = kNaN;
        float temperature = kNaN;
        float voltage     = kNaN;
        float current     = kNaN;
        uint8_t mode      = 0;
    };

    /// `health_valid` is the caller's verdict on whether this motor's feedback
    /// is currently both present and fresh
    void detect_steer_fault(
        std::size_t index, const steadywin_protocol::MotorState & state,
        bool health_valid, const std::string & joint_name, uint8_t esc_id,
        bool motors_enabled);

    void detect_drive_fault(
        std::size_t index, const damiao_protocol::MotorState & state,
        bool health_valid, const std::string & joint_name, uint8_t esc_id,
        bool motors_enabled);

    /// The bus is a faultable
    /// component like any motor, so it flows through the same event pipeline
    void detect_bus_fault(
        CanBus::BusState state, const std::string & can_interface, bool motors_enabled);

    /// Returns and clears everything queued since the last call. Safe to call
    /// from a different thread than detect_*().
    std::vector<indomitus_interfaces::msg::FaultEvent> drain_events();

    /// Events discarded because the queue was full — non-zero means the drain
    /// side stalled, which is itself worth reporting.
    std::size_t dropped_events() const;

private:
    struct FaultTracker {
        rover_fault::Fault fault{rover_fault::Fault::NONE};
        bool health_valid{true};
        bool seen{false};
        // A motor is silent before it first answers, which is not a dropout.
        // SIGNAL_OK is only meaningful once a SIGNAL_LOST has been reported.
        bool signal_lost_reported{false};
    };

    /// Shared edge-triggering for one motor. Returns without building anything
    /// when nothing changed.
    void detect_motor_fault(
        FaultTracker & tracker, bool is_steer, std::size_t index,
        rover_fault::Fault current, bool health_valid, uint8_t raw_code,
        const std::string & joint_name, uint8_t esc_id, const char * vendor,
        bool motors_enabled, const FreezeFrame & freeze);

    void queue_event(const indomitus_interfaces::msg::FaultEvent & ev);

    rclcpp::Clock::SharedPtr clock_;

    std::array<FaultTracker, NUM_WHEELS> steer_fault_;
    std::array<FaultTracker, NUM_WHEELS> drive_fault_;
    FaultTracker bus_fault_;

    /// Scratch message reused across transitions, so the strings that make up
    /// an event keep their heap buffers instead of reallocating each time.
    /// Owned exclusively by the detect_*() thread; never touched by the drain.
    indomitus_interfaces::msg::FaultEvent staging_event_;

    static constexpr std::size_t kMaxPendingFaultEvents{256};

    mutable std::mutex events_mutex_;
    std::array<indomitus_interfaces::msg::FaultEvent, kMaxPendingFaultEvents> pending_events_;
    std::size_t pending_head_{0};   ///< next slot to write
    std::size_t pending_tail_{0};   ///< oldest unread slot
    std::size_t pending_count_{0};
    std::size_t dropped_events_{0};
};

}  // namespace rover_hardware_interface
