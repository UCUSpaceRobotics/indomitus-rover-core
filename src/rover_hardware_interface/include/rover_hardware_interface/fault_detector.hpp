#pragma once

#include <array>
#include <cstdint>
#include <functional>
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
// detect_*() is called once per control cycle from the RT read() thread and
// is NOT internally synchronized against itself (single-writer by
// construction); only the pending-events queue is protected, since it is
// drained from a different thread (the diagnostics/status executor).

/// Edge-triggers fault/health-change events for the eight motors and the CAN
/// bus itself, and queues them for a publisher to drain.
class FaultDetector
{
public:
    explicit FaultDetector(rclcpp::Clock::SharedPtr clock);

    void detect_steer_fault(
        std::size_t index, const steadywin_protocol::MotorState & state,
        const std::string & joint_name, uint8_t esc_id, bool motors_enabled);

    void detect_drive_fault(
        std::size_t index, const damiao_protocol::MotorState & state,
        const std::string & joint_name, uint8_t esc_id, bool motors_enabled);

    /// Same edge-triggering for the transport itself. The bus is a faultable
    /// component like any motor, so it flows through the same event pipeline
    /// and every downstream consumer handles it with no new code.
    void detect_bus_fault(
        CanBus::BusState state, const std::string & can_interface, bool motors_enabled);

    /// Returns and clears everything queued since the last call. Safe to call
    /// from a different thread than detect_*().
    std::vector<indomitus_interfaces::msg::FaultEvent> drain_events();

private:
    struct FaultTracker {
        rover_fault::Fault fault{rover_fault::Fault::NONE};
        bool health_valid{true};
        bool seen{false};
    };

    using KinematicFiller = std::function<void(indomitus_interfaces::msg::FaultEvent &)>;

    /// Shared edge-triggering for one motor. `fill` populates the
    /// vendor-specific kinematic/health fields on the event before it is
    /// queued.
    void detect_motor_fault(
        FaultTracker & tracker, bool is_steer, std::size_t index,
        rover_fault::Fault current, bool health_valid, uint8_t raw_code,
        const std::string & joint_name, uint8_t esc_id, const char * vendor,
        bool motors_enabled, const KinematicFiller & fill);

    void queue_event(const indomitus_interfaces::msg::FaultEvent & ev);

    rclcpp::Clock::SharedPtr clock_;

    std::array<FaultTracker, NUM_WHEELS> steer_fault_;
    std::array<FaultTracker, NUM_WHEELS> drive_fault_;
    FaultTracker bus_fault_;

    std::mutex events_mutex_;
    std::vector<indomitus_interfaces::msg::FaultEvent> pending_events_;
    static constexpr std::size_t kMaxPendingFaultEvents{256};
};

}  // namespace rover_hardware_interface
