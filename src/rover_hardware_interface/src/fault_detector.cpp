#include "rover_hardware_interface/fault_detector.hpp"

namespace rover_hardware_interface {

namespace {
constexpr std::array<const char *, NUM_WHEELS> kWheelNames = {"FL", "FR", "RL", "RR"};
}  // namespace

FaultDetector::FaultDetector(rclcpp::Clock::SharedPtr clock)
    : clock_(clock)
{
}

void FaultDetector::queue_event(const indomitus_interfaces::msg::FaultEvent & ev)
{
    std::lock_guard<std::mutex> lock(events_mutex_);

    if (pending_count_ == kMaxPendingFaultEvents) {
        pending_tail_ = (pending_tail_ + 1) % kMaxPendingFaultEvents;
        ++dropped_events_;
    } else {
        ++pending_count_;
    }

    pending_events_[pending_head_] = ev;
    pending_head_ = (pending_head_ + 1) % kMaxPendingFaultEvents;
}

std::vector<indomitus_interfaces::msg::FaultEvent> FaultDetector::drain_events()
{
    std::lock_guard<std::mutex> lock(events_mutex_);

    std::vector<indomitus_interfaces::msg::FaultEvent> events;
    events.reserve(pending_count_);
    for (std::size_t i = 0; i < pending_count_; ++i) {
        events.push_back(pending_events_[(pending_tail_ + i) % kMaxPendingFaultEvents]);
    }

    pending_tail_ = pending_head_;
    pending_count_ = 0;
    return events;
}

std::size_t FaultDetector::dropped_events() const
{
    std::lock_guard<std::mutex> lock(events_mutex_);
    return dropped_events_;
}

// detect_bus_fault — edge-trigger the transport

void FaultDetector::detect_bus_fault(
    CanBus::BusState state, const std::string & can_interface, bool motors_enabled)
{
    rover_fault::Fault current = rover_fault::Fault::NONE;
    switch (state) {
        case CanBus::BusState::BUS_OFF:        current = rover_fault::Fault::CAN_BUS_OFF;       break;
        case CanBus::BusState::ERROR_PASSIVE:  current = rover_fault::Fault::CAN_ERROR_PASSIVE; break;
        case CanBus::BusState::ERROR_WARNING:
        case CanBus::BusState::OK:             current = rover_fault::Fault::NONE;              break;
    }

    if (bus_fault_.seen && current == bus_fault_.fault) return;

    if (bus_fault_.seen) {
        auto & ev = staging_event_;
        ev.header.stamp = clock_->now();
        ev.component.assign("chassis/").append(can_interface);
        ev.joint_name.clear();
        ev.vendor.assign("socketcan");
        ev.esc_id    = 0;
        ev.raw_code  = static_cast<uint8_t>(state);
        ev.event = !rover_fault::is_fault(bus_fault_.fault)
            ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER
            : (!rover_fault::is_fault(current)
                   ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR
                   : indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CHANGE);
        ev.fault          = static_cast<uint8_t>(current);
        ev.previous_fault = static_cast<uint8_t>(bus_fault_.fault);
        ev.fault_name.assign(rover_fault::to_string(current));
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(current));
        // The bus has no kinematics; the counters are what explain it, and they
        // ride in the diagnostics entry alongside.
        const FreezeFrame empty;
        ev.position    = empty.position;
        ev.velocity    = empty.velocity;
        ev.torque      = empty.torque;
        ev.temperature = empty.temperature;
        ev.voltage     = empty.voltage;
        ev.current     = empty.current;
        ev.mode    = static_cast<uint8_t>(state);
        ev.enabled = motors_enabled;
        queue_event(ev);
    }

    bus_fault_.fault = current;
    bus_fault_.seen  = true;
}

// detect_motor_fault — edge-trigger one motor, queue an event if changed

void FaultDetector::detect_motor_fault(
    FaultTracker & tracker, bool is_steer, std::size_t index,
    rover_fault::Fault current, bool health_valid, uint8_t raw_code,
    const std::string & joint_name, uint8_t esc_id, const char * vendor,
    bool motors_enabled, const FreezeFrame & freeze)
{
    const bool signal_edge =
        health_valid != tracker.health_valid &&
        (health_valid ? tracker.signal_lost_reported : tracker.seen);

    // Fault codes are meaningless without feedback
    const bool fault_edge =
        health_valid && tracker.seen && current != tracker.fault &&
        (rover_fault::is_fault(tracker.fault) || rover_fault::is_fault(current));

    if (!signal_edge && !fault_edge) {
        tracker.health_valid = health_valid;
        tracker.seen = true;
        // Hold the last known fault while feedback is missing, so recovery is
        // detected correctly once frames resume.
        if (health_valid) tracker.fault = current;
        return;
    }

    // ── Transition: build the event once and reuse it for both edges ──────────
    auto & ev = staging_event_;
    ev.header.stamp = clock_->now();
    ev.component.assign(is_steer ? "chassis/steer_" : "chassis/drive_")
        .append(kWheelNames[index]);
    ev.joint_name = joint_name;
    ev.vendor.assign(vendor);
    ev.esc_id      = esc_id;
    ev.raw_code    = raw_code;
    ev.position    = freeze.position;
    ev.velocity    = freeze.velocity;
    ev.torque      = freeze.torque;
    ev.temperature = freeze.temperature;
    ev.voltage     = freeze.voltage;
    ev.current     = freeze.current;
    ev.mode        = freeze.mode;
    ev.enabled     = motors_enabled;

    if (signal_edge) {
        tracker.signal_lost_reported = !health_valid;
        ev.event = health_valid
            ? indomitus_interfaces::msg::FaultEvent::EVENT_SIGNAL_OK
            : indomitus_interfaces::msg::FaultEvent::EVENT_SIGNAL_LOST;
        ev.fault          = static_cast<uint8_t>(tracker.fault);
        ev.previous_fault = static_cast<uint8_t>(tracker.fault);
        ev.fault_name.assign(rover_fault::to_string(tracker.fault));
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(tracker.fault));
        queue_event(ev);
    }

    if (fault_edge) {
        const bool was_fault = rover_fault::is_fault(tracker.fault);
        const bool is_now    = rover_fault::is_fault(current);
        ev.event = !was_fault
            ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER
            : (!is_now ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR
                       : indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CHANGE);
        ev.fault          = static_cast<uint8_t>(current);
        ev.previous_fault = static_cast<uint8_t>(tracker.fault);
        ev.fault_name.assign(rover_fault::to_string(current));
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(current));
        queue_event(ev);
    }

    tracker.health_valid = health_valid;
    tracker.seen = true;
    if (health_valid) tracker.fault = current;
}

void FaultDetector::detect_steer_fault(
    std::size_t index, const steadywin_protocol::MotorState & state,
    bool health_valid, const std::string & joint_name, uint8_t esc_id,
    bool motors_enabled)
{
    const auto current = steadywin_protocol::translateFault(state.fault_code, state.mode);

    FreezeFrame freeze;
    if (state.pos_valid) freeze.position = state.pos_rad;
    // velocity/torque stay NaN — not reported by this vendor
    freeze.temperature = static_cast<float>(state.temperature);
    freeze.voltage     = state.voltage;
    freeze.current     = state.bus_current;
    freeze.mode        = state.mode;

    detect_motor_fault(
        steer_fault_[index], /*is_steer=*/true, index,
        current, health_valid, state.fault_code,
        joint_name, esc_id, "steadywin", motors_enabled, freeze);
}

void FaultDetector::detect_drive_fault(
    std::size_t index, const damiao_protocol::MotorState & state,
    bool health_valid, const std::string & joint_name, uint8_t esc_id,
    bool motors_enabled)
{
    const auto current = damiao_protocol::translateFault(state.err);

    FreezeFrame freeze;
    if (state.valid) {
        freeze.position = state.pos;
        freeze.velocity = state.vel;
        freeze.torque   = state.tor;
    }
    freeze.temperature = static_cast<float>(state.t_mos);
    // voltage/current stay NaN — not reported by this vendor
    // Gate on the ERR nibble, not just feedback presence: a motor that has
    // faulted out is no longer running in mode 3, and reporting otherwise
    // makes a dead motor look commanded.
    freeze.mode = (state.valid && state.err == damiao_protocol::ERR_ENABLED) ? 3u : 0u;

    detect_motor_fault(
        drive_fault_[index], /*is_steer=*/false, index,
        current, health_valid, state.err,
        joint_name, esc_id, "damiao", motors_enabled, freeze);
}

}  // namespace rover_hardware_interface
