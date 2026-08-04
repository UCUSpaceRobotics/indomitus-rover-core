#include "rover_hardware_interface/fault_detector.hpp"

#include <limits>

namespace rover_hardware_interface {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr std::array<const char *, NUM_WHEELS> kWheelNames = {"FL", "FR", "RL", "RR"};
}  // namespace

FaultDetector::FaultDetector(rclcpp::Clock::SharedPtr clock)
    : clock_(clock)
{
}

void FaultDetector::queue_event(const indomitus_interfaces::msg::FaultEvent & ev)
{
    std::lock_guard<std::mutex> lock(events_mutex_);
    pending_events_.push_back(ev);

    // Bound the queue. If the drain side ever stalls while a fault is
    // flapping, detect_*() would otherwise grow this without limit at
    // control-loop rate. Dropping the oldest keeps the most recent picture,
    // which is what an operator needs; the count of drops is not worth
    // tracking here because reaching this at all means something downstream
    // is already broken.
    if (pending_events_.size() > kMaxPendingFaultEvents) {
        pending_events_.erase(pending_events_.begin());
    }
}

std::vector<indomitus_interfaces::msg::FaultEvent> FaultDetector::drain_events()
{
    std::lock_guard<std::mutex> lock(events_mutex_);
    std::vector<indomitus_interfaces::msg::FaultEvent> events;
    events.swap(pending_events_);
    return events;
}

// detect_bus_fault — edge-trigger the transport

void FaultDetector::detect_bus_fault(
    CanBus::BusState state, const std::string & can_interface, bool motors_enabled)
{
    rover_fault::Fault current = rover_fault::Fault::NONE;
    switch (state) {
        case CanBus::BusState::BUS_OFF:        current = rover_fault::Fault::CAN_BUS_OFF;       break;
        case CanBus::BusState::ERROR_PASSIVE:  current = rover_fault::Fault::CAN_ERROR_PASSIVE; break;
        // Error-warning is a counter threshold, not a loss of function: the
        // controller is still transmitting normally. Reporting it as a fault
        // would cry wolf on a bus that is merely busy.
        case CanBus::BusState::ERROR_WARNING:
        case CanBus::BusState::OK:             current = rover_fault::Fault::NONE;              break;
    }

    if (bus_fault_.seen && current == bus_fault_.fault) return;

    if (bus_fault_.seen) {
        indomitus_interfaces::msg::FaultEvent ev;
        ev.header.stamp = clock_->now();
        ev.component = "chassis/" + can_interface;
        ev.vendor    = "socketcan";
        ev.esc_id    = 0;
        ev.raw_code  = static_cast<uint8_t>(state);
        ev.event = !rover_fault::is_fault(bus_fault_.fault)
            ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER
            : (!rover_fault::is_fault(current)
                   ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR
                   : indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CHANGE);
        ev.fault          = static_cast<uint8_t>(current);
        ev.previous_fault = static_cast<uint8_t>(bus_fault_.fault);
        ev.fault_name     = rover_fault::to_string(current);
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(current));
        // The bus has no kinematics; the counters are what explain it, and they
        // ride in the diagnostics entry alongside.
        ev.position = ev.velocity = ev.torque = kNaN;
        ev.temperature = ev.voltage = ev.current = kNaN;
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
    bool motors_enabled, const KinematicFiller & fill)
{
    indomitus_interfaces::msg::FaultEvent ev;
    ev.header.stamp = clock_->now();
    ev.component  = std::string(is_steer ? "chassis/steer_" : "chassis/drive_") + kWheelNames[index];
    ev.joint_name = joint_name;
    ev.vendor     = vendor;
    ev.esc_id     = esc_id;
    ev.raw_code   = raw_code;
    fill(ev);
    ev.enabled = motors_enabled;

    // Feedback presence is tracked separately from faults: a motor that has
    // dropped off the bus reports no fault code at all, so without this it
    // would be indistinguishable from a healthy one.
    if (tracker.seen && health_valid != tracker.health_valid) {
        ev.event = health_valid
            ? indomitus_interfaces::msg::FaultEvent::EVENT_SIGNAL_OK
            : indomitus_interfaces::msg::FaultEvent::EVENT_SIGNAL_LOST;
        ev.fault          = static_cast<uint8_t>(tracker.fault);
        ev.previous_fault = static_cast<uint8_t>(tracker.fault);
        ev.fault_name     = rover_fault::to_string(tracker.fault);
        ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(tracker.fault));
        queue_event(ev);
    }
    tracker.health_valid = health_valid;

    if (!health_valid) {
        // Fault codes are meaningless without feedback. Hold the last known
        // fault so recovery is detected correctly once frames resume.
        tracker.seen = true;
        return;
    }

    if (tracker.seen && current != tracker.fault) {
        const bool was_fault = rover_fault::is_fault(tracker.fault);
        const bool is_now    = rover_fault::is_fault(current);

        // NONE <-> NOT_ENABLED is an ordinary enable/disable, not a fault edge.
        if (was_fault || is_now) {
            ev.event = !was_fault
                ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_ENTER
                : (!is_now ? indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CLEAR
                           : indomitus_interfaces::msg::FaultEvent::EVENT_FAULT_CHANGE);
            ev.fault          = static_cast<uint8_t>(current);
            ev.previous_fault = static_cast<uint8_t>(tracker.fault);
            ev.fault_name     = rover_fault::to_string(current);
            ev.recovery       = static_cast<uint8_t>(rover_fault::recovery_for(current));
            queue_event(ev);
        }
    }

    tracker.fault = current;
    tracker.seen  = true;
}

void FaultDetector::detect_steer_fault(
    std::size_t index, const steadywin_protocol::MotorState & state,
    const std::string & joint_name, uint8_t esc_id, bool motors_enabled)
{
    const auto current = steadywin_protocol::translateFault(state.fault_code, state.mode);
    detect_motor_fault(
        steer_fault_[index], /*is_steer=*/true, index,
        current, state.diag_valid, state.fault_code,
        joint_name, esc_id, "steadywin", motors_enabled,
        [&state](indomitus_interfaces::msg::FaultEvent & ev) {
            ev.position    = state.pos_valid ? state.pos_rad : kNaN;
            ev.velocity    = kNaN;   // not reported by this vendor
            ev.torque      = kNaN;
            ev.temperature = static_cast<float>(state.temperature);
            ev.voltage     = state.voltage;
            ev.current     = state.bus_current;
            ev.mode        = state.mode;
        });
}

void FaultDetector::detect_drive_fault(
    std::size_t index, const damiao_protocol::MotorState & state,
    const std::string & joint_name, uint8_t esc_id, bool motors_enabled)
{
    const auto current = damiao_protocol::translateFault(state.err);
    detect_motor_fault(
        drive_fault_[index], /*is_steer=*/false, index,
        current, state.valid, state.err,
        joint_name, esc_id, "damiao", motors_enabled,
        [&state](indomitus_interfaces::msg::FaultEvent & ev) {
            ev.position    = state.valid ? state.pos : kNaN;
            ev.velocity    = state.valid ? state.vel : kNaN;
            ev.torque      = state.valid ? state.tor : kNaN;
            ev.temperature = static_cast<float>(state.t_mos);
            ev.voltage     = kNaN;   // not reported by this vendor
            ev.current     = kNaN;
            // Gate on the ERR nibble, not just feedback presence: a motor that
            // has faulted out is no longer running in mode 3, and reporting
            // otherwise makes a dead motor look commanded.
            ev.mode = (state.valid && state.err == damiao_protocol::ERR_ENABLED) ? 3u : 0u;
        });
}

}  // namespace rover_hardware_interface
