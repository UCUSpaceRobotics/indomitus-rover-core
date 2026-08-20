// Edge-triggering behaviour of FaultDetector.
//
// These tests exist because the failure mode of an edge detector is silence:
// a missing SIGNAL_LOST looks exactly like a healthy rover, and a spurious
// SIGNAL_OK looks exactly like a recovery. Neither shows up in a smoke test.

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "rover_hardware_interface/fault_detector.hpp"

using rover_hardware_interface::FaultDetector;
using Event = indomitus_interfaces::msg::FaultEvent;

namespace {

rclcpp::Clock::SharedPtr test_clock()
{
    // Steady rather than ROS time: no node, no time source, no /clock.
    return std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
}

steadywin_protocol::MotorState steer_state(uint8_t fault_code, uint8_t mode, bool diag_valid)
{
    steadywin_protocol::MotorState s;
    s.fault_code = fault_code;
    s.mode = mode;
    s.diag_valid = diag_valid;
    s.pos_valid = true;
    s.pos_rad = 1.5f;
    s.voltage = 48.0f;
    s.bus_current = 2.0f;
    s.temperature = 41;
    return s;
}

damiao_protocol::MotorState drive_state(uint8_t err, bool valid)
{
    damiao_protocol::MotorState s;
    s.err = err;
    s.valid = valid;
    s.pos = 0.25f;
    s.vel = 3.0f;
    s.tor = 1.0f;
    s.t_mos = 39;
    return s;
}

/// One control cycle for steer motor 0.
void tick_steer(FaultDetector & d, const steadywin_protocol::MotorState & s, bool health_valid)
{
    d.detect_steer_fault(0, s, health_valid, "fl_steer", 11, /*motors_enabled=*/true);
}

}  // namespace

// ── First observation ────────────────────────────────────────────────────────

TEST(FaultDetectorFirstObservation, SilentMotorAtStartupEmitsNothing) {
    FaultDetector d(test_clock());
    const auto silent = steer_state(0x00, 0, /*diag_valid=*/false);

    for (int i = 0; i < 5; ++i) tick_steer(d, silent, false);

    EXPECT_TRUE(d.drain_events().empty())
        << "a motor that has not answered yet has not lost anything";
}

TEST(FaultDetectorFirstObservation, FirstFeedbackIsNotARecovery) {
    FaultDetector d(test_clock());

    tick_steer(d, steer_state(0x00, 0, false), false);           // never answered
    tick_steer(d, steer_state(0x00, 4, true), true);             // first ever frame

    EXPECT_TRUE(d.drain_events().empty())
        << "SIGNAL_OK here would be unpaired: no SIGNAL_LOST was ever reported";
}

// ── Signal loss and recovery ─────────────────────────────────────────────────

TEST(FaultDetectorSignal, DropoutAndRecoveryArePairedAndReportedOnce) {
    FaultDetector d(test_clock());
    const auto healthy = steer_state(0x00, 4, true);

    tick_steer(d, healthy, true);
    d.drain_events();

    // Feedback stops. The state itself is unchanged — the vendor "valid" flag
    // latches — so only the caller's freshness verdict moves.
    tick_steer(d, healthy, false);
    tick_steer(d, healthy, false);   // still silent: must not repeat

    auto lost = d.drain_events();
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_EQ(lost[0].event, Event::EVENT_SIGNAL_LOST);
    EXPECT_EQ(lost[0].component, "chassis/steer_FL");
    EXPECT_EQ(lost[0].esc_id, 11u);

    tick_steer(d, healthy, true);
    tick_steer(d, healthy, true);

    auto back = d.drain_events();
    ASSERT_EQ(back.size(), 1u);
    EXPECT_EQ(back[0].event, Event::EVENT_SIGNAL_OK);
}

TEST(FaultDetectorSignal, SecondDropoutIsStillReported) {
    FaultDetector d(test_clock());
    const auto healthy = steer_state(0x00, 4, true);

    tick_steer(d, healthy, true);
    tick_steer(d, healthy, false);
    tick_steer(d, healthy, true);
    d.drain_events();

    tick_steer(d, healthy, false);
    auto events = d.drain_events();
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].event, Event::EVENT_SIGNAL_LOST);
}

// ── Fault edges ──────────────────────────────────────────────────────────────

TEST(FaultDetectorFault, EnterChangeClear) {
    FaultDetector d(test_clock());

    tick_steer(d, steer_state(0x00, 4, true), true);
    d.drain_events();

    tick_steer(d, steer_state(steadywin_protocol::FAULT_TEMPERATURE, 4, true), true);
    auto enter = d.drain_events();
    ASSERT_EQ(enter.size(), 1u);
    EXPECT_EQ(enter[0].event, Event::EVENT_FAULT_ENTER);
    EXPECT_EQ(enter[0].fault, Event::FAULT_OVERTEMP);
    EXPECT_EQ(enter[0].recovery, Event::RECOVERY_THERMAL);
    EXPECT_FLOAT_EQ(enter[0].temperature, 41.0f);   // freeze frame captured
    EXPECT_FLOAT_EQ(enter[0].voltage, 48.0f);

    tick_steer(d, steer_state(steadywin_protocol::FAULT_HARDWARE, 4, true), true);
    auto change = d.drain_events();
    ASSERT_EQ(change.size(), 1u);
    EXPECT_EQ(change[0].event, Event::EVENT_FAULT_CHANGE);
    EXPECT_EQ(change[0].fault, Event::FAULT_HARDWARE);
    EXPECT_EQ(change[0].previous_fault, Event::FAULT_OVERTEMP);

    tick_steer(d, steer_state(0x00, 4, true), true);
    auto clear = d.drain_events();
    ASSERT_EQ(clear.size(), 1u);
    EXPECT_EQ(clear[0].event, Event::EVENT_FAULT_CLEAR);
    EXPECT_EQ(clear[0].fault, Event::FAULT_NONE);
}

TEST(FaultDetectorFault, EnableDisableIsNotAFaultEdge) {
    FaultDetector d(test_clock());

    tick_steer(d, steer_state(0x00, 4, true), true);   // NONE
    tick_steer(d, steer_state(0x00, 0, true), true);   // NOT_ENABLED
    tick_steer(d, steer_state(0x00, 4, true), true);   // NONE again

    EXPECT_TRUE(d.drain_events().empty())
        << "an operator disabling the motors is not a fault";
}

TEST(FaultDetectorFault, FaultCodeWhileSilentIsHeldUntilFeedbackResumes) {
    FaultDetector d(test_clock());

    tick_steer(d, steer_state(steadywin_protocol::FAULT_ENCODER, 4, true), true);
    d.drain_events();   // FAULT_ENTER

    // While silent, whatever the stale struct says must not be believed.
    tick_steer(d, steer_state(0x00, 4, true), false);
    auto lost = d.drain_events();
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_EQ(lost[0].event, Event::EVENT_SIGNAL_LOST);
    EXPECT_EQ(lost[0].fault, Event::FAULT_ENCODER) << "last known fault is held";

    // Coming back genuinely healthy is a recovery *and* a fault clear.
    tick_steer(d, steer_state(0x00, 4, true), true);
    auto back = d.drain_events();
    ASSERT_EQ(back.size(), 2u);
    EXPECT_EQ(back[0].event, Event::EVENT_SIGNAL_OK);
    EXPECT_EQ(back[1].event, Event::EVENT_FAULT_CLEAR);
}

TEST(FaultDetectorFault, DriveMotorCommLossIsReportedWithFreezeFrame) {
    FaultDetector d(test_clock());

    d.detect_drive_fault(1, drive_state(damiao_protocol::ERR_ENABLED, true), true,
                         "fr_drive", 21, true);
    d.drain_events();

    d.detect_drive_fault(1, drive_state(0xD, true), true, "fr_drive", 21, true);
    auto events = d.drain_events();
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].event, Event::EVENT_FAULT_ENTER);
    EXPECT_EQ(events[0].fault, Event::FAULT_COMM_LOSS);
    EXPECT_EQ(events[0].component, "chassis/drive_FR");
    EXPECT_EQ(events[0].vendor, "damiao");
    EXPECT_EQ(events[0].raw_code, 0xDu);
    EXPECT_FLOAT_EQ(events[0].velocity, 3.0f);
}

// ── Transport ────────────────────────────────────────────────────────────────

TEST(FaultDetectorBus, WarningIsNotAFaultButPassiveIs) {
    using Bus = rover_hardware_interface::CanBus::BusState;
    FaultDetector d(test_clock());

    d.detect_bus_fault(Bus::OK, "can0", true);
    EXPECT_TRUE(d.drain_events().empty());

    // Error-warning is a counter threshold; the controller still transmits.
    d.detect_bus_fault(Bus::ERROR_WARNING, "can0", true);
    EXPECT_TRUE(d.drain_events().empty());

    d.detect_bus_fault(Bus::ERROR_PASSIVE, "can0", true);
    auto passive = d.drain_events();
    ASSERT_EQ(passive.size(), 1u);
    EXPECT_EQ(passive[0].event, Event::EVENT_FAULT_ENTER);
    EXPECT_EQ(passive[0].fault, Event::FAULT_CAN_ERROR_PASSIVE);
    EXPECT_EQ(passive[0].component, "chassis/can0");

    d.detect_bus_fault(Bus::BUS_OFF, "can0", true);
    auto off = d.drain_events();
    ASSERT_EQ(off.size(), 1u);
    EXPECT_EQ(off[0].event, Event::EVENT_FAULT_CHANGE);
    EXPECT_EQ(off[0].fault, Event::FAULT_CAN_BUS_OFF);

    // Recovery all the way back down must clear, not stay latched.
    d.detect_bus_fault(Bus::ERROR_WARNING, "can0", true);
    auto recovered = d.drain_events();
    ASSERT_EQ(recovered.size(), 1u);
    EXPECT_EQ(recovered[0].event, Event::EVENT_FAULT_CLEAR);
}

// ── Queue behaviour ──────────────────────────────────────────────────────────

TEST(FaultDetectorQueue, OverflowDropsOldestAndCountsIt) {
    FaultDetector d(test_clock());
    constexpr int kCapacity = 256;
    constexpr int kEmitted = kCapacity + 44;

    tick_steer(d, steer_state(0x00, 4, true), true);

    // Alternate healthy/faulted so every cycle is a transition, and never
    // drain: this is the "drain side stalled while a fault flaps" case.
    for (int i = 0; i < kEmitted; ++i) {
        const uint8_t code = (i % 2 == 0) ? steadywin_protocol::FAULT_TEMPERATURE : 0x00;
        tick_steer(d, steer_state(code, 4, true), true);
    }

    EXPECT_EQ(d.dropped_events(), static_cast<std::size_t>(kEmitted - kCapacity));

    auto events = d.drain_events();
    EXPECT_EQ(events.size(), static_cast<std::size_t>(kCapacity));
    // Oldest dropped means the newest survived: the last emitted transition
    // was i = kEmitted-1, which is odd, i.e. a clear.
    EXPECT_EQ(events.back().event, Event::EVENT_FAULT_CLEAR);
}

TEST(FaultDetectorQueue, ConcurrentDrainLosesNothingUnaccounted) {
    FaultDetector d(test_clock());
    constexpr int kEmitted = 4000;

    tick_steer(d, steer_state(0x00, 4, true), true);

    std::atomic<bool> producing{true};
    std::atomic<std::size_t> drained{0};

    std::thread consumer([&]() {
        for (;;) {
            // Sample the flag *before* draining, so the final pass is
            // guaranteed to run after the producer has finished.
            const bool final_pass = !producing.load();
            drained += d.drain_events().size();
            if (final_pass) break;
            std::this_thread::yield();
        }
    });

    for (int i = 0; i < kEmitted; ++i) {
        const uint8_t code = (i % 2 == 0) ? steadywin_protocol::FAULT_TEMPERATURE : 0x00;
        tick_steer(d, steer_state(code, 4, true), true);
    }
    producing.store(false);
    consumer.join();

    drained += d.drain_events().size();

    // Every emitted event was either delivered or explicitly counted as dropped.
    EXPECT_EQ(drained.load() + d.dropped_events(), static_cast<std::size_t>(kEmitted));
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
