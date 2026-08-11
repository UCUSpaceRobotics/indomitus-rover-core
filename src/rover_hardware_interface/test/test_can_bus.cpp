// CanBus lifecycle against a real SocketCAN interface.
//
// These need a CAN interface to bind to and are skipped when none is present,
// so they run on a developer machine with
//
//     sudo modprobe vcan && sudo ip link add dev vcan0 type vcan
//     sudo ip link set up vcan0
//
// and quietly skip in a container that has no networking privileges. Set
// ROVER_TEST_CAN_IFACE to use something other than vcan0.
//
// The shutdown test is the reason this file exists: a silent bus is the normal
// state at deactivation (the motors have just been told to stop), and that is
// exactly the case where a receive loop that waits in a bare blocking read()
// never comes back.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rover_hardware_interface/can_bus.hpp"

using rover_hardware_interface::CanBus;
using namespace std::chrono_literals;

namespace {

std::string test_interface()
{
    const char * env = std::getenv("ROVER_TEST_CAN_IFACE");
    return env ? std::string(env) : std::string("vcan0");
}

std::unique_ptr<CanBus> make_bus()
{
    return std::make_unique<CanBus>(
        rclcpp::get_logger("test_can_bus"),
        std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
}

}  // namespace

class CanBusTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        bus_ = make_bus();
        if (!bus_->open(test_interface())) {
            GTEST_SKIP() << "no CAN interface '" << test_interface() << "' available";
        }
    }

    std::unique_ptr<CanBus> bus_;
};

TEST_F(CanBusTest, StopRxReturnsOnASilentBus) {
    std::atomic<int> frames{0};
    bus_->start_rx([&frames](const struct can_frame &) { ++frames; });

    // Nothing is transmitting, so the RX thread is parked with no traffic to
    // wake it. A stop that depends on the next frame arriving never returns.
    std::promise<void> stopped;
    auto done = stopped.get_future();

    CanBus * raw = bus_.get();
    std::thread stopper([raw, &stopped]() {
        raw->stop_rx();
        stopped.set_value();
    });

    const bool finished = done.wait_for(2s) == std::future_status::ready;
    if (finished) {
        stopper.join();
    } else {
        // The thread is wedged inside stop_rx(). Detach it and deliberately
        // leak the bus so the process can still report the failure instead of
        // hanging the whole test run or crashing on a freed object.
        stopper.detach();
        bus_.release();
    }

    ASSERT_TRUE(finished) << "stop_rx() blocked with no CAN traffic to unblock it";
    EXPECT_EQ(frames.load(), 0);
}

TEST_F(CanBusTest, CloseWithoutStopRxIsSafe) {
    bus_->start_rx([](const struct can_frame &) {});

    std::promise<void> closed;
    auto done = closed.get_future();

    CanBus * raw = bus_.get();
    std::thread closer([raw, &closed]() {
        raw->close();          // must stop the RX thread itself
        closed.set_value();
    });

    const bool finished = done.wait_for(2s) == std::future_status::ready;
    if (finished) {
        closer.join();
    } else {
        closer.detach();
        bus_.release();
    }

    ASSERT_TRUE(finished) << "close() blocked waiting for the RX thread";
    EXPECT_FALSE(bus_->is_open());
}

TEST_F(CanBusTest, RepeatedOpenCloseCycles) {
    // A hardware component gets activated and deactivated repeatedly over a
    // session; each cycle must leave the object usable, not leak a thread and
    // not terminate on a re-started RX thread.
    bus_->close();

    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(bus_->open(test_interface())) << "cycle " << i;
        EXPECT_TRUE(bus_->is_open());
        bus_->start_rx([](const struct can_frame &) {});
        bus_->close();
        EXPECT_FALSE(bus_->is_open());
    }
}

TEST_F(CanBusTest, StopRxIsIdempotent) {
    bus_->start_rx([](const struct can_frame &) {});
    bus_->stop_rx();
    bus_->stop_rx();   // no thread left to join, no eventfd left to write
    SUCCEED();
}

TEST_F(CanBusTest, SendOnAClosedBusReportsBusDownRatherThanCrashing) {
    bus_->close();

    const uint8_t payload[1] = {0xAE};
    EXPECT_EQ(bus_->send(0x0B, payload, 1), CanBus::SendResult::BUS_DOWN);
}

TEST_F(CanBusTest, SendSucceedsOnAnOpenBus) {
    const uint8_t payload[1] = {0xAE};
    EXPECT_EQ(bus_->send(0x0B, payload, 1), CanBus::SendResult::OK);
    EXPECT_EQ(bus_->tx_dropped(), 0);
}

TEST_F(CanBusTest, LoopbackFramesReachTheCallback) {
    // Two sockets on the same vcan: what one sends, the other receives. This
    // is also the only cheap check that the non-blocking RX path actually
    // drains the queue rather than waiting for a second wakeup per frame.
    auto sender = make_bus();
    ASSERT_TRUE(sender->open(test_interface()));

    std::atomic<int> received{0};
    bus_->start_rx([&received](const struct can_frame &) { ++received; });

    constexpr int kFrames = 10;
    const uint8_t payload[1] = {0xAE};
    for (int i = 0; i < kFrames; ++i) {
        ASSERT_EQ(sender->send(0x0B, payload, 1), CanBus::SendResult::OK);
    }

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (received.load() < kFrames && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
    }

    bus_->stop_rx();
    sender->close();

    EXPECT_EQ(received.load(), kFrames);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
