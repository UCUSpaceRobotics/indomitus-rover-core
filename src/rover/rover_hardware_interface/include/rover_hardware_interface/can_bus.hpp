#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"

namespace rover_hardware_interface {


class CanBus
{
public:
    enum class SendResult { OK, WOULD_BLOCK, BUS_DOWN, ERROR };
    enum class BusState { OK, ERROR_WARNING, ERROR_PASSIVE, BUS_OFF };

    CanBus(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
    ~CanBus();

    CanBus(const CanBus &) = delete;
    CanBus & operator=(const CanBus &) = delete;

    bool open(const std::string & interface_name);
    void close();

    bool is_open() const { return fd_.load() >= 0; }

    bool start_rx(std::function<void(const struct can_frame &)> on_frame);
    void stop_rx();

    SendResult send(uint32_t id, const uint8_t * data, uint8_t dlc, bool is_extended = false);

    BusState state() const { return bus_state_.load(); }
    int tx_error_count() const { return tx_error_count_.load(); }
    int rx_error_count() const { return rx_error_count_.load(); }
    int tx_dropped() const { return tx_dropped_.load(); }
    int rx_overflow() const { return rx_overflow_.load(); }

private:
    void rx_thread_fn(std::function<void(const struct can_frame &)> on_frame);
    void on_can_error(const struct can_frame & frame);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    std::string interface_name_;

    /// Only assigned under tx_mutex_; see the class comment.
    std::atomic<int> fd_{-1};
    std::mutex tx_mutex_;

    /// Wakes the RX thread out of poll() at shutdown. Owned for the lifetime of
    /// the RX thread only.
    std::atomic<int> stop_fd_{-1};

    std::atomic<bool> rx_running_{false};
    std::thread rx_thread_;

    std::atomic<BusState> bus_state_{BusState::OK};
    std::atomic<int> tx_error_count_{0};
    std::atomic<int> rx_error_count_{0};
    std::atomic<int> tx_dropped_{0};   ///< frames lost to a full TX queue
    std::atomic<int> rx_overflow_{0};  ///< frames lost to a full kernel RX queue
};

}  // namespace rover_hardware_interface
