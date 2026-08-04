#include "rover_hardware_interface/can_bus.hpp"

#include <cstring>

#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can/error.h>   // CAN_ERR_* classes and CAN_ERR_CRTL_* bits
#include <linux/can/raw.h>

namespace rover_hardware_interface {

CanBus::CanBus(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
    : logger_(logger), clock_(clock)
{
}

CanBus::~CanBus()
{
    stop_rx();
    close();
}

bool CanBus::open(const std::string & interface_name)
{
    interface_name_ = interface_name;

    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] socket(PF_CAN) failed: %s", std::strerror(errno));
        return false;
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL |
        CAN_ERR_PROT | CAN_ERR_TRX | CAN_ERR_ACK | CAN_ERR_BUSOFF  |
        CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    // Bind to the named CAN interface
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] ioctl SIOCGIFINDEX failed for '%s': %s",
            interface_name_.c_str(), std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] bind() failed: %s", std::strerror(errno));
        ::close(fd_); fd_ = -1;
        return false;
    }

    // Disable loopback so we don't receive our own TX frames
    int loopback = 0;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback));

    // rx_thread uses blocking recv() — no need to set O_NONBLOCK on the fd
    // send() is non-blocking by nature for CAN frames

    RCLCPP_INFO(logger_,
        "[CanBus] SocketCAN opened: %s (fd=%d)", interface_name_.c_str(), fd_);
    return true;
}

void CanBus::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        RCLCPP_INFO(logger_, "[CanBus] SocketCAN closed.");
    }
}

void CanBus::start_rx(std::function<void(const struct can_frame &)> on_frame)
{
    rx_running_.store(true);
    rx_thread_ = std::thread(&CanBus::rx_thread_fn, this, std::move(on_frame));
}

void CanBus::stop_rx()
{
    rx_running_.store(false);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

// Return type must be qualified: a leading return type on an out-of-class
// member definition is not looked up in class scope, and SendResult is a
// nested type.
CanBus::SendResult CanBus::send(uint32_t id, const uint8_t * data, uint8_t dlc, bool is_extended)
{
    // Socket not open — same practical meaning to a caller as the bus being
    // gone: there is no transport.
    if (fd_ < 0) return SendResult::BUS_DOWN;

    struct can_frame frame{};
    frame.can_id  = is_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    std::lock_guard<std::mutex> lock(tx_mutex_);
    const ssize_t nbytes = ::write(fd_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
        return SendResult::OK;
    }

    if (errno == ENOBUFS || errno == EAGAIN) {
        // TX queue full. Transient, but not nothing: a sustained burst of these
        // means frames are being dropped, and 200 ms of dropped frames is
        // exactly what trips the Damiao TIMEOUT register into a comm-loss
        // fault. Silence here would hide the cause of the fault we then report.
        tx_dropped_.fetch_add(1);
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
            "[CanBus] CAN TX queue full (id=0x%X), frame dropped — %d total",
            id, tx_dropped_.load());
        return SendResult::WOULD_BLOCK;
    }
    if (errno == ENETDOWN) {
        bus_state_.store(BusState::BUS_OFF);
        return SendResult::BUS_DOWN;
    }

    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "[CanBus] send id=0x%X failed: %s", id, std::strerror(errno));
    return SendResult::ERROR;
}

// rx_thread_fn — blocking receive loop
//
// Runs in a dedicated thread so it doesn't block the control loop.
// Error frames update bus_state_/error counters here; everything else is
// handed to `on_frame` for the caller to decode.

void CanBus::rx_thread_fn(std::function<void(const struct can_frame &)> on_frame)
{
    struct can_frame frame{};

    while (rx_running_.load()) {
        const ssize_t nbytes = ::read(fd_, &frame, sizeof(frame));

        if (nbytes < 0) {
            if (errno == EINTR) continue;  // interrupted by signal — retry
            if (!rx_running_.load()) break; // socket closed during shutdown
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                "[CanBus] CAN recv error: %s", std::strerror(errno));
            continue;
        }

        if (nbytes != static_cast<ssize_t>(sizeof(frame))) continue;

        if (frame.can_id & CAN_ERR_FLAG) {
            on_can_error(frame);
            continue;  // don't pass error frames to on_frame
        }

        // Strip flags from CAN ID before dispatch
        const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
        frame.can_id = raw_id;

        on_frame(frame);
    }
}

void CanBus::on_can_error(const struct can_frame & frame)
{
    // frame.data[1] holds controller-problem bits when CAN_ERR_CRTL is set
    if (frame.can_id & CAN_ERR_BUSOFF) {
        bus_state_.store(BusState::BUS_OFF);
        RCLCPP_ERROR(logger_, "[CanBus] CAN bus-off detected on %s", interface_name_.c_str());
    } else if (frame.can_id & CAN_ERR_CRTL) {
        if (frame.data[1] & CAN_ERR_CRTL_TX_PASSIVE ||
            frame.data[1] & CAN_ERR_CRTL_RX_PASSIVE) {
            bus_state_.store(BusState::ERROR_PASSIVE);
            RCLCPP_WARN(logger_, "[CanBus] CAN controller error-passive");
        } else if (frame.data[1] & CAN_ERR_CRTL_TX_WARNING ||
                   frame.data[1] & CAN_ERR_CRTL_RX_WARNING) {
            bus_state_.store(BusState::ERROR_WARNING);
        }
        tx_error_count_.store(frame.data[6]);
        rx_error_count_.store(frame.data[7]);
    } else if (frame.can_id & CAN_ERR_RESTARTED) {
        bus_state_.store(BusState::OK);
        RCLCPP_INFO(logger_, "[CanBus] CAN interface auto-restarted, bus recovered");
    }
}

}  // namespace rover_hardware_interface
