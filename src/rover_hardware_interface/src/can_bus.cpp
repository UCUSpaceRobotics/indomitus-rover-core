#include "rover_hardware_interface/can_bus.hpp"

#include <cstring>

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can/error.h>   // CAN_ERR_* classes and CAN_ERR_CRTL_* bits
#include <linux/can/raw.h>

#ifndef CAN_ERR_CRTL_ACTIVE
#define CAN_ERR_CRTL_ACTIVE 0x40
#endif

namespace rover_hardware_interface {

namespace {

constexpr uint8_t kCtrlPassiveBits = CAN_ERR_CRTL_TX_PASSIVE | CAN_ERR_CRTL_RX_PASSIVE;
constexpr uint8_t kCtrlWarningBits = CAN_ERR_CRTL_TX_WARNING | CAN_ERR_CRTL_RX_WARNING;
constexpr uint8_t kCtrlOverflowBits = CAN_ERR_CRTL_RX_OVERFLOW | CAN_ERR_CRTL_TX_OVERFLOW;

}  // namespace

CanBus::CanBus(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
    : logger_(logger), clock_(clock) {}

CanBus::~CanBus() {
    close();
}

bool CanBus::open(const std::string& interface_name)
{
    interface_name_ = interface_name;

    const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        RCLCPP_ERROR(logger_, "[CanBus] socket failed: %s", std::strerror(errno));
        return false;
    }

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_LOSTARB | CAN_ERR_CRTL |
        CAN_ERR_PROT | CAN_ERR_TRX | CAN_ERR_ACK | CAN_ERR_BUSOFF  |
        CAN_ERR_BUSERROR | CAN_ERR_RESTARTED;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    // Bind socket to the named CAN interface
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] ioctl SIOCGIFINDEX failed for '%s': %s",
            interface_name_.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] bind() failed: %s", std::strerror(errno));
        ::close(fd);
        return false;
    }

    // Disable loopback so we don't receive our own TX frames
    int loopback = 0;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback));

    // Non-blocking in both directions
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] fcntl(O_NONBLOCK) failed: %s", std::strerror(errno));
        ::close(fd);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        fd_.store(fd);
    }

    RCLCPP_INFO(logger_,
        "[CanBus] SocketCAN opened: %s (fd=%d)", interface_name_.c_str(), fd);
    return true;
}

void CanBus::close()
{
    stop_rx();

    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        fd = fd_.exchange(-1);
    }

    if (fd >= 0) {
        ::close(fd);
        RCLCPP_INFO(logger_, "[CanBus] SocketCAN closed.");
    }
}

bool CanBus::start_rx(std::function<void(const struct can_frame &)> on_frame)
{
    // Restart rather than move-assign onto a live thread, which terminates.
    if (rx_thread_.joinable()) {
        stop_rx();
    }

    const int stop_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (stop_fd < 0) {
        RCLCPP_ERROR(logger_,
            "[CanBus] eventfd() failed, RX thread not started: %s", std::strerror(errno));
        return false;
    }
    stop_fd_.store(stop_fd);

    rx_running_.store(true);
    rx_thread_ = std::thread(&CanBus::rx_thread_fn, this, std::move(on_frame));
    return true;
}

void CanBus::stop_rx()
{
    rx_running_.store(false);

    // Wake the thread out of poll(). Clearing the flag alone is not enough:
    // the thread only re-tests it after poll() returns, and on a silent bus
    // that would be never.
    const int stop_fd = stop_fd_.load();
    if (stop_fd >= 0) {
        const uint64_t one = 1;
        ssize_t written;
        do {
            written = ::write(stop_fd, &one, sizeof(one));
        } while (written < 0 && errno == EINTR);
    }

    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }

    const int to_close = stop_fd_.exchange(-1);
    if (to_close >= 0) {
        ::close(to_close);
    }
}

CanBus::SendResult CanBus::send(uint32_t id, const uint8_t * data, uint8_t dlc, bool is_extended)
{
    struct can_frame frame{};
    frame.can_id  = is_extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    // The lock spans the fd check and the write so that close() — which takes
    // the same lock — cannot invalidate the descriptor between them.
    std::lock_guard<std::mutex> lock(tx_mutex_);

    // Socket not open — same practical meaning to a caller as the bus being
    // gone: there is no transport.
    const int fd = fd_.load();
    if (fd < 0) return SendResult::BUS_DOWN;

    const ssize_t nbytes = ::write(fd, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
        return SendResult::OK;
    }

    const int err = (nbytes < 0) ? errno : 0;

    if (err == ENOBUFS || err == EAGAIN || err == EWOULDBLOCK) {
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
    if (err == ENETDOWN) {
        bus_state_.store(BusState::BUS_OFF);
        return SendResult::BUS_DOWN;
    }

    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "[CanBus] send id=0x%X failed: %s", id, std::strerror(err));
    return SendResult::ERROR;
}

// rx_thread_fn — poll/read receive loop
//
// Runs in a dedicated thread so it doesn't block the control loop. Waits in
// poll() on the socket plus a stop eventfd, which is what makes shutdown
// bounded on a silent bus. Error frames update bus_state_/error counters here;
// everything else is handed to `on_frame` for the caller to decode.

void CanBus::rx_thread_fn(std::function<void(const struct can_frame &)> on_frame)
{
    struct can_frame frame{};

    while (rx_running_.load()) {
        const int fd = fd_.load();
        const int stop_fd = stop_fd_.load();
        if (fd < 0 || stop_fd < 0) break;

        struct pollfd fds[2];
        fds[0].fd = fd;       fds[0].events = POLLIN; fds[0].revents = 0;
        fds[1].fd = stop_fd;  fds[1].events = POLLIN; fds[1].revents = 0;

        const int ready = ::poll(fds, 2, -1);
        if (ready < 0) {
            if (errno == EINTR) continue;
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                "[CanBus] poll() failed: %s", std::strerror(errno));
            continue;
        }

        // Stop requested — checked before the socket so shutdown wins a tie.
        if (fds[1].revents & POLLIN) break;

        if (!(fds[0].revents & (POLLIN | POLLERR | POLLHUP))) continue;

        // Drain everything the kernel has queued before polling again: one
        // wakeup can cover several frames, and at eight motors on a shared bus
        // the queue is routinely non-empty.
        while (rx_running_.load()) {
            const ssize_t nbytes = ::read(fd, &frame, sizeof(frame));

            if (nbytes < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // queue drained
                RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                    "[CanBus] CAN recv error: %s", std::strerror(errno));
                break;
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
}

void CanBus::on_can_error(const struct can_frame & frame)
{
    // frame.data[1] holds controller-problem bits when CAN_ERR_CRTL is set
    if (frame.can_id & CAN_ERR_BUSOFF) {
        bus_state_.store(BusState::BUS_OFF);
        RCLCPP_ERROR(logger_, "[CanBus] CAN bus-off detected on %s", interface_name_.c_str());
    } else if (frame.can_id & CAN_ERR_CRTL) {
        const uint8_t ctrl = frame.data[1];

        if (ctrl & kCtrlPassiveBits) {
            if (bus_state_.exchange(BusState::ERROR_PASSIVE) != BusState::ERROR_PASSIVE) {
                RCLCPP_WARN(logger_, "[CanBus] CAN controller error-passive");
            }
        } else if (ctrl & kCtrlWarningBits) {
            bus_state_.store(BusState::ERROR_WARNING);
        } else if (ctrl & CAN_ERR_CRTL_ACTIVE) {
            if (bus_state_.exchange(BusState::OK) != BusState::OK) {
                RCLCPP_INFO(logger_, "[CanBus] CAN controller back to error-active");
            }
        }

        if (ctrl & kCtrlOverflowBits) {
            rx_overflow_.fetch_add(1);
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                "[CanBus] CAN queue overflow — %d total", rx_overflow_.load());
        }

        tx_error_count_.store(frame.data[6]);
        rx_error_count_.store(frame.data[7]);
    } else if (frame.can_id & CAN_ERR_RESTARTED) {
        bus_state_.store(BusState::OK);
        RCLCPP_INFO(logger_, "[CanBus] CAN interface auto-restarted, bus recovered");
    }
}

}  // namespace rover_hardware_interface
