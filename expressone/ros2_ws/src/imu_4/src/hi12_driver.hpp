#pragma once
#include "hi12_imu/protocol.hpp"
#include "hi12_imu/serial_port.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace hi12_imu {

using ImuDataCallback = std::function<void(const protocol::ImuData&)>;

class HI12Driver {
public:
    HI12Driver() = default;
    ~HI12Driver();

    HI12Driver(const HI12Driver&) = delete;
    HI12Driver& operator=(const HI12Driver&) = delete;

    bool open(const std::string& device, int baudrate, ImuDataCallback callback);
    void close() noexcept;
    bool is_running() const noexcept { return _running.load(std::memory_order_acquire); }

    /// Send configuration commands to the HI12 sensor
    bool configure(int new_baudrate = -1, int output_rate_hz = -1, uint16_t output_mask = 0);
    bool save_config();
    bool factory_reset();

    struct Stats {
        uint64_t valid_frames  = 0;
        uint64_t crc_errors    = 0;
        uint64_t parse_errors  = 0;
        uint64_t serial_errors = 0;
        double   uptime_sec    = 0.0;
        double   effective_hz  = 0.0;
        size_t   bytes_read    = 0;
        int      baudrate      = 0;
        std::string device;
    };
    Stats stats() const noexcept;

private:
    void reader_loop() noexcept;

    SerialPort                _port;
    protocol::FrameParser     _parser;
    ImuDataCallback           _callback;
    std::unique_ptr<std::thread> _thread;
    std::atomic<bool>         _running{false};

    /// Saved for auto-reconnect on serial error / hotplug
    std::string               _device;
    int                       _baudrate = 0;

    std::atomic<uint64_t> _valid_frames{0};
    std::atomic<uint64_t> _crc_errors{0};
    std::atomic<uint64_t> _parse_errors{0};
    std::atomic<uint64_t> _serial_errors{0};
    double _start_time = 0.0;
};

} // namespace hi12_imu
