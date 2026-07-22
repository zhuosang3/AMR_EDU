#include "hi12_imu/hi12_driver.hpp"
#include <chrono>
#include <cstring>

namespace hi12_imu {

HI12Driver::~HI12Driver() { close(); }

bool HI12Driver::open(const std::string& device, int baudrate, ImuDataCallback callback) {
    if (is_running() || !callback) return false;
    if (!_port.open(device, baudrate, 50)) return false;

    _device   = device;
    _baudrate = baudrate;
    _callback = std::move(callback);
    _parser.reset();
    _running.store(true, std::memory_order_release);

    _start_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    _thread = std::make_unique<std::thread>(&HI12Driver::reader_loop, this);
    return true;
}

void HI12Driver::close() noexcept {
    if (!_running.load(std::memory_order_acquire)) return;
    _running.store(false, std::memory_order_release);
    _port.close();
    if (_thread && _thread->joinable()) _thread->join();
    _thread.reset();
    _callback = nullptr;
}

bool HI12Driver::configure(int new_baudrate, int output_rate_hz, uint16_t output_mask) {
    if (!_port.is_open()) return false;
    bool ok = true;

    if (new_baudrate > 0) {
        uint8_t code = protocol::baud_to_code(new_baudrate);
        if (code == 0xFF) return false;
        uint8_t buf[protocol::FRAME_HEADER_SIZE + 2];
        size_t len = protocol::build_config_frame(protocol::CfgReg::BAUD_RATE, &code, 1, buf);
        ok &= (_port.write(buf, len) == static_cast<int>(len));
    }
    if (output_rate_hz > 0) {
        uint8_t rate[2] = {
            static_cast<uint8_t>(output_rate_hz & 0xFF),
            static_cast<uint8_t>((output_rate_hz >> 8) & 0xFF)};
        uint8_t buf[protocol::FRAME_HEADER_SIZE + 3];
        size_t len = protocol::build_config_frame(protocol::CfgReg::OUTPUT_RATE, rate, 2, buf);
        ok &= (_port.write(buf, len) == static_cast<int>(len));
    }
    if (output_mask != 0) {
        uint8_t mask[2] = {
            static_cast<uint8_t>(output_mask & 0xFF),
            static_cast<uint8_t>((output_mask >> 8) & 0xFF)};
        uint8_t buf[protocol::FRAME_HEADER_SIZE + 3];
        size_t len = protocol::build_config_frame(protocol::CfgReg::OUTPUT_SELECT, mask, 2, buf);
        ok &= (_port.write(buf, len) == static_cast<int>(len));
    }
    _port.flush();
    return ok;
}

bool HI12Driver::save_config() {
    if (!_port.is_open()) return false;
    uint8_t v = 0x01;
    uint8_t buf[protocol::FRAME_HEADER_SIZE + 2];
    size_t len = protocol::build_config_frame(protocol::CfgReg::SAVE_CONFIG, &v, 1, buf);
    bool ok = (_port.write(buf, len) == static_cast<int>(len));
    _port.flush();
    return ok;
}

bool HI12Driver::factory_reset() {
    if (!_port.is_open()) return false;
    uint8_t v = 0x01;
    uint8_t buf[protocol::FRAME_HEADER_SIZE + 2];
    size_t len = protocol::build_config_frame(protocol::CfgReg::RESET_FACTORY, &v, 1, buf);
    bool ok = (_port.write(buf, len) == static_cast<int>(len));
    _port.flush();
    return ok;
}

void HI12Driver::reader_loop() noexcept {
    uint64_t frame_count = 0;
    constexpr int reconnect_delay_ms = 2000;

    while (_running.load(std::memory_order_acquire)) {
        int b = _port.read_byte();
        if (b < 0) {
            if (_running.load(std::memory_order_acquire))
                _serial_errors.fetch_add(1, std::memory_order_relaxed);

            // Auto-reconnect loop
            _port.close();
            while (_running.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(reconnect_delay_ms));
                if (!_running.load(std::memory_order_acquire)) break;
                if (_port.open(_device, _baudrate, 50)) {
                    _parser.reset();
                    break;  // reconnected, back to reading
                }
            }
            continue;  // either reconnected or shutting down
        }

        const uint8_t* payload = nullptr;
        size_t payload_len = 0;
        bool valid = _parser.feed(static_cast<uint8_t>(b), &payload, &payload_len);

        if (valid) {
            protocol::ImuData data;
            data.frame_id = ++frame_count;
            data.timestamp = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

            if (protocol::parse_payload(payload, payload_len, data)) {
                _valid_frames.fetch_add(1, std::memory_order_relaxed);
                try {
                    _callback(data);
                } catch (...) {}
            } else {
                _parse_errors.fetch_add(1, std::memory_order_relaxed);
            }
        } else if (_parser.bytes_consumed() >= protocol::FRAME_HEADER_SIZE) {
            _crc_errors.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

HI12Driver::Stats HI12Driver::stats() const noexcept {
    Stats s;
    s.valid_frames  = _valid_frames.load(std::memory_order_relaxed);
    s.crc_errors    = _crc_errors.load(std::memory_order_relaxed);
    s.parse_errors  = _parse_errors.load(std::memory_order_relaxed);
    s.serial_errors = _serial_errors.load(std::memory_order_relaxed);
    s.baudrate      = _port.baudrate();
    s.device        = _port.device();
    s.bytes_read    = _port.bytes_read();

    auto now = std::chrono::steady_clock::now();
    s.uptime_sec = std::chrono::duration<double>(now.time_since_epoch()).count() - _start_time;
    if (s.uptime_sec > 0.0)
        s.effective_hz = static_cast<double>(s.valid_frames) / s.uptime_sec;
    return s;
}

} // namespace hi12_imu
