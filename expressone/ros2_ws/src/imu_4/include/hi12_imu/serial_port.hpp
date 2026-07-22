#pragma once
#include <cstdint>
#include <string>
#include <chrono>

namespace hi12_imu {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    bool open(const std::string& device, int baudrate, int timeout_ms = 50);
    void close() noexcept;
    bool is_open() const noexcept { return _fd >= 0; }

    int read(uint8_t* buf, size_t len) noexcept;
    int read_byte() noexcept;
    int write(const uint8_t* data, size_t len) noexcept;
    void flush() noexcept;
    void flush_input() noexcept;

    int fd() const noexcept { return _fd; }
    int baudrate() const noexcept { return _baudrate; }
    const std::string& device() const noexcept { return _device; }
    size_t bytes_written() const noexcept { return _bytes_written; }
    size_t bytes_read() const noexcept { return _bytes_read; }

private:
    int         _fd = -1;
    int         _baudrate = 0;
    std::string _device;
    size_t      _bytes_written = 0;
    size_t      _bytes_read = 0;
};

} // namespace hi12_imu
