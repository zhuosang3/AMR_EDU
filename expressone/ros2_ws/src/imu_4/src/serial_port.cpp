#include "hi12_imu/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace hi12_imu {

SerialPort::SerialPort(SerialPort&& other) noexcept
    : _fd(other._fd), _baudrate(other._baudrate), _device(std::move(other._device))
    , _bytes_written(other._bytes_written), _bytes_read(other._bytes_read) {
    other._fd = -1;
    other._baudrate = 0;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        close();
        _fd = other._fd; _baudrate = other._baudrate; _device = std::move(other._device);
        _bytes_written = other._bytes_written; _bytes_read = other._bytes_read;
        other._fd = -1; other._baudrate = 0;
    }
    return *this;
}

static speed_t baud_to_speed(int baud) {
    switch (baud) {
    case 4800:    return B4800;    case 9600:    return B9600;
    case 19200:   return B19200;   case 38400:   return B38400;
    case 57600:   return B57600;   case 115200:  return B115200;
    case 230400:  return B230400;  case 460800:  return B460800;
    case 500000:  return B500000;  case 576000:  return B576000;
    case 921600:  return B921600;  case 1000000: return B1000000;
    case 1152000: return B1152000; case 1500000: return B1500000;
    case 2000000: return B2000000;
    default:      return B115200;
    }
}

bool SerialPort::open(const std::string& device, int baudrate, int timeout_ms) {
    close();
    _fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (_fd < 0) return false;

    if (ioctl(_fd, TIOCEXCL) < 0) { ::close(_fd); _fd = -1; return false; }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(_fd, &tty) != 0) { ::close(_fd); _fd = -1; return false; }

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_iflag |= IGNPAR;
    tty.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    tty.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);

    speed_t speed = baud_to_speed(baudrate);
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
        ::close(_fd); _fd = -1; return false;
    }

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = static_cast<cc_t>((timeout_ms + 99) / 100);
    if (tty.c_cc[VTIME] == 0 && timeout_ms > 0) tty.c_cc[VTIME] = 1;

    if (tcsetattr(_fd, TCSANOW, &tty) != 0) { ::close(_fd); _fd = -1; return false; }
    tcflush(_fd, TCIOFLUSH);

    _device = device; _baudrate = baudrate; _bytes_read = 0; _bytes_written = 0;
    return true;
}

void SerialPort::close() noexcept {
    if (_fd >= 0) { tcflush(_fd, TCIOFLUSH); ::close(_fd); _fd = -1; }
    _baudrate = 0; _device.clear();
}

int SerialPort::read(uint8_t* buf, size_t len) noexcept {
    if (_fd < 0 || buf == nullptr || len == 0) return -1;
    ssize_t n = ::read(_fd, buf, len);
    if (n > 0) { _bytes_read += static_cast<size_t>(n); return static_cast<int>(n); }
    if (n == 0) return 0;
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    return -1;
}

int SerialPort::read_byte() noexcept {
    if (_fd < 0) return -1;
    uint8_t b;
    int n = read(&b, 1);
    return (n == 1) ? static_cast<int>(b) : -1;
}

int SerialPort::write(const uint8_t* data, size_t len) noexcept {
    if (_fd < 0 || data == nullptr || len == 0) return -1;
    ssize_t n = ::write(_fd, data, len);
    if (n > 0) { _bytes_written += static_cast<size_t>(n); return static_cast<int>(n); }
    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    return -1;
}

void SerialPort::flush() noexcept { if (_fd >= 0) tcdrain(_fd); }
void SerialPort::flush_input() noexcept { if (_fd >= 0) tcflush(_fd, TCIFLUSH); }

} // namespace hi12_imu
