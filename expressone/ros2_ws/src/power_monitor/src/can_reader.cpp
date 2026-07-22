#include "power_monitor/can_reader.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>

CanReader::CanReader(const std::string& iface) : iface_(iface) {}

CanReader::~CanReader() { stop(); }

bool CanReader::open() {
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    last_error_ = "socket(): " + std::string(std::strerror(errno));
    return false;
  }


  struct ifreq ifr = {};
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    last_error_ = "ioctl(SIOCGIFINDEX) " + iface_ + ": " + std::string(std::strerror(errno));
    ::close(socket_fd_); socket_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr = {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    last_error_ = "bind(): " + std::string(std::strerror(errno));
    ::close(socket_fd_); socket_fd_ = -1;
    return false;
  }

  return true;
}

void CanReader::start() {
  if (running_.load()) return;
  running_.store(true);
  auto* t = new std::thread(&CanReader::read_loop, this);
  thread_handle_ = t;
}

void CanReader::stop() {
  if (!running_.load()) return;
  running_.store(false);

  if (thread_handle_) {
    auto* t = static_cast<std::thread*>(thread_handle_);
    if (t->joinable()) t->join();
    delete t;
    thread_handle_ = nullptr;
  }

  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanReader::copy_out(BatteryData& dest) const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!has_data_ || !running_.load()) return false;
  dest = latest_;
  return true;
}

void CanReader::read_loop() {
  struct can_frame frame;
  BatteryData local;

  while (running_.load()) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(socket_fd_, &fds);
    struct timeval tv = {1, 0};  // 1s timeout

    int ret = ::select(socket_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) continue;
      last_error_ = "select(): " + std::string(std::strerror(errno));
      break;
    }
    if (ret == 0) continue;

    ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      if (n < 0) {
        last_error_ = "read(): " + std::string(std::strerror(errno));
        break;
      }
      continue;
    }

    parse_can_frame(frame.can_id & CAN_EFF_MASK, frame.data, frame.len, local);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_ = local;
      has_data_ = true;
    }
  }
}
