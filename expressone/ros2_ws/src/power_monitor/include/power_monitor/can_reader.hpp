#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include "power_monitor/battery_protocol.hpp"

class CanReader {
public:
  explicit CanReader(const std::string& iface);
  ~CanReader();

  CanReader(const CanReader&) = delete;
  CanReader& operator=(const CanReader&) = delete;
  CanReader(CanReader&&) = delete;
  CanReader& operator=(CanReader&&) = delete;

  bool open();
  void start();
  void stop();
  bool copy_out(BatteryData& dest) const;
  bool is_running() const { return running_.load(); }
  const std::string& last_error() const { return last_error_; }

private:
  void read_loop();

  std::string iface_;
  int socket_fd_ = -1;
  std::atomic<bool> running_{false};
  mutable std::mutex data_mutex_;
  BatteryData latest_;
  bool has_data_ = false;
  void* thread_handle_ = nullptr;
  std::string last_error_;
};
