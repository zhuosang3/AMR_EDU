#pragma once

#include <string>

struct mosquitto;

class MqttPublisher {
public:
  explicit MqttPublisher(const std::string& broker_host = "localhost",
                         int broker_port = 1883,
                         const std::string& client_id = "power_monitor");
  ~MqttPublisher();

  MqttPublisher(const MqttPublisher&) = delete;
  MqttPublisher& operator=(const MqttPublisher&) = delete;
  MqttPublisher(MqttPublisher&&) = delete;
  MqttPublisher& operator=(MqttPublisher&&) = delete;

  bool init();
  bool publish(const std::string& topic, const std::string& payload,
               int qos = 0, bool retain = false);
  bool is_connected() const;

private:
  std::string host_;
  int port_;
  std::string client_id_;
  struct mosquitto* mosq_ = nullptr;
};
