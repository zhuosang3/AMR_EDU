#include "power_monitor/mqtt_publisher.hpp"

#include <mosquitto.h>

MqttPublisher::MqttPublisher(const std::string& broker_host,
                             int broker_port,
                             const std::string& client_id)
    : host_(broker_host), port_(broker_port), client_id_(client_id) {}

MqttPublisher::~MqttPublisher() {
  if (mosq_) {
    ::mosquitto_disconnect(mosq_);
    ::mosquitto_loop_stop(mosq_, true);
    ::mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  }
  ::mosquitto_lib_cleanup();
}

bool MqttPublisher::init() {
  if (::mosquitto_lib_init() != MOSQ_ERR_SUCCESS) return false;

  mosq_ = ::mosquitto_new(client_id_.c_str(), true, nullptr);
  if (!mosq_) return false;

  int rc = ::mosquitto_connect(mosq_, host_.c_str(), port_, 60);
  if (rc != MOSQ_ERR_SUCCESS) {
    ::mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    return false;
  }

  rc = ::mosquitto_loop_start(mosq_);
  if (rc != MOSQ_ERR_SUCCESS) {
    ::mosquitto_disconnect(mosq_);
    ::mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    return false;
  }

  return true;
}

bool MqttPublisher::publish(const std::string& topic,
                            const std::string& payload,
                            int qos,
                            bool retain) {
  if (!mosq_) return false;
  int rc = ::mosquitto_publish(mosq_, nullptr, topic.c_str(),
                               static_cast<int>(payload.size()),
                               payload.data(), qos, retain);
  return rc == MOSQ_ERR_SUCCESS;
}

bool MqttPublisher::is_connected() const {
  return mosq_ != nullptr;
}
