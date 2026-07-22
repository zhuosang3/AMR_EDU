#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "power_monitor/battery_protocol.hpp"
#include "power_monitor/can_reader.hpp"
#include "power_monitor/mqtt_publisher.hpp"

using namespace std::chrono_literals;

class PowerMonitorNode : public rclcpp::Node {
public:
  PowerMonitorNode() : Node("power_monitor_node") {
    declare_parameter("can_interface", "can0");
    declare_parameter("mqtt_host", "localhost");
    declare_parameter("mqtt_port", 1883);
    declare_parameter("mqtt_topic", "/car/power");
    declare_parameter("publish_interval_s", 5);

    can_iface_  = get_parameter("can_interface").as_string();
    mqtt_host_  = get_parameter("mqtt_host").as_string();
    mqtt_port_  = get_parameter("mqtt_port").as_int();
    mqtt_topic_ = get_parameter("mqtt_topic").as_string();
    int interval_s = get_parameter("publish_interval_s").as_int();

    RCLCPP_INFO(get_logger(), "PowerMonitor starting — CAN:%s MQTT:%s:%d topic:%s",
                can_iface_.c_str(), mqtt_host_.c_str(), mqtt_port_, mqtt_topic_.c_str());

    // Open CAN
    can_reader_ = std::make_unique<CanReader>(can_iface_);
    if (!can_reader_->open()) {
      RCLCPP_ERROR(get_logger(), "CAN open failed: %s", can_reader_->last_error().c_str());
    } else {
      can_reader_->start();
      RCLCPP_INFO(get_logger(), "CAN reader started on %s", can_iface_.c_str());
    }

    // Init MQTT
    mqtt_ = std::make_unique<MqttPublisher>(mqtt_host_, mqtt_port_,
                                            "power_monitor_" + std::to_string(getpid()));
    if (!mqtt_->init()) {
      RCLCPP_ERROR(get_logger(), "MQTT init failed — will retry on timer");
    } else {
      RCLCPP_INFO(get_logger(), "MQTT connected to %s:%d", mqtt_host_.c_str(), mqtt_port_);
    }

    // 5-second timer
    timer_ = create_wall_timer(std::chrono::seconds(interval_s),
                               [this] { timer_callback(); });

    RCLCPP_INFO(get_logger(), "PowerMonitor node READY (publish every %ds)", interval_s);
  }

  ~PowerMonitorNode() override {
    if (can_reader_) can_reader_->stop();
  }

private:
  void timer_callback() {
    BatteryData data;
    if (!can_reader_ || !can_reader_->copy_out(data)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                           "No CAN data yet — waiting for BMS frames...");
      return;
    }

    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "{\"power\":%u,\"charge_flag\":%u,\"current\":%d}",
                  data.rsoc, data.charge_flag, data.current);

    if (mqtt_ && mqtt_->is_connected()) {
      if (mqtt_->publish(mqtt_topic_, buf)) {
        RCLCPP_INFO(get_logger(), "MQTT → %s  %s", mqtt_topic_.c_str(), buf);
      } else {
        RCLCPP_WARN(get_logger(), "MQTT publish failed");
      }
    } else {
      // Try to reconnect
      if (mqtt_) {
        mqtt_.reset();
        mqtt_ = std::make_unique<MqttPublisher>(mqtt_host_, mqtt_port_,
                                                "power_monitor_" + std::to_string(getpid()));
        if (mqtt_->init()) {
          RCLCPP_INFO(get_logger(), "MQTT reconnected");
        }
      }
    }
  }

  std::string can_iface_, mqtt_host_, mqtt_topic_;
  int mqtt_port_;
  std::unique_ptr<CanReader> can_reader_;
  std::unique_ptr<MqttPublisher> mqtt_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PowerMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
