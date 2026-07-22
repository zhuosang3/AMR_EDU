/**
 * @file hi12_node.cpp
 * @brief ROS2 node for HI12 IMU — publishes sensor_msgs/Imu.
 *
 * Topics:
 *   /imu   — sensor_msgs::msg::Imu (orientation, gyro, accel)
 *
 * Data is parsed from the HI12 fixed-struct 76-byte payload.
 * Conversions (G→m/s², °/s→rad/s) are done in protocol::parse_payload().
 */

#include "hi12_imu/hi12_driver.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class HI12Node : public rclcpp::Node {
public:
    explicit HI12Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("hi12_imu", options)
    {
        device_     = declare_parameter("device", "/dev/ttyS4");
        baudrate_   = declare_parameter("baudrate", 115200);
        frame_id_   = declare_parameter("frame_id", "imu_link");
        publish_hz_ = declare_parameter("publish_hz", 200);

        double gyro_cov  = declare_parameter("angular_velocity_covariance", 0.01);
        double accel_cov = declare_parameter("linear_acceleration_covariance", 0.1);
        gyro_covariance_  = gyro_cov;
        accel_covariance_ = accel_cov;

        // Publisher — sensor data QoS
        auto qos = rclcpp::QoS(10).best_effort().durability_volatile();
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", qos);

        // Driver
        driver_ = std::make_unique<hi12_imu::HI12Driver>();
        bool ok = driver_->open(device_, baudrate_,
            [this](const hi12_imu::protocol::ImuData& data) {
                this->on_imu_data(data);
            });

        if (!ok) {
            RCLCPP_FATAL(get_logger(),
                "Failed to open HI12 on %s at %d baud", device_.c_str(), baudrate_);
            throw std::runtime_error("HI12 serial open failed");
        }

        RCLCPP_INFO(get_logger(),
            "HI12 IMU node started: device=%s, baud=%d, frame=%s",
            device_.c_str(), baudrate_, frame_id_.c_str());

        stats_timer_ = create_wall_timer(10s, [this]() { log_stats(); });
    }

    ~HI12Node() override {
        if (driver_) driver_->close();
    }

private:
    void on_imu_data(const hi12_imu::protocol::ImuData& data) {
        // Rate limiting
        if (publish_hz_ > 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_publish_time_).count();
            if (elapsed < (1.0 / publish_hz_)) return;
            last_publish_time_ = now;
        }

        auto stamp = get_clock()->now();

        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        // Orientation — quaternion from sensor (already normalized)
        msg.orientation.w = data.quat_w;
        msg.orientation.x = data.quat_x;
        msg.orientation.y = data.quat_y;
        msg.orientation.z = data.quat_z;

        // Angular velocity — already in rad/s from parse_payload()
        msg.angular_velocity.x = data.gyro_x;
        msg.angular_velocity.y = data.gyro_y;
        msg.angular_velocity.z = data.gyro_z;

        // Linear acceleration — already in m/s² from parse_payload()
        msg.linear_acceleration.x = data.accel_x;
        msg.linear_acceleration.y = data.accel_y;
        msg.linear_acceleration.z = data.accel_z;

        // Covariance
        for (int i = 0; i < 9; i++) {
            msg.orientation_covariance[i] = 0.0;
            msg.angular_velocity_covariance[i] = 0.0;
            msg.linear_acceleration_covariance[i] = 0.0;
        }
        msg.orientation_covariance[0] = -1.0;  // unknown

        double gc = gyro_covariance_;
        msg.angular_velocity_covariance[0] = gc;
        msg.angular_velocity_covariance[4] = gc;
        msg.angular_velocity_covariance[8] = gc;

        double ac = accel_covariance_;
        msg.linear_acceleration_covariance[0] = ac;
        msg.linear_acceleration_covariance[4] = ac;
        msg.linear_acceleration_covariance[8] = ac;

        imu_pub_->publish(msg);
    }

    void log_stats() {
        auto s = driver_->stats();
        RCLCPP_INFO(get_logger(),
            "HI12 stats: %.1f Hz, %lu valid, %lu CRC err, %lu parse err, %lu serial err, "
            "%.1f MB read, uptime %.0f s",
            s.effective_hz,
            (unsigned long)s.valid_frames, (unsigned long)s.crc_errors,
            (unsigned long)s.parse_errors, (unsigned long)s.serial_errors,
            s.bytes_read / 1e6, s.uptime_sec);
    }

    std::unique_ptr<hi12_imu::HI12Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::string device_;
    int         baudrate_;
    std::string frame_id_;
    int         publish_hz_;
    double      gyro_covariance_;
    double      accel_covariance_;

    std::chrono::steady_clock::time_point last_publish_time_{};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<HI12Node>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("hi12_imu"), "Fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
