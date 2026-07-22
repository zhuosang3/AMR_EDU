/**
 * DYP-A21 Ultrasonic CAN Driver (C++ version)
 *
 * Protocol (datasheet section 3.9):
 *   CANID = 0x0520 + slave_address
 *   Read cmd:  [addr, 0x03, reg_H, reg_L, 0x00, 0x01]
 *   Response:  [addr, 0x83, 0x02, data_H, data_L]
 *   Value = data_H << 8 | data_L
 *
 * Register 0x0101:
 *   Real-time distance value in mm (per datasheet section 3.9.4).
 *   Value / 1000.0 = meters. Response time ~15-140ms.
 *
 * Special values: 0xFFFE = interference, 0xFFFD = no object detected
 */

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <limits>
#include <vector>
#include <string>
#include <optional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

// CAN Protocol Constants
static constexpr uint32_t kCanIdBase      = 0x0520;
static constexpr uint16_t kRegDistance    = 0x0101;  // real-time distance in mm (register 0x0101)
static constexpr uint8_t  kCmdRead        = 0x03;
static constexpr uint8_t  kRespRead       = 0x83;
static constexpr uint8_t  kRespLen        = 0x02;
static constexpr uint16_t kValInterference = 0xFFFE;
static constexpr uint16_t kValNoObject    = 0xFFFD;

struct SensorConfig {
    uint8_t  address;
    uint32_t can_id;
    std::string frame_id;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher;
    double field_of_view;
    double min_range;
    double max_range;
};

class UltrasonicNode : public rclcpp::Node {
public:
    UltrasonicNode()
        : Node("dyp_a21_ultrasonic")
        , can_socket_(-1)
    {
        declare_parameter("can_interface", "can0");
        declare_parameter("can_bitrate", 500000);
        declare_parameter("publish_rate", 2.0);
        declare_parameter("sensor_names", std::vector<std::string>{"front", "rear"});

        can_iface_  = get_parameter("can_interface").as_string();
        can_bitrate_ = get_parameter("can_bitrate").as_int();
        double rate = get_parameter("publish_rate").as_double();
        auto names  = get_parameter("sensor_names").as_string_array();

        for (const auto& name : names) {
            declare_parameter(name + ".address", 0x01);
            declare_parameter(name + ".frame_id", "ultrasonic_" + name);
            declare_parameter(name + ".topic", "~/range_" + name);
            declare_parameter(name + ".field_of_view", 0.7);
            declare_parameter(name + ".min_range", 0.03);
            declare_parameter(name + ".max_range", 5.0);
        }

        if (!setup_can(can_iface_, can_bitrate_)) {
            RCLCPP_ERROR(get_logger(), "Failed to open CAN bus %s", can_iface_.c_str());
            rclcpp::shutdown();
            return;
        }

        for (const auto& name : names) {
            SensorConfig s;
            s.address  = static_cast<uint8_t>(get_parameter(name + ".address").as_int());
            s.can_id   = kCanIdBase + s.address;
            s.frame_id = get_parameter(name + ".frame_id").as_string();
            s.field_of_view = get_parameter(name + ".field_of_view").as_double();
            s.min_range     = get_parameter(name + ".min_range").as_double();
            s.max_range     = get_parameter(name + ".max_range").as_double();

            auto topic = get_parameter(name + ".topic").as_string();
            s.publisher = create_publisher<sensor_msgs::msg::Range>(topic, 10);

            sensors_.push_back(std::move(s));
            RCLCPP_INFO(get_logger(), "  %s: 0x%02X -> CANID 0x%03X -> %s",
                        name.c_str(), s.address, s.can_id, topic.c_str());
        }

        // Install CAN receive filters
        struct can_filter filters[sensors_.size()];
        for (size_t i = 0; i < sensors_.size(); ++i) {
            filters[i].can_id   = sensors_[i].can_id;
            filters[i].can_mask = CAN_SFF_MASK;
        }
        setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
                   filters, sizeof(filters));

        int period_ms = static_cast<int>(1000.0 / std::max(rate, 0.1));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&UltrasonicNode::poll_callback, this));

        RCLCPP_INFO(get_logger(),
                    "CAN driver ready: %s @ %ld bps, %zu sensor(s) @ %.1f Hz "
                    "(real-time distance mm, reg=0x%04X)",
                    can_iface_.c_str(), can_bitrate_, sensors_.size(),
                    rate, kRegDistance);
    }

    ~UltrasonicNode() override {
        if (can_socket_ >= 0) close(can_socket_);
    }

private:
    bool setup_can(const std::string& iface, long bitrate) {
        // Bring interface up if down
        std::string cmd = "ip link show " + iface
                        + " 2>/dev/null | grep -q 'state DOWN'";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            cmd = "sudo ip link set " + iface
                + " type can bitrate " + std::to_string(bitrate)
                + " 2>/dev/null";
            system(cmd.c_str());
            cmd = "sudo ip link set " + iface + " up 2>/dev/null";
            system(cmd.c_str());
            RCLCPP_INFO(get_logger(), "Brought %s up @ %ld bps", iface.c_str(), bitrate);
        }

        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(get_logger(), "socket(PF_CAN): %s", strerror(errno));
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(get_logger(), "ioctl(SIOCGIFINDEX) %s: %s",
                         iface.c_str(), strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr),
                 sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "bind() %s: %s", iface.c_str(), strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        RCLCPP_INFO(get_logger(), "CAN bus %s opened @ %ld bps", iface.c_str(), bitrate);
        return true;
    }

    void send_query(const SensorConfig& sensor) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id  = sensor.can_id;
        frame.can_dlc = 6;
        frame.data[0] = sensor.address;
        frame.data[1] = kCmdRead;
        frame.data[2] = (kRegDistance >> 8) & 0xFF;
        frame.data[3] = kRegDistance & 0xFF;
        frame.data[4] = 0x00;
        frame.data[5] = 0x01;

        if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "CAN write error: %s", strerror(errno));
        }
    }

    std::optional<uint16_t> parse_response(const struct can_frame& frame,
                                           uint8_t expected_addr) {
        if (frame.can_dlc < 5)            return std::nullopt;
        if (frame.data[0] != expected_addr) return std::nullopt;
        if (frame.data[1] != kRespRead)    return std::nullopt;
        if (frame.data[2] != kRespLen)     return std::nullopt;

        uint16_t raw = (static_cast<uint16_t>(frame.data[3]) << 8)
                     |  static_cast<uint16_t>(frame.data[4]);

        // 0xFFFE = interference, 0xFFFD = no object (per datasheet)
        if (raw == kValInterference || raw == kValNoObject) {
            return std::nullopt;
        }
        return raw;
    }

void poll_callback() {
        auto stamp = now();

        for (auto& sensor : sensors_) {
            // ---- First read: trigger measurement ----
            std::optional<uint16_t> raw1 = do_read(sensor, 100ms);
            if (!raw1) continue;

            // ---- Small delay to let sensor update register ----
            std::this_thread::sleep_for(50ms);

            // ---- Second read: get updated value ----
            std::optional<uint16_t> raw2 = do_read(sensor, 100ms);

            uint16_t best_raw;
            if (!raw2) {
                best_raw = *raw1;
            } else if (*raw1 == *raw2) {
                best_raw = *raw1;  // consistent, use as-is
            } else {
                // Inconsistent: pick the one with non-zero low byte
                bool lo1 = (*raw1 & 0xFF) != 0;
                bool lo2 = (*raw2 & 0xFF) != 0;
                if (lo1 && !lo2) {
                    best_raw = *raw1;
                } else if (lo2 && !lo1) {
                    best_raw = *raw2;
                } else {
                    best_raw = *raw2;  // both or neither have low byte, use newer
                }
            }

            // Log raw value for diagnostics (throttled to ~every 5s per sensor)
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "%s raw=0x%04X (%u)",
                sensor.frame_id.c_str(), best_raw, best_raw);

            auto msg = sensor_msgs::msg::Range();
            msg.header.stamp         = stamp;
            msg.header.frame_id      = sensor.frame_id;
            msg.radiation_type       = sensor_msgs::msg::Range::ULTRASOUND;
            msg.field_of_view        = sensor.field_of_view;
            msg.min_range            = sensor.min_range;
            msg.max_range            = sensor.max_range;

            if (best_raw == 0xFFFD) {
                // 0xFFFD = no object detected (per datasheet)
                msg.range = std::numeric_limits<double>::infinity();
            } else if (best_raw == 0xFFFE) {
                // 0xFFFE = interference (per datasheet)
                msg.range = std::numeric_limits<double>::infinity();
            } else {
                // register 0x0101: real-time distance in mm
                double dist_m = static_cast<double>(best_raw) / 1000.0;
                if (dist_m > sensor.max_range) continue;
                if (dist_m < sensor.min_range) continue;
                msg.range = dist_m;
            }

            sensor.publisher->publish(msg);
        }
    }

    std::optional<uint16_t> do_read(const SensorConfig& sensor,
                                    std::chrono::milliseconds timeout) {
        send_query(sensor);

        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 15000;

            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(can_socket_, &readfds);

            int ret = select(can_socket_ + 1, &readfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (ret == 0) continue;

            struct can_frame frame;
            std::memset(&frame, 0, sizeof(frame));
            ssize_t n = read(can_socket_, &frame, sizeof(frame));
            if (n < 0) continue;

            if (frame.can_id != sensor.can_id) continue;

            auto raw = parse_response(frame, sensor.address);
            if (raw) return raw;
        }
        return std::nullopt;
    }

    int can_socket_;
    std::string can_iface_;
    long can_bitrate_;
    std::vector<SensorConfig> sensors_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UltrasonicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
