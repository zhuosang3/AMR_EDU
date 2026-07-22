/**
 * @file lights_client.hpp
 * @brief EXP2 灯光客户端 — 头文件即可用，无需链接额外库 (v2.0)
 *
 * 用法 (C++ ROS2 节点中):
 *   #include "lights_client.hpp"
 *
 *   // 在节点构造函数中初始化
 *   LightsClient lights(node);
 *
 *   // 语义化 API
 *   lights.on(1, "r");          // 左前红
 *   lights.on("lf", "green");   // 左前绿
 *   lights.on("右后", "黄");     // 右后黄
 *   lights.off("rf", "red");
 *   lights.flash("lb", "y", 500);
 *   lights.all_on();
 *   lights.all_off();
 *
 * === v2.0 注意事项 ===
 * 底层已实现方位互斥: 每个方位同时只能点亮一种颜色。
 * 调用 light_on() 时会自动关断同方位其他颜色。
 *
 * 原理: 封装对 /lights/command 话题的发布，无需链接 exp2_lights 库。
 * 必须在 lights_bridge 节点运行的情况下使用。
 *
 * 线程安全: 所有 public 方法可跨线程调用。
 */

#ifndef EXP2_LIGHTS_CLIENT_HPP_
#define EXP2_LIGHTS_CLIENT_HPP_

#include <map>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LightsClient {
public:
    // 位置常量
    static constexpr int LEFT_FRONT  = 1;
    static constexpr int RIGHT_FRONT = 2;
    static constexpr int LEFT_BACK   = 3;
    static constexpr int RIGHT_BACK  = 4;

    // 颜色常量
    static constexpr const char *RED    = "r";
    static constexpr const char *GREEN  = "g";
    static constexpr const char *YELLOW = "y";

    /**
     * @param node  你的 ROS2 节点引用（用于创建 publisher）
     * @param topic 灯光命令话题名，默认 /lights/command
     */
    explicit LightsClient(rclcpp::Node &node,
                          const std::string &topic = "/lights/command")
        : logger_(node.get_logger())
    {
        pub_ = node.create_publisher<std_msgs::msg::String>(topic, 10);
    }

    // ── 语义 API ──────────────────────────────────────

    /// 点亮灯光 (自动关断同方位其他颜色)
    /// @param pos  位置: 1/2/3/4, "lf"/"rf"/"lb"/"rb", "左前"/"右前"/"左后"/"右后"
    /// @param color 颜色: "r"/"g"/"y", "red"/"green"/"yellow", "红"/"绿"/"黄"
    void on(int pos, const std::string &color) {
        publish("on " + pos_str(pos) + " " + color_str(color));
    }
    void on(const std::string &pos, const std::string &color) {
        publish("on " + pos_str(pos) + " " + color_str(color));
    }

    /// 关闭灯光
    void off(int pos, const std::string &color) {
        publish("off " + pos_str(pos) + " " + color_str(color));
    }
    void off(const std::string &pos, const std::string &color) {
        publish("off " + pos_str(pos) + " " + color_str(color));
    }

    /// 单灯闪烁
    /// @param ms 闪烁持续毫秒（阻塞）
    void flash(int pos, const std::string &color, int ms) {
        publish("flash " + pos_str(pos) + " " + color_str(color) + " " + std::to_string(ms));
    }
    void flash(const std::string &pos, const std::string &color, int ms) {
        publish("flash " + pos_str(pos) + " " + color_str(color) + " " + std::to_string(ms));
    }

    /// 全开 (每个方位亮红色) / 全关
    void all_on()  { publish("all_on"); }
    void all_off() { publish("all_off"); }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Logger logger_;
    std::mutex mtx_;

    void publish(const std::string &cmd) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = cmd;
        pub_->publish(std::move(msg));
    }

    // ── 参数展开 ──────────────────────────────────────

    static std::string pos_str(int pos) {
        switch (pos) {
            case 1: return "1";
            case 2: return "2";
            case 3: return "3";
            case 4: return "4";
            default: return std::to_string(pos);
        }
    }

    static std::string pos_str(const std::string &s) {
        static const std::map<std::string, std::string> map = {
            {"lf","1"}, {"左前","1"}, {"left_front","1"},
            {"rf","2"}, {"右前","2"}, {"right_front","2"},
            {"lb","3"}, {"左后","3"}, {"left_back","3"},
            {"rb","4"}, {"右后","4"}, {"right_back","4"},
        };
        auto it = map.find(s);
        return (it != map.end()) ? it->second : s;
    }

    static std::string color_str(const std::string &s) {
        static const std::map<std::string, std::string> map = {
            {"r","r"}, {"red","r"}, {"红","r"}, {"红色","r"},
            {"g","g"}, {"green","g"}, {"绿","g"}, {"绿色","g"},
            {"y","y"}, {"yellow","y"}, {"黄","y"}, {"黄色","y"},
        };
        auto it = map.find(s);
        return (it != map.end()) ? it->second : s;
    }
};

#endif  // EXP2_LIGHTS_CLIENT_HPP_
