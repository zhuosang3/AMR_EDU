/**
 * @file lights_bridge.cpp
 * @brief ROS2 bridge: wraps exp2_lights C library (v2.0 带方位互斥)
 *
 * 订阅 /lights/command (std_msgs/String), 解析命令控制 GPIO 灯光.
 *
 * === v2.0 变更 ===
 * - 移除 MAX_LIGHTS_ON=4 硬限制
 * - 改为方位互斥: 同方位同时只能点亮一种颜色
 * - 底层 lights.c 已实现硬件级互斥 (light_on 自动关断同方位其他颜色)
 * - g_lit 状态追踪同步更新
 *
 * === 新语义命令 (推荐) ===
 *   on    <位置> <颜色>         e.g. on 1 r, on 左前 红, on lf red
 *   off   <位置> <颜色>         e.g. off 2 g, off 右后 黄
 *   flash <位置> <颜色> <毫秒>  e.g. flash 3 y 500
 *   all_on / all_off / status
 *
 * === 兼容旧命令 ===
 *   on <gpio> / off <gpio> / flash <gpio> <ms>
 */

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

extern "C" {
#include "lights.h"
}

// ============================================================
// 通用工具
// ============================================================
static bool is_number(const std::string &s) {
    if (s.empty()) return false;
    for (char c : s) if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    return true;
}

static std::vector<std::string> split(const std::string &s) {
    std::vector<std::string> tokens;
    std::string token;
    for (char c : s) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (!token.empty()) { tokens.push_back(token); token.clear(); }
        } else {
            token += c;
        }
    }
    if (!token.empty()) tokens.push_back(token);
    return tokens;
}

// ============================================================
// GPIO 引脚定义 (与 lights.c 保持一致)
// ============================================================

// Int 类型 GPIO
static const int kIntPositions[] = {17, 27, 22, 6, 13, 26, 18, 12, 21};
// String 类型 GPIO
static const char * kStrPositions[] = {"SDA", "SCL", "MISO"};

// ── 语义化引脚映射 ──
struct LightId {
    std::string key;   // GPIO 标识 "17", "SDA", ...
    bool        is_int;
};

static std::map<std::string, LightId> g_semantic;  // "1r" → LightId

// ── 方位 sibling 映射: GPIO key → 同方位其他 GPIO key 列表 ──
// 用于状态追踪: 当点亮一个灯时，自动从 g_lit 移除 siblings
static std::map<std::string, std::vector<std::string>> g_siblings;

static void init_semantic_map() {
    // 左前 (位置1)
    g_semantic["1r"] = {"17", true};
    g_semantic["1g"] = {"27", true};
    g_semantic["1y"] = {"22", true};

    // 右前 (位置2)
    g_semantic["2r"] = {"SDA", false};
    g_semantic["2g"] = {"SCL", false};
    g_semantic["2y"] = {"MISO", false};

    // 左后 (位置3)
    g_semantic["3r"] = {"6", true};
    g_semantic["3g"] = {"13", true};
    g_semantic["3y"] = {"26", true};

    // 右后 (位置4)
    g_semantic["4r"] = {"18", true};
    g_semantic["4g"] = {"12", true};
    g_semantic["4y"] = {"21", true};

    // ── 构建 sibling 映射 ──
    auto add_group = [](const std::string &a,
                        const std::string &b,
                        const std::string &c) {
        g_siblings[a] = {b, c};
        g_siblings[b] = {a, c};
        g_siblings[c] = {a, b};
    };

    // 左前 siblings (GPIO key: "17", "27", "22")
    add_group("17", "27", "22");
    // 右前 siblings (GPIO key: "SDA", "SCL", "MISO")
    add_group("SDA", "SCL", "MISO");
    // 左后 siblings (GPIO key: "6", "13", "26")
    add_group("6", "13", "26");
    // 右后 siblings (GPIO key: "18", "12", "21")
    add_group("18", "12", "21");
}

// 位置名解析 → 1/2/3/4 (0 = 无法识别)
static int parse_position(const std::string &s) {
    if (s.empty()) return 0;
    if (s == "1") return 1;
    if (s == "2") return 2;
    if (s == "3") return 3;
    if (s == "4") return 4;
    if (s == "lf" || s == "左前") return 1;
    if (s == "rf" || s == "右前") return 2;
    if (s == "lb" || s == "左后") return 3;
    if (s == "rb" || s == "右后") return 4;
    return 0;
}

// 颜色名解析 → 'r'/'g'/'y' (0 = 无法识别)
static char parse_color(const std::string &s) {
    if (s.empty()) return 0;
    if (s == "r" || s == "red"   || s == "红" || s == "红色") return 'r';
    if (s == "g" || s == "green" || s == "绿" || s == "绿色") return 'g';
    if (s == "y" || s == "yellow"|| s == "黄" || s == "黄色") return 'y';
    return 0;
}

// 生成语义 key: "1r", "3g", ...
static std::string semantic_key(int pos, char color) {
    std::string k;
    k += static_cast<char>('0' + pos);
    k += color;
    return k;
}

// ── 操控函数 ──
static void do_on(const LightId &id) {
    if (id.is_int) light_on_int(std::stoi(id.key));
    else           light_on_str(id.key.c_str());
}
static void do_off(const LightId &id) {
    if (id.is_int) light_off_int(std::stoi(id.key));
    else           light_off_str(id.key.c_str());
}

// ============================================================
// 状态追踪
// ============================================================
static std::set<std::string> g_lit;  // key = GPIO 标识 "17", "SDA", ...

// 移除同方位 sibling 的状态追踪条目 (底层 lights.c 已做硬件关断)
static void state_mutex_off_siblings(const std::string &gpio_key) {
    auto it = g_siblings.find(gpio_key);
    if (it == g_siblings.end()) return;

    for (const auto &sib : it->second) {
        if (g_lit.count(sib)) {
            fprintf(stderr, "[lights_bridge] 互斥: 关断同方位 %s\n", sib.c_str());
            g_lit.erase(sib);
        }
    }
}

// ============================================================
// 命令执行
// ============================================================
static void execute_command(const std::string &cmd_line, rclcpp::Logger logger) {
    fprintf(stderr, "[lights_bridge] 收到命令: \"%s\"\n", cmd_line.c_str());
    fflush(stderr);

    auto tokens = split(cmd_line);
    if (tokens.empty()) return;

    std::string cmd = tokens[0];

    // ============================================================
    // 新语义命令: on <pos> <color>
    // ============================================================
    if ((cmd == "on" || cmd == "off") && tokens.size() >= 3) {
        int  pos   = parse_position(tokens[1]);
        char color = parse_color(tokens[2]);

        if (pos && color) {
            std::string sk = semantic_key(pos, color);
            auto it = g_semantic.find(sk);
            if (it == g_semantic.end()) { fflush(stderr); return; }

            const LightId &id = it->second;
            std::string key = id.key;

            if (cmd == "on") {
                if (g_lit.count(key)) {
                    fprintf(stderr, "[lights_bridge] %s 已亮, 忽略\n", key.c_str());
                    fflush(stderr); return;
                }
                // 移除同方位 sibling 的状态追踪
                state_mutex_off_siblings(key);
                fprintf(stderr, "[lights_bridge] light_on(\"%s\")\n", key.c_str());
                do_on(id);
                g_lit.insert(key);
            } else {  // off
                fprintf(stderr, "[lights_bridge] light_off(\"%s\")\n", key.c_str());
                do_off(id);
                g_lit.erase(key);
            }
            fflush(stderr);
            return;
        }
        // 解析失败 → 走旧格式
    }

    // ============================================================
    // 新语义命令: flash <pos> <color> <ms>
    // ============================================================
    if (cmd == "flash" && tokens.size() >= 4) {
        int  pos   = parse_position(tokens[1]);
        char color = parse_color(tokens[2]);

        if (pos && color) {
            std::string sk = semantic_key(pos, color);
            auto it = g_semantic.find(sk);
            if (it == g_semantic.end()) { fflush(stderr); return; }

            const LightId &id = it->second;
            std::string key = id.key;
            int ms = std::stoi(tokens[3]);

            fprintf(stderr, "[lights_bridge] flash %s (%dms)\n", key.c_str(), ms);
            bool was_lit = g_lit.count(key);

            // 移除同方位 sibling 的状态追踪
            state_mutex_off_siblings(key);
            do_on(id);
            if (!was_lit) g_lit.insert(key);
            rclcpp::sleep_for(std::chrono::milliseconds(ms));
            do_off(id);
            if (!was_lit) g_lit.erase(key);
            fflush(stderr);
            return;
        }
        // 解析失败 → 走旧格式
    }

    // ============================================================
    // 旧格式兼容: on <gpio>
    // ============================================================
    if (cmd == "on" && tokens.size() >= 2) {
        const std::string &gpio = tokens[1];
        std::string key = gpio;

        if (g_lit.count(key)) {
            fprintf(stderr, "[lights_bridge] %s 已亮, 忽略\n", key.c_str());
            fflush(stderr); return;
        }
        // 移除同方位 sibling 的状态追踪
        state_mutex_off_siblings(key);
        if (is_number(gpio)) {
            light_on_int(std::stoi(gpio));
        } else {
            light_on_str(gpio.c_str());
        }
        g_lit.insert(key);
        fflush(stderr);
        return;
    }

    // ============================================================
    // 旧格式兼容: off <gpio>
    // ============================================================
    if (cmd == "off" && tokens.size() >= 2) {
        const std::string &gpio = tokens[1];
        if (is_number(gpio)) light_off_int(std::stoi(gpio));
        else light_off_str(gpio.c_str());
        g_lit.erase(gpio);
        fflush(stderr);
        return;
    }

    // ============================================================
    // 旧格式兼容: flash <gpio> <ms>
    // ============================================================
    if (cmd == "flash" && tokens.size() >= 3) {
        const std::string &gpio = tokens[1];
        int ms = std::stoi(tokens[2]);
        std::string key = gpio;

        bool was_lit = g_lit.count(key);
        // 移除同方位 sibling 的状态追踪
        state_mutex_off_siblings(key);
        if (is_number(gpio)) light_on_int(std::stoi(gpio));
        else light_on_str(gpio.c_str());
        if (!was_lit) g_lit.insert(key);
        rclcpp::sleep_for(std::chrono::milliseconds(ms));
        if (is_number(gpio)) light_off_int(std::stoi(gpio));
        else light_off_str(gpio.c_str());
        if (!was_lit) g_lit.erase(key);
        fflush(stderr);
        return;
    }

    // ============================================================
    // all_on: 每个方位点亮一种颜色 (默认全部亮红色)
    // ============================================================
    if (cmd == "all_on") {
        fprintf(stderr, "[lights_bridge] 全开: 4 方位各亮红色\n");

        // 先清状态，然后每个方位亮红色
        g_lit.clear();

        // 左前红 17
        light_on_int(17);  g_lit.insert("17");
        // 右前红 SDA
        light_on_str("SDA"); g_lit.insert("SDA");
        // 左后红 6
        light_on_int(6);   g_lit.insert("6");
        // 右后红 18
        light_on_int(18);  g_lit.insert("18");

        fflush(stderr);
        return;
    }

    // ============================================================
    // all_off / status
    // ============================================================
    if (cmd == "all_off") {
        fprintf(stderr, "[lights_bridge] 全关 %d 个灯\n", (int)g_lit.size());
        for (int p : kIntPositions) light_off_int(p);
        for (const char *p : kStrPositions) light_off_str(p);
        g_lit.clear();
        fflush(stderr);
        return;
    }
    if (cmd == "status") {
        fprintf(stderr, "[lights_bridge] 状态: %d 个灯亮 (方位互斥模式)", (int)g_lit.size());
        if (!g_lit.empty()) {
            fprintf(stderr, " (");
            for (auto &k : g_lit) fprintf(stderr, "%s ", k.c_str());
            fprintf(stderr, ")");
        }
        fprintf(stderr, "\n");
        fflush(stderr);
        return;
    }

    // ============================================================
    // 未知命令
    // ============================================================
    fprintf(stderr, "[lights_bridge] 未知命令: \"%s\"\n", cmd_line.c_str());
    fprintf(stderr, "[lights_bridge] 新格式: on/off/flash <位置> <颜色> [ms]\n");
    fprintf(stderr, "[lights_bridge]   位置: 1/2/3/4, lf/rf/lb/rb, 左前/右前/左后/右后\n");
    fprintf(stderr, "[lights_bridge]   颜色: r/g/y, red/green/yellow, 红/绿/黄\n");
    fprintf(stderr, "[lights_bridge]   示例: on 1 r, off 右前 绿, flash lr y 500\n");
    fprintf(stderr, "[lights_bridge] 旧格式: on <gpio>, off <gpio>, all_off, all_on, status\n");
    fprintf(stderr, "[lights_bridge] 注意: 同方位只能亮一种颜色 (硬件互斥)\n");
    fflush(stderr);
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lights_bridge");
    auto logger = node->get_logger();

    init_semantic_map();

    RCLCPP_INFO(logger, "EXP2 灯光桥接 v2.0 启动 — 方位互斥模式");
    fprintf(stderr, "[lights_bridge] 节点启动，初始化 GPIO...\n");

    light_init();

    fprintf(stderr, "[lights_bridge] GPIO 初始化完成, 方位互斥模式\n");
    fprintf(stderr, "[lights_bridge] 规则: 每个方位同时只能点亮一种颜色\n");
    fprintf(stderr, "[lights_bridge] 新命令: on/off/flash <位置> <颜色> [ms]\n");
    fprintf(stderr, "[lights_bridge]   例: on 1 r  (左前红), off 右前 绿, flash lb y 500\n");
    fprintf(stderr, "[lights_bridge] 旧命令兼容: on 17, off SDA, all_off, all_on, status\n");

    auto sub = node->create_subscription<std_msgs::msg::String>(
        "/lights/command",
        10,
        [logger](std_msgs::msg::String::ConstSharedPtr msg) {
            execute_command(msg->data, logger);
        });

    rclcpp::spin(node);

    light_cleanup();
    rclcpp::shutdown();
    return 0;
}
