# EXP2 小车灯光系统 — 使用说明书

> 版本: 2.0 | 更新: 2026-06-16 | 适用于 Orange Pi 5 Plus / ROS2 Jazzy

---

## 一、系统原理

```
┌─────────────────────────────────────────────────────────┐
│  用户                                                      │
│    │  $ light on lf r                                     │
│    ▼                                                      │
│  /usr/local/bin/light          ← 快捷脚本（自动 source）     │
│    │  ros2 topic pub /lights/command std_msgs/String       │
│    ▼                                                      │
│  ROS2 网络  ← /lights/command 话题                         │
│    │                                                      │
│    ▼                                                      │
│  lights_bridge (exp2_lights_bridge)  ← C++ ROS2 桥接节点    │
│    │  解析命令, 方位互斥, 调用 C 库                           │
│    ▼                                                      │
│  libexp2_lights.a (exp2_lights)  ← 纯 C 库                 │
│    │  libgpiod → /dev/gpiochip0~4                          │
│    ▼                                                      │
│  Orange Pi 5 Plus GPIO 引脚  →  12 个 LED 灯               │
└─────────────────────────────────────────────────────────┘
```

**分层设计**：

| 层 | 包名 | 语言 | 职责 |
|----|------|------|------|
| 快捷脚本 | `/usr/local/bin/light` | zsh | 封装 ros2 topic pub，用户直接敲简短命令 |
| Bridge 节点 | `exp2_lights_bridge` | C++ (ROS2) | 订阅 `/lights/command`，解析命令，方位互斥 |
| 硬件驱动库 | `exp2_lights` | C (libgpiod) | 直接操作 GPIO 寄存器，点亮/熄灭 LED |

**互斥机制**：每个方位（左前/右前/左后/右后）同时只能点亮一种颜色。调用 `light_on()` 时会自动关断同方位的其他颜色，保证硬件级互斥。不同方位之间互相独立，可同时各亮一种颜色（最多 4 灯）。

---

## 二、硬件布局与 GPIO 对照

小车共 4 个位置 × 3 种颜色 = **12 个 LED**：

```
                    ↑ 前进方向 ↑
         ┌─────────────────────────────┐
         │   左前 ①          右前 ②    │
         │ 红17  绿27  黄22   红SDA 绿SCL 黄MISO │
         │                             │
         │   左后 ③          右后 ④    │
         │ 红6   绿13  黄26   红18  绿12  黄21  │
         └─────────────────────────────┘
```

| 位置 | 缩写 | 中文 | 红 | 绿 | 黄 |
|------|------|------|----|----|-----|
| 左前 ① | `lf` / `1` | 左前 | **17** | **27** | **22** |
| 右前 ② | `rf` / `2` | 右前 | **SDA** | **SCL** | **MISO** |
| 左后 ③ | `lb` / `3` | 左后 | **6** | **13** | **26** |
| 右后 ④ | `rb` / `4` | 右后 | **18** | **12** | **21** |

> 注：右前 3 个灯使用排针功能名 (SDA/SCL/MISO)，其余 9 个使用 GPIO 编号。

---

## 三、快速上手

### 3.1 最简单的用法

SSH 连接到小车后直接敲：

```bash
# 开灯  <位置> <颜色>
light on 1 r          # 左前红亮
light on lf y         # 左前黄亮（红自动灭）
light on 右前 绿       # 右前绿亮

# 关灯
light off 2 r         # 右前红灭
light off rb g        # 右后绿灭

# 闪烁  <位置> <颜色> <毫秒>
light flash 3 y 500   # 左后黄闪 500 毫秒
light flash lf r 1000 # 左前红闪 1 秒

# 全局
light all_on          # 全开（每个方位亮红色，共4灯）
light all_off         # 全关
light status          # 查看状态
```

### 3.2 位置写法（任选，效果相同）

| 风格 | 左前 | 右前 | 左后 | 右后 |
|------|------|------|------|------|
| 数字 | `1` | `2` | `3` | `4` |
| 英文缩写 | `lf` | `rf` | `lb` | `rb` |
| 中文 | `左前` | `右前` | `左后` | `右后` |

缩写规则：**L**eft / **R**ight + **F**ront / **B**ack

### 3.3 颜色写法（任选，效果相同）

| 风格 | 红 | 绿 | 黄 |
|------|----|----|-----|
| 字母 | `r` | `g` | `y` |
| 英文 | `red` | `green` | `yellow` |
| 中文 | `红` | `绿` | `黄` |

### 3.4 混搭示例

```bash
light on 1 red        # 数字 + 英文
light on lf 红         # 缩写 + 中文
light off 右后 green   # 中文 + 英文
light flash 左前 黄 300
```

---

## 四、完整命令参考

### 4.1 新格式（推荐）

| 命令 | 参数 | 示例 |
|------|------|------|
| `light on <位置> <颜色>` | 位置 + 颜色 | `light on 1 r` |
| `light off <位置> <颜色>` | 位置 + 颜色 | `light off rf g` |
| `light flash <位置> <颜色> <ms>` | 位置 + 颜色 + 毫秒 | `light flash lb y 500` |
| `light all_on` | 无 | 全开（每个方位亮红色，共4灯） |
| `light all_off` | 无 | 全关 |
| `light status` | 无 | 查看当前亮灯状态 |
| `light` | 无 | 显示帮助 |

### 4.2 旧格式（兼容，不推荐）

```bash
light on 17           # 直接用 GPIO 编号
light off SDA         # 直接用排针功能名
light flash 22 300    # 旧格式闪烁
```

---

## 五、快捷脚本

**位置**：`/usr/local/bin/light`

**内容**：

```zsh
#!/bin/zsh
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh
if [ $# -eq 0 ]; then
    echo "用法: light <命令>"
    echo "  light on/off <位置> <颜色>    开关灯"
    echo "  light flash <位置> <颜色> <ms> 闪烁"
    echo "  light all_on / all_off / status"
    echo "  位置: 1/lf/左前  2/rf/右前  3/lb/左后  4/rb/右后"
    echo "  颜色: r/红  g/绿  y/黄"
    exit 0
fi
ros2 topic pub /lights/command std_msgs/msg/String "data: \"$*\"" -1
```

该脚本由 root 安装在 `/usr/local/bin`，所有用户可直接使用，内部已自动完成 ROS2 环境初始化。

如果将来需要修改，编辑后执行：
```bash
sudo vi /usr/local/bin/light    # 编辑
sudo chmod +x /usr/local/bin/light  # 确保可执行
```

---

## 六、PM2 进程管理

`lights_bridge` 节点通过 PM2 管理，开机自动启动。

### 6.1 常用 PM2 命令

```bash
pm2 list                          # 查看所有进程状态
pm2 logs lights_bridge            # 查看实时日志
pm2 logs lights_bridge --lines 20 # 最近 20 行
pm2 restart lights_bridge         # 重启节点
pm2 stop lights_bridge            # 停止节点
pm2 start lights_bridge           # 启动节点
```

### 6.2 启动脚本

**位置**：`/home/expressone/start_lights.sh`

```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/expressone/ros2_ws/install/setup.bash 2>/dev/null
exec /home/expressone/ros2_ws/install/exp2_lights_bridge/lib/exp2_lights_bridge/lights_bridge
```

GPIO 设备已通过 `chmod 666` 放开权限，无需 sudo。

### 6.3 源码位置

```
~/ros2_ws/src/
├── exp2_lights/              # 纯 C GPIO 库
│   ├── lights.h              # 头文件（含引脚定义）
│   ├── lights.c              # 实现（libgpiod 操作）
│   └── CMakeLists.txt
│
└── exp2_lights_bridge/       # ROS2 桥接节点
    ├── src/
    │   └── lights_bridge.cpp # 命令解析 + 方位互斥
    ├── CMakeLists.txt
    └── package.xml
```

---

## 七、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| `light` 命令一直 `Waiting for matching subscription` | `lights_bridge` 没在运行 | `pm2 restart lights_bridge` |
| `light` 输出 `command not found` | 脚本未安装 | `sudo cp /tmp/light.sh /usr/local/bin/light` |
| 某几个灯不亮 | GPIO 线松动或代码映射错误 | 检查接线，对照第二章表格 |
| 全不亮 | GPIO 权限丢失 | `sudo chmod 666 /dev/gpiochip*` |
| 同方位想换颜色但亮不了 | 方位互斥：需先关再开，或直接开新颜色（自动覆盖） | 直接 `light on <位置> <新颜色>` 即可，旧颜色自动灭 |
| 重启后灯不亮 | PM2 未保存配置 | `pm2 save` |

### 手动启动（脱离 PM2 调试时用）

```bash
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh
/home/expressone/ros2_ws/install/exp2_lights_bridge/lib/exp2_lights_bridge/lights_bridge
```

如果 GPIO 权限丢失：
```bash
sudo chmod 666 /dev/gpiochip0 /dev/gpiochip1 /dev/gpiochip2 /dev/gpiochip3 /dev/gpiochip4
```

---

## 九、开发者 API

如果要在你自己的 ROS2 代码中控制灯光，不需要操作 GPIO，直接调下面的客户端库即可。它们内部往 `/lights/command` 话题发消息，由 `lights_bridge` 统一执行。

### 9.1 方式一：C++ 头文件（推荐）

**文件位置**：`~/ros2_ws/src/exp2_lights_bridge/include/lights_client.hpp`

**用法** — 单头文件，无需链接额外库：

```cpp
#include "lights_client.hpp"

// 在你的 ROS2 节点中
LightsClient lights(*this);  // this = rclcpp::Node&

// 语义化调用
lights.on(1, "r");              // 左前红
lights.on("lf", "green");       // 左前绿
lights.on("右后", "黄");         // 右后黄
lights.off("rf", "red");        // 右前红灭
lights.flash("lb", "y", 500);   // 左后黄闪 500ms
lights.all_on();                // 全开（每个方位亮红色）
lights.all_off();               // 全关

// 也可用常量
lights.on(LightsClient::LEFT_FRONT, LightsClient::RED);
lights.on(LightsClient::RIGHT_BACK, LightsClient::YELLOW);
```

### 9.2 方式二：Python 模块

**文件位置**：`~/ros2_ws/src/exp2_lights_bridge/lights_client.py`

**用法**：

```python
from lights_client import LightsClient

# 在你的 ROS2 节点中
lights = LightsClient(self)  # self = rclpy.node.Node

lights.on(1, "r")              # 左前红
lights.on("lf", "green")       # 左前绿
lights.on("右后", "黄")         # 右后黄
lights.off("rf", "red")        # 右前红灭
lights.flash("lb", "y", 500)   # 左后黄闪
lights.all_on()                # 全开（每个方位亮红色）
lights.all_off()               # 全关
```

### 9.3 方式三：直接操作 GPIO（高级）

如果需要绕过 ROS2 直接控制 GPIO（如非 ROS 程序），可以链接 C 静态库：

**头文件**：`~/ros2_ws/install/exp2_lights/include/exp2_lights/lights.h`
**库文件**：`~/ros2_ws/install/exp2_lights/lib/libexp2_lights.a`

```c
#include "lights.h"

light_init();
light_on(17);           // 左前红亮 (GPIO 17)
light_on_str("SDA");    // 右前红亮 (排针名)
light_off_int(17);      // 左前红灭
light_cleanup();        // 释放资源
```

编译链接：
```bash
gcc your_program.c -I~/ros2_ws/install/exp2_lights/include \
    ~/ros2_ws/install/exp2_lights/lib/libexp2_lights.a -lgpiod
```

> ⚠️ 注意：直接操作 GPIO 会和 `lights_bridge` 冲突，两者不能同时使用。

### 9.4 方式四：通过话题（任意语言）

只要能发 ROS2 消息的语言都能控制：

```
话题: /lights/command
类型: std_msgs/msg/String
格式: "<命令> <参数>"
```

伪代码示例：
```
publish("/lights/command", "on 1 r")       // 左前红
publish("/lights/command", "off rf g")     // 右前绿灭
publish("/lights/command", "flash 3 y 500") // 左后黄闪
publish("/lights/command", "all_off")      // 全关
```

### 9.5 方式对比

| 方式 | 适用场景 | 依赖 | 优点 |
|------|---------|------|------|
| C++ 头文件 | ROS2 C++ 节点 | rclcpp, std_msgs | 类型安全，IDE 补全 |
| Python 模块 | ROS2 Python 节点 | rclpy, std_msgs | 最简，一行一个灯 |
| 直连 GPIO | 非 ROS 程序 | libgpiod | 零延迟，无 ROS 依赖 |
| 话题消息 | 任意语言 | ROS2 客户端库 | 语言无关 |

---

## 十、重新编译

修改源码后：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
colcon build --packages-select exp2_lights exp2_lights_bridge
pm2 restart lights_bridge
```

---

*文档更新日期：2026-06-16 | 基于 Orange Pi 5 Plus / Armbian / ROS2 Jazzy 实际部署*
