# EXP2 小车 IMU 驱动 — 使用说明书

> HI12M0-MI1-000 惯性测量单元 | Orange Pi 5 Plus | ROS2 Jazzy
> 部署位置：`expressone@192.168.8.30:~/ros2_ws/src/imu_4/`

---

## 一、系统原理

### 1.1 IMU 是什么

IMU（Inertial Measurement Unit，惯性测量单元）实时测量小车的运动和姿态：

```
HI12 传感器                       Orange Pi 5 Plus                  ROS2 网络
   │                                    │                              │
   │  UART /dev/hi12_imu                    │                              │
   │  82 字节/帧, 100 Hz                 │                              │
   │ ──────────────────────────────────→ │                              │
   │                                    │  ① 串口读原始字节              │
   │                                    │  ② 找 5A A5 帧头               │
   │                                    │  ③ CRC16 校验                 │
   │                                    │  ④ 解析 76 字节数据体          │
   │                                    │  ⑤ 单位转换 (G→m/s², °/s→rad/s)│
   │                                    │  ⑥ 发布 sensor_msgs/Imu ────→ │
   │                                    │  话题: /imu (100 Hz)      │
```

### 1.2 传感器输出

| 数据 | 原始单位 | ROS 单位 | 用途 |
|------|---------|---------|------|
| 加速度 (X/Y/Z) | G | m/s² | 检测运动、碰撞 |
| 陀螺仪 (X/Y/Z) | °/s | rad/s | 检测旋转、转弯 |
| 四元数 (W/X/Y/Z) | — | — | 姿态表示（无死锁） |
| 欧拉角 (Pitch/Roll/Yaw) | 度 | 度 | 直观姿态 |

### 1.3 分层架构

```
┌─────────────────────────────────────┐
│  ROS2 Node (hi12_node.cpp)          │  ← 入口：创建节点、发布者、定时器
├─────────────────────────────────────┤
│  Driver (hi12_driver.cpp)           │  ← 核心：串口配置、读线程管理
├─────────────────────────────────────┤
│  Protocol (protocol.cpp)            │  ← 解析：帧同步、CRC、字节→浮点数
├─────────────────────────────────────┤
│  Serial Port (serial_port.cpp)      │  ← 底层：Linux 原生串口 read()
├─────────────────────────────────────┤
│  Linux Kernel (/dev/hi12_imu)        │  ← 硬件抽象 (USB转串口)
│                                     │     也支持 GPIO UART4 (/dev/ttyS4)
└─────────────────────────────────────┘
```

---

## 二、硬件连接

```
HI12 传感器 (Molex 8-pin)     Orange Pi 5 Plus (40-pin GPIO)
     TX  ────────────────→  RX   (UART4_RX, 物理引脚 19)
     RX  ←────────────────  TX   (UART4_TX, 物理引脚 23)
     GND ────────────────  GND  (物理引脚 6/9/14/20/25/30/34/39)
     VDD ←───────────────  3.3V (物理引脚 1/17)
```

| 参数 | 值 |
|------|-----|
| 接口 | UART (通用异步收发) |
| 设备路径 | `/dev/hi12_imu` (USB转串口) 或 `/dev/ttyS4` (GPIO UART4) |
| 默认波特率 | 115200 (8N1) |
| 模块尺寸 | 22×22×10 mm, < 11g |
| 工作温度 | -40°C ~ 85°C |

---

## 二点五、生产部署配置

生产环境通过 CP210x USB-UART 连接（非直连 UART4），带 udev 规则确保设备名稳定。

### udev 规则

**`/etc/udev/rules.d/99-hi12-imu.rules`** — 固定设备名：
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="3c113646c6cdf011bcfb85c5e9520995", SYMLINK+="hi12_imu", MODE="0666"
```
→ 始终指向 CP210x，无论枚举为 ttyUSB0 还是 ttyUSB1

**`/etc/udev/rules.d/98-usb-power.rules`** — 禁用 USB Hub 自动休眠：
```
ACTION=="add", SUBSYSTEM=="usb", ENV{ID_VENDOR_ID}=="05e3", ENV{ID_MODEL_ID}=="0610", ATTR{power/control}:="on"
```
→ 防止 USB Hub runtime suspend 导致下游设备断开

### 启动脚本

`~/start_imu.sh` (PM2 管理)：
```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/expressone/ros2_ws/install/setup.bash
exec /home/expressone/ros2_ws/install/hi12_imu/lib/hi12_imu/hi12_imu_node \
    --ros-args -p device:=/dev/hi12_imu -p baudrate:=115200 -p frame_id:=imu_link -p publish_hz:=100
```

### 自动重连

驱动 `reader_loop()` 在串口出错后每 2 秒自动重试 `open()`，支持热插拔自动恢复，无需手动重启。

---

## 三、代码结构

```
~/ros2_ws/src/imu_4/
├── src/
│   ├── hi12_node.cpp          ← ROS2 节点入口 (main)
│   ├── hi12_driver.cpp        ← 驱动核心：串口操作、读线程、发布
│   ├── protocol.cpp           ← 协议解析：帧同步、CRC16、字段提取
│   └── serial_port.cpp        ← 串口封装：open/read/write/close
├── include/hi12_imu/
│   ├── hi12_driver.hpp        ← 驱动类声明
│   ├── protocol.hpp           ← 协议常量和函数声明
│   └── serial_port.hpp        ← 串口类声明
├── launch/
│   └── hi12_imu.launch.py     ← 启动文件
├── config/
│   └── hi12_params.yaml       ← 默认参数
├── CMakeLists.txt
└── package.xml
```

**构建产物**：
| 产物 | 类型 | 路径 |
|------|------|------|
| `hi12_imu_node` | 可执行文件 | `install/hi12_imu/lib/hi12_imu/` |
| `libhi12_imu_core.a` | 静态库 | `install/hi12_imu/lib/` |
| 头文件 | — | `install/hi12_imu/include/hi12_imu/` |

---

## 四、依赖

### 4.1 系统依赖

| 依赖 | 说明 |
|------|------|
| Linux 内核 ≥ 5.10 | 串口驱动 (`/dev/ttyS4`) |
| pthread | 后台读线程 |
| GCC ≥ 9 / Clang ≥ 14 | C++17 编译 |

### 4.2 ROS2 依赖

| 包 | 用途 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |
| `sensor_msgs` | `sensor_msgs/Imu` 消息类型 |
| `std_msgs` | 标准消息类型 |
| `ament_cmake` | 构建系统 |

### 4.3 编译要求

```bash
# CMakeLists.txt 配置
cmake_minimum_required(VERSION 3.8)
CMAKE_CXX_STANDARD 17
```

---

## 五、快速上手

### 5.1 构建

```bash
ssh expressone@192.168.8.30

# 交互式登录后
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh
cd ~/ros2_ws
colcon build --packages-select hi12_imu
```

### 5.2 快捷命令

为了方便，`imu` 命令已安装到 `/usr/local/bin/imu`，无需手动 source：

```bash
imu data     # 实时查看 IMU 数据
imu hz       # 查看发布频率
imu info     # 查看话题信息
imu          # 显示帮助
```

### 5.3 启动

```bash
# 默认参数（/dev/ttyS4, 115200, 200Hz 上限）
ros2 launch hi12_imu hi12_imu.launch.py

# 自定义参数
ros2 launch hi12_imu hi12_imu.launch.py \
    device:=/dev/hi12_imu \
    baudrate:=460800 \
    frame_id:=imu_link \
    publish_hz:=500
```

### 5.3 验证

```bash
# 查看话题列表
ros2 topic list | grep imu

# 查看实时数据
ros2 topic echo /imu

# 查看发布频率
ros2 topic hz /imu
# 预期输出: average rate: 100.0 Hz
```

### 5.4 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device` | `/dev/hi12_imu` | 串口设备路径 (USB转串口, 也支持 `/dev/ttyS4`) |
| `baudrate` | `115200` | 波特率 (460800 支持 500Hz) |
| `frame_id` | `imu_link` | TF 坐标系名 |
| `publish_hz` | `200` | 最大发布频率 (0 = 不限制) |
| `params_file` | `config/hi12_params.yaml` | YAML 参数文件 |

---

## 六、数据协议（简要）

### 6.1 帧结构

```
┌──────┬──────┬──────┬──────────────────┐
│ 5A A5│ 长度  │ CRC16│  76 字节数据体      │
│ 帧头  │ 2 字节 │ 2 字节│                    │
└──────┴──────┴──────┴──────────────────┘
  ←──────────── 82 字节/帧 ──────────────→
```

### 6.2 数据体字段（76 字节）

| 偏移 | 字节数 | 字段 | 类型 |
|------|--------|------|------|
| 0 | 1 | 产品 ID (0x91) | uint8 |
| 8-11 | 4 | 时间戳 | uint32 LE (ms) |
| 12-15 | 4 | 加速度 X | float32 LE (G) |
| 16-19 | 4 | 加速度 Y | float32 LE (G) |
| 20-23 | 4 | 加速度 Z | float32 LE (G) |
| 24-27 | 4 | 陀螺仪 X | float32 LE (°/s) |
| 28-31 | 4 | 陀螺仪 Y | float32 LE (°/s) |
| 32-35 | 4 | 陀螺仪 Z | float32 LE (°/s) |
| 48-51 | 4 | 欧拉角 Pitch | float32 LE (度) |
| 52-55 | 4 | 欧拉角 Roll | float32 LE (度) |
| 56-59 | 4 | 欧拉角 Yaw | float32 LE (度) |
| 60-63 | 4 | 四元数 W | float32 LE |
| 64-67 | 4 | 四元数 X | float32 LE |
| 68-71 | 4 | 四元数 Y | float32 LE |
| 72-75 | 4 | 四元数 Z | float32 LE |

> 驱动内部自动完成单位转换：加速度 G×9.80665→m/s²，陀螺仪 °/s×π/180→rad/s

### 6.3 CRC 校验

- 算法：CRC16/CCITT
- 多项式：`0x1021`
- 初始值：`0x0000`
- 校验范围：帧头 + 长度 + 数据体（CRC 字段本身除外）
- 不通过 → 丢弃整帧，计数但不中断

---

## 七、开发者 API

如果要在你自己的 ROS2 节点中消费 IMU 数据，不需要链接 IMU 驱动库，直接订阅话题即可。

### 7.1 方式一：订阅 /imu/data（推荐）

```cpp
// C++
#include "sensor_msgs/msg/imu.hpp"

auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10,
    [](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // 加速度
        double ax = msg->linear_acceleration.x;  // m/s²
        // 角速度
        double gz = msg->angular_velocity.z;     // rad/s (偏航转速)
        // 方向
        double qw = msg->orientation.w;          // 四元数
    });
```

```python
# Python
from sensor_msgs.msg import Imu

def imu_callback(msg: Imu):
    ax = msg.linear_acceleration.x   # m/s²
    gz = msg.angular_velocity.z      # rad/s

node.create_subscription(Imu, '/imu', imu_callback, 10)
```

### 7.2 方式二：链接驱动库（高级）

如果需要在非 ROS 环境中直接读串口解析 IMU，可以链接 `libhi12_imu_core.a`：

```cpp
#include "hi12_imu/hi12_driver.hpp"
#include "hi12_imu/protocol.hpp"

// 打开串口、启动读线程、接收解析后的数据
hi12_imu::HI12Driver driver;
driver.open("/dev/hi12_imu", 115200);
driver.set_data_callback([](const sensor_msgs::msg::Imu &msg) {
    // 处理 IMU 数据
});
driver.start();
```

**头文件**：`~/ros2_ws/install/hi12_imu/include/hi12_imu/`
**库文件**：`~/ros2_ws/install/hi12_imu/lib/libhi12_imu_core.a`

编译链接：
```bash
g++ your_program.cpp \
    -I~/ros2_ws/install/hi12_imu/include \
    ~/ros2_ws/install/hi12_imu/lib/libhi12_imu_core.a \
    -lrclcpp -lpthread
```

### 7.3 接入方式对比

| 方式 | 适用场景 | 是否需 ROS2 |
|------|---------|------------|
| 订阅 `/imu` | 自己的 ROS2 节点读取姿态 | 是 |
| 链接 `libhi12_imu_core.a` | 非 ROS 程序直接读串口 | 否 |
| 订阅话题（任意语言） | 任何支持 ROS2 的客户端 | 是 |

---

## 八、性能指标

| 指标 | 值 |
|------|-----|
| 输出频率 | **100 Hz** (默认, 10ms 间隔) |
| 最高频率 | 500 Hz (需 baudrate=460800) |
| CRC 错误率 | **0%** |
| 解析错误率 | **0%** |
| 加速度量程 | ±16G |
| 陀螺仪量程 | ±2000°/s |
| 静态姿态精度 | Roll/Pitch < 0.2°, Yaw < 1° |

---

## 九、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 启动报 `No such file or directory` | 串口设备路径错误 | `ls /dev/hi12_imu /dev/ttyS*` 确认设备；`pm2 restart hi12_imu` 重启节点 |
| `/imu` 无数据 | 串口参数错误、传感器未供电或 USB 断开 | 检查波特率 (115200)；`tail ~/.pm2/logs/hi12-imu-error.log` 查看错误统计 |
| 热插拔后无数据 | 旧版驱动无重连逻辑 | 已修复：驱动每 2 秒自动重连；若仍无数据 `pm2 restart hi12_imu` |
| `ros2 topic hz` 频率不稳 | CPU 负载高 | 降低 `publish_hz`；`sudo renice -20 -p $(pgrep hi12_imu_node)` |
| 大量 CRC 错误 | 电磁干扰或接线松动 | 检查 GND 接地；缩短杜邦线长度；换屏蔽线 |
| 数据值异常（如加速度一直是 0） | 协议帧格式不匹配 | 确认传感器型号为 HI12（产品 ID = 0x91） |
| 设备名漂移 (ttyUSB0→ttyUSB1) | USB 枚举顺序不稳 | 已修复：udev 规则 `/dev/hi12_imu` 永久绑定设备序列号 |

```bash
# 手动抓包验证（传感器是否在发数据）
sudo cat /dev/hi12_imu | xxd | head -100

# 如果看到规律性的 5a a5 帧头，说明硬件没问题
```

---

## 十、已踩过的坑

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | 所有帧被静默丢弃 | 错误假设数据格式是"寄存器地址+数据"的键值对，把产品 ID `0x91` 当寄存器地址导致查表返回 0 | 重写 `parse_payload()`，改为按固定偏移量解析 76 字节数据体 |
| 2 | 加速度/角速度值不对 | 传感器输出 G 和 °/s，ROS 要求 m/s² 和 rad/s | 驱动内做单位转换 |
| 3 | 上电后 `/imu` 无数据 | USB Hub autosuspend → CP210x 断开 → 设备名从 ttyUSB0 漂移到 ttyUSB1 → 驱动无重连逻辑 | ① udev 规则固定 `/dev/hi12_imu`；② 禁用 USB Hub autosuspend；③ `reader_loop()` 增加自动重连 |

---

## 十一、重新编译与重启

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
colcon build --packages-select hi12_imu
# 编译后重启节点
pm2 restart hi12_imu
```

---

*文档日期：2026-06-16 | HI12M0-MI1-000 | Orange Pi 5 Plus / ROS2 Jazzy*
