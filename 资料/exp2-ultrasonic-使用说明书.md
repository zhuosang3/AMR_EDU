# EXP2 小车超声波驱动 — 使用说明书

> DYP-A21-V1.0 超声波传感器 (CAN 模式) | Orange Pi 5 Plus | ROS2 Jazzy
> 当前 CAN 接口：**can0** @ **500K**（与电池/电机共享总线）
> 部署位置：`expressone@192.168.8.30:~/ros2_ws/src/dyp_a21_ultrasonic/`
> 最后更新：2026-06-21

---

## 一、系统原理

### 1.1 超声波传感器是什么

DYP-A21 通过发射超声波脉冲并测量回波时间来测距。本系统使用 **CAN 总线模式**：

```
DYP-A21 (车头)              DYP-A21 (车尾)              Orange Pi 5 Plus              ROS2 网络
   │                            │                            │                              │
   │  CANL/CANH                  │  CANL/CANH                  │                              │
   │  CANID 0x528                │  CANID 0x529                │                              │
   │  (地址 0x08)                │  (地址 0x09)                │                              │
   │ ─────────────────────────── │ ───────────────────────────→│                              │
   │                            │                            │  ① CAN 发送读距离指令          │
   │                            │                            │  ② 等待传感器响应             │
   │                            │                            │  ③ 解析距离 (mm)             │
   │                            │                            │  ④ 发布 sensor_msgs/Range ──→│
   │                            │                            │  话题: /snoic/front     │
   │                            │                            │  话题: /snoic/back      │
```

### 1.2 CAN 总线拓扑

```
Orange Pi 5 Plus
     │
     ├── PCAN-USB FD ── can0 @ 500K ──┬── 车头传感器 (0x08)
     │                               ├── 车尾传感器 (0x09)
     │                               ├── 电机驱动 (DS20270DA)
     │                               └── 电池 BMS
     │
     └── PCAN-USB ── can1 @ 500K (备用)
```

两个传感器挂在同一条 CAN 总线上，通过不同的 CANID（= 0x0520 + 从机地址）区分。

### 1.3 分层架构

```
┌─────────────────────────────────────┐
│  ROS2 Node (ultrasonic_node.py)     │  ← 入口：创建节点、发布者、定时器 (2Hz)
├─────────────────────────────────────┤
│  python-can (socketcan)             │  ← CAN 驱动：收发 CAN 帧
├─────────────────────────────────────┤
│  SocketCAN (Linux Kernel)           │  ← 内核层：CAN 子系统
├─────────────────────────────────────┤
│  PEAK PCAN-USB (硬件)               │  ← USB 转 CAN 适配器
└─────────────────────────────────────┘
```

---

## 二、硬件连接

### 2.1 CAN 总线接线

```
DYP-A21 传感器 (4-pin)      CAN 总线
     Pin 1: VCC  ────────  电源 (3.3~24V)
     Pin 2: GND  ────────  地
     Pin 3: CANL ────────  CAN_L (PCAN-USB)
     Pin 4: CANH ────────  CAN_H (PCAN-USB)
```

> ⚠️ CAN 型号为 **DYP-A21*CAW-V1.0**（单角度）或 **DYP-A21BYYCAW-V1.0**（双角度），引脚功能与输出方式一一对应，不能与其他输出方式并存。

### 2.2 CAN 适配器

| 接口 | 设备 | 用途 |
|------|------|------|
| can0 | PEAK PCAN-USB FD | **超声波传感器 + 电机 + BMS（共享总线）** |
| can1 | PEAK PCAN-USB | 备用 |

| 参数 | 值 |
|------|-----|
| 帧格式 | CAN 2.0 标准帧 |
| 帧类型 | 数据帧 |
| 波特率 | **500Kbps**（与 can0 上其他设备统一） |
| CANID 基址 | 0x0520 |

---

## 三、传感器配置

| 传感器 | 从机地址 | CANID | ROS2 话题 | frame_id | 位置 |
|--------|---------|-------|-----------|----------|------|
| 传感器1 | 0x08 | 0x528 | `/snoic/front` | `ultrasonic_front` | 车头 |
| 传感器2 | 0x09 | 0x529 | `/snoic/back` | `ultrasonic_rear` | 车尾 |

---

## 四、代码结构

```
~/ros2_ws/src/dyp_a21_ultrasonic/
├── dyp_a21_ultrasonic/
│   └── ultrasonic_node.py     ← ROS2 节点 (CAN 驱动核心)
├── launch/
│   └── ultrasonic.launch.py   ← 启动文件
├── config/
│   └── params.yaml            ← 可调参数
├── scripts/
│   └── ultrasonic_node        ← 入口脚本
├── setup.py
├── package.xml
└── resource/
    └── dyp_a21_ultrasonic
```

**关键文件说明**：
| 文件 | 职责 |
|------|------|
| `ultrasonic_node.py` | CAN 初始化、传感器轮询、数据解析、话题发布 |
| `params.yaml` | CAN 接口名、波特率、传感器地址、话题名等全部可配 |

---

## 五、依赖

### 5.1 系统依赖

| 依赖 | 说明 |
|------|------|
| python3-can | SocketCAN Python 驱动 |
| SocketCAN | Linux 内核 CAN 子系统 |
| PCAN-USB 驱动 | `sudo modprobe peak_usb` |

### 5.2 ROS2 依赖

| 包 | 用途 |
|------|------|
| `rclpy` | ROS2 Python 客户端库 |
| `sensor_msgs` | `sensor_msgs/Range` 消息类型 |

### 5.3 安装

```bash
# 安装 python-can
sudo apt install -y python3-can

# 确认 CAN 硬件
lsusb | grep -i peak
# 预期: PEAK System PCAN-USB / PCAN-USB FD
```

---

## 六、快速上手

### 6.1 构建

```bash
ssh expressone@192.168.8.30

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon build --packages-select dyp_a21_ultrasonic
```

### 6.2 启动

```bash
# 一键启动
snoic
```

`snoic` 命令已安装到 `/usr/local/bin/snoic`，无需手动 source。

```bash
snoic          # 启动驱动
snoic front    # 查看车头距离
snoic back     # 查看车尾距离
snoic hz       # 查看发布频率
snoic info     # 查看话题信息
```

### 6.3 验证

```bash
# 启动后，另一个终端
snoic front    # 预期: range 字段在 0.03 ~ 5.0 之间变动
snoic back
snoic hz       # 预期: average rate: 2.0 Hz
```

### 6.4 预期输出

```
/snoic/front:
  frame_id: ultrasonic_front
  radiation_type: 0 (ULTRASOUND)
  field_of_view: 0.7
  min_range: 0.03
  max_range: 5.0
  range: 0.895    ← 当前距离 (米)
---
```

---

## 七、可调参数

### 7.1 `config/params.yaml`

```yaml
dyp_a21_ultrasonic:
  ros__parameters:
    can_interface: "can0"      # CAN 接口名 (can0 / can1)
    can_bitrate: 500000        # CAN 波特率 (250000, 500000, 1000000)，必须与总线统一
    publish_rate: 2.0          # 发布频率 (Hz)
    sensor_names: ["front", "rear"]  # 传感器列表

    front:
      address: 0x08            # 从机地址
      frame_id: "ultrasonic_front"
      topic: "/snoic/front"
      field_of_view: 0.7       # 波束角 (rad)
      min_range: 0.03          # 最小量程 (m)
      max_range: 5.0           # 最大量程 (m)

    rear:
      address: 0x09
      frame_id: "ultrasonic_rear"
      topic: "/snoic/back"
      field_of_view: 0.7
      min_range: 0.03
      max_range: 5.0
```

### 7.2 添加第三个传感器

```yaml
sensor_names: ["front", "rear", "left"]
left:
  address: 0x0A
  frame_id: "ultrasonic_left"
  topic: "/snoic/left"
  field_of_view: 0.7
  min_range: 0.03
  max_range: 5.0
```

---

## 八、CAN 数据协议

### 8.1 读距离（当前：回波时间模式）

> ⚠️ **2026-06-21 更新**：由于 PCAN-USB FD 适配器与传感器固件的兼容性问题，寄存器 0x0100/0x0101 在 can0 上始终返回 0。现改用**寄存器 0x010A（回波时间）**计算距离，更可靠且不受硬件差异影响。

**主机发送（读回波时间 0x010A）**：
```
CANID:    0x0520 + 从机地址
数据区:   [addr, 0x03, 0x01, 0x0A, 0x00, 0x01]  (6 字节)
```

**从机回应**：
```
CANID:    0x0520 + 从机地址
数据区:   [addr, 0x83, 0x02, data_H, data_L]     (5 字节)
```

**距离计算**：`distance_mm = (data_H * 256 + data_L) / 5.75`（回波时间 μs ÷ 5.75 = mm）

**错误值**：
| 值 | 含义 |
|------|------|
| 0xFFFE | 同频干扰 |
| 0xFFFD | 未检测到物体 |
| 0x0000 | 无回波（等同于未检测到） |

<details>
<summary>旧方案（寄存器 0x0101 实时值，已废弃）</summary>

**主机发送**：
```
CANID:    0x0520 + 从机地址
数据区:   [addr, 0x03, 0x01, 0x01, 0x00, 0x01]  (6 字节)
```
**距离计算**：`distance_mm = data_H * 256 + data_L`

</details>

### 8.2 修改从机地址（寄存器 0x0200）

```bash
# 将地址从 0x01 改为 0xXX (CANID 0x0521, 数据 6 字节)
cansend can0 521#0106020000XX
# 传感器回复: 0186020000XX 表示成功
# 之后使用新 CANID: 0x52XX
```

### 8.3 修改 CAN 波特率（寄存器 0x020A）

```bash
# 改为 500K（与 can0 总线统一）
cansend can0 5XX#XX06020A0007
# 0x04=100K  0x05=125K  0x06=250K  0x07=500K  0x08=1M
```

### 8.4 完整寄存器表（CAN 模式）

**只读寄存器**：
| 地址 | 功能 | 说明 |
|------|------|------|
| 0x0000 | 软件版本 | 十六进制 |
| 0x0100 | 处理值 | 算法处理后距离 (mm), 响应 190~750ms |
| 0x0101 | 实时值 | 实时距离 (mm), 响应 15~140ms |
| 0x0102 | 温度 | 单位 0.1℃ |
| 0x010A | 回波时间 | 单位 us, ÷5.75 = mm |

**读写寄存器**：
| 地址 | 功能 | 默认值 | 说明 |
|------|------|--------|------|
| 0x0200 | 从机地址 | 0x01 | 范围 0x01~0xFE, 0xFF=广播 |
| 0x0208 | 角度等级 | 4 | 1~4 级, 越大角度越大 |
| 0x020A | CAN 波特率 | 0x06 (250K) | 0x08=1Mbps |
| 0x021A | 降噪等级 | 1 | 1~5 级 |
| 0x021F | 量程等级 | 5 | 1=50cm ~ 5=500cm |

---

## 九、开发者 API

在自己的 ROS2 节点中订阅超声波数据：

```python
# Python
from sensor_msgs.msg import Range

def front_callback(msg: Range):
    distance_m = msg.range           # 当前距离 (米)
    distance_cm = distance_m * 100   # 厘米

node.create_subscription(Range, '/snoic/front', front_callback, 10)
node.create_subscription(Range, '/snoic/back', back_callback, 10)
```

```cpp
// C++
#include "sensor_msgs/msg/range.hpp"

auto sub = node->create_subscription<sensor_msgs::msg::Range>(
    "/snoic/front", 10,
    [](sensor_msgs::msg::Range::ConstSharedPtr msg) {
        double dist = msg->range;  // 米
    });
```

---

## 十、性能指标

| 指标 | 值 |
|------|-----|
| 发布频率 | **2 Hz** (可调) |
| 传感器响应时间 | 15~140 ms (实时值) |
| 盲区 | ≤ 3 cm |
| 量程 | 3 cm ~ 500 cm |
| 精度 | 1 + (S × 0.3%) cm |
| CAN 波特率 | **500 Kbps**（与电机/BMS 共享总线） |
| 距离数据源 | 寄存器 0x010A 回波时间 (echo us / 5.75) |
| 供电电压 | 3.3 ~ 24V |

---

## 十一、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 启动报 `No such file or directory` | CAN 接口未创建 | `sudo ip link add dev can0 type can` |
| 话题无数据 | CAN 波特率不匹配 | 确认传感器波特率: `ip -details link show can0` |
| 传感器不回应 | 地址错误 | 扫描 CAN 地址: `for addr in 01..10; do cansend can0 52X#...; done` |
| 大量 `0xFFFE` | 同频干扰 | 两个传感器不要面对面安装 |
| 数据一直为 0xFFFD | 超出量程或无目标 | 检查量程; 确认目标在有效探测范围内 |
| 距离始终为 0 | 寄存器 0x0101 在 PCAN-USB FD 上不兼容 | 改用 0x010A 回波时间寄存器（代码已默认），公式 mm = us / 5.75 |
| 个别传感器无话题数据 | 共享总线上其他设备帧抢占 | 确认代码已加 CAN ID 过滤循环（2026-06-21 修复） |
| `Network is down` | CAN 接口未 UP | `sudo ip link set can0 up` |

```bash
# 手动探测传感器
sudo ip link set can0 type can bitrate 250000 up
cansend can0 521#010301010001
candump can0
# 看到 018302XXXX 说明传感器在线

# 确认 CAN 设备
lsusb | grep -i peak
ip link show | grep can
```

---

## 十二、已踩过的坑

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 1 | 传感器配置了地址但新地址不响应 | 地址修改后下一次通讯才生效，切波特率时数据区要用新地址 | 先改地址 → 用新地址发波特率修改 → 最后切 CAN 口波特率 |
| 2 | 两个传感器同时在线时地址冲突 | 出厂默认都是 0x01 | 逐个传感器单独上电配置，配完一个再配下一个 |
| 3 | `ros2 run` 找不到可执行文件 | ROS2 Jazzy 需要 `lib/<pkg>/` 目录 | `data_files` 中安装到 `lib/dyp_a21_ultrasonic/` |
| 4 | 改波特率后传感器失联 | 改波特率后 CAN 总线也要同步切换 | 先发波特率修改指令，收到确认后立即切 `ip link set can0 type can bitrate 1000000` |
| 5 | PCAN-USB FD 上寄存器 0x0101 始终返回 0 | FD 控制器与传感器固件存在微妙寄存器兼容差异 | 改用寄存器 0x010A 回波时间计算距离 (mm = us / 5.75)，物理层时序无关，更可靠 |
| 6 | 共享 CAN 总线时后传感器无数据 | 电机/BMS 的大量 CAN 帧（0x581/0x601 等）抢占 `recv()`，传感器回复被丢弃 | `recv()` 加 CAN ID 过滤循环，跳过不匹配的帧直到收到目标传感器回复或超时 |
| 7 | 驱动启动后电池/电机短暂失联 | `_setup_can()` 无条件 down/up CAN 接口 | 改为先检查接口状态，已 UP 时不再操作，只创建 SocketCAN socket |

---

## 十三、常用命令速查

```bash
# === 日常使用 ===
snoic              # 启动驱动
snoic front        # 看车头距离
snoic back         # 看车尾距离
snoic hz           # 看发布频率

# === 编译 ===
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select dyp_a21_ultrasonic

# === CAN 底层调试 ===
sudo ip link set can0 type can bitrate 1000000 up   # 配置并启用 CAN0
candump can0                                         # 监听 CAN 总线
cansend can0 528#080301010001                        # 读车头传感器

# === 修改传感器配置 ===
cansend can0 521#010602000008                        # 改地址 0x01→0x08
cansend can0 528#0806020A0008                        # 改波特率→1Mbps
```

---

*文档日期：2026-06-18 | DYP-A21-V1.0 CAN | Orange Pi 5 Plus / ROS2 Jazzy*
