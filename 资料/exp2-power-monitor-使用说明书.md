# EXP2 小车电池监控 — 使用说明书

> BMS CAN 总线 → MQTT 电量发布 | Orange Pi 5 Plus | ROS2 Jazzy
> 部署位置：`expressone@192.168.8.30:~/ros2_ws/src/power_monitor/`

---

## 一、系统原理

```
BMS 电池管理模块                  Orange Pi 5 Plus                    MQTT 订阅者
     │                                  │                                │
     │  CAN bus (500kbps)               │                                │
     │  0x200: 电压/电流                │                                │
     │  0x201: 容量/RSOC               │                                │
     │  0x202: 温度/保护状态            │                                │
     │ ──────────────────────────────→ │                                │
     │     BMS 每秒自报一次             │  CAN 后台线程持续读取              │
     │                                  │  解析 3 个 CAN ID               │
     │                                  │  提取 RSOC（剩余电量百分比）       │
     │                                  │  每 5 秒通过 MQTT 发布 ─────────→ │
     │                                  │  话题: /car/power               │
     │                                  │  格式: {"power":79,"charge_flag":1,"current":602}             │
```

**数据来源**：BMS（Battery Management System，电池管理系统）通过 CAN 总线主动上报，波特率 500k，11 位标准帧 ID。驱动只解析 `0x200`、`0x201`、`0x202` 三个 ID，其余全部屏蔽。

---

## 二、代码结构

```
~/ros2_ws/src/power_monitor/
├── src/
│   ├── power_monitor_node.cpp    ← ROS2 节点入口，组装各组件
│   ├── battery_protocol.cpp      ← CAN 帧解析 (0x200/0x201/0x202)
│   ├── can_reader.cpp            ← SocketCAN 封装：打开、读线程、数据缓存
│   └── mqtt_publisher.cpp        ← MQTT 封装 (libmosquitto)：连接、发布、重连
├── include/power_monitor/
│   ├── battery_protocol.hpp      ← BatteryData 结构体 + 解析声明
│   ├── can_reader.hpp            ← CanReader 类声明
│   └── mqtt_publisher.hpp        ← MqttPublisher 类声明
├── CMakeLists.txt
└── package.xml
```

**构建产物**：
| 产物 | 路径 |
|------|------|
| 可执行文件 | `install/power_monitor/lib/power_monitor/power_monitor_node` |
| 静态库 | `install/power_monitor/lib/libpower_monitor_core.a` |

---

## 三、依赖

### 3.1 系统依赖

| 依赖 | 说明 |
|------|------|
| Linux SocketCAN | CAN 接口驱动（`can0`） |
| libmosquitto | MQTT 客户端库 |
| Mosquitto broker | MQTT 消息中间件（通常 `localhost:1883`） |

### 3.2 ROS2 依赖

| 包 | 用途 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |

### 3.3 CAN 接口要求

驱动假设 CAN 接口已配置好 500kbps。如未配置：

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

验证：
```bash
ip -details link show can0
# 应显示: bitrate 500000
```

---

## 四、CAN 协议摘要

BMS 每秒自报一次，驱动解析以下三个 CAN ID：

### 0x200 — 电压/电流（8 字节）

| 字节 | 字段 | 类型 | 单位 |
|------|------|------|------|
| 0-1 | 总电压 | uint16 BE | 10mV |
| 2-3 | 电流 | int16 BE | 10mA（充电为正，放电为负） |
| 4-5 | 最低单体电压 | uint16 BE | mV |
| 6-7 | 最高单体电压 | uint16 BE | mV |

### 0x201 — 容量（8 字节）

| 字节 | 字段 | 类型 | 单位 |
|------|------|------|------|
| 0-1 | 剩余容量 | uint16 BE | 10mAh |
| 2-3 | 充满容量 | uint16 BE | 10mAh |
| 4-5 | 循环次数 | uint16 BE | 次 |
| 6-7 | **RSOC (电量%)** | uint16 BE | **%** |

> RSOC 就是 MQTT 发布的 `power` 值。

### 0x202 — 温度/状态（8 字节）

| 字节 | 字段 | 类型 | 单位 |
|------|------|------|------|
| 0-1 | 保护标志 | uint16 BE | 位掩码 |
| 2 | 充电标志 | uint8 | 0=未充电, 1=充电中 |
| 3 | MOSFET 状态 | uint8 | bit0=充电 MOS, bit1=放电 MOS |
| 4-5 | 最高温度 | int16 BE | 0.1℃ |
| 6-7 | 最低温度 | int16 BE | 0.1℃ |

**保护标志位**：SOC < 30% 报警 (bit14)，SOC < 10% 保护休眠 (bit15)。

---

## 五、快速上手

### 5.0 power 命令输出示例

```bash
$ power status
正在获取电量...
当前电量: 98%
[状态] 未充电
[放电] 1.22A

# 充电中时显示:
$ power status
正在获取电量...
当前电量: 45%
[状态] 正在充电
[电流] +6.02A

# 未充电且电量低时:
$ power status
正在获取电量...
当前电量: 25%
[状态] 未充电
[放电] 2.10A
[提醒] 电量偏低，建议充电
```

> `charge_flag` 来自 BMS 0x202 帧 BYTE2：0=未充电, 1=充电中。
> `current` 来自 BMS 0x200 帧 BYTE2-3：正值=充电电流，负值=放电电流，单位 10mA。

### 5.1 前置条件

```bash
# 确认 CAN 接口
ip -details link show can0 | grep bitrate

# 确认 MQTT broker 运行
systemctl status mosquitto
```

### 5.2 构建

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
colcon build --packages-select power_monitor
```

### 5.3 快捷命令

`power` 命令已安装到 `/usr/local/bin/power`：

```bash
power status    # 当前电量 + 充电状态
power watch     # 实时监控 (每 5 秒)
power info      # 话题信息
power           # 帮助
```

### 5.4 启动

```bash
# 默认参数
ros2 run power_monitor power_monitor_node

# 自定义参数
ros2 run power_monitor power_monitor_node \
    --ros-args \
    -p can_interface:=can0 \
    -p mqtt_host:=localhost \
    -p mqtt_port:=1883 \
    -p mqtt_topic:=/car/power \
    -p publish_interval_s:=5

# 后台运行
nohup ros2 run power_monitor power_monitor_node > /tmp/power_monitor.log 2>&1 &
```

### 5.5 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | CAN 接口名 |
| `mqtt_host` | `localhost` | MQTT 服务器地址 |
| `mqtt_port` | `1883` | MQTT 端口 |
| `mqtt_topic` | `/car/power` | 发布的 MQTT 话题 |
| `publish_interval_s` | `5` | 发布间隔（秒） |

---

## 六、开发者 API

### 6.1 方式一：订阅 MQTT（推荐，语言无关）

```bash
# 命令行
mosquitto_sub -t '/car/power'
# 输出: {"power":79,"charge_flag":1,"current":602}
```

```python
# Python (需要 paho-mqtt)
import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    print(f"电量: {msg.payload.decode()}")

client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)
client.subscribe("/car/power")
client.loop_forever()
```

```cpp
// C++ (需要 libmosquitto)
#include <mosquitto.h>
// 初始化 mosquitto，连接 localhost:1883，订阅 /car/power
```

### 6.2 方式二：链接静态库

如果需要在非 ROS 环境中直接读 CAN 总线解析电池数据，可以链接 `libpower_monitor_core.a`：

**头文件**：`~/ros2_ws/install/power_monitor/include/power_monitor/`
**库文件**：`~/ros2_ws/install/power_monitor/lib/libpower_monitor_core.a`

```cpp
#include "power_monitor/battery_protocol.hpp"

BatteryData data;
// 从 CAN 帧原始字节调用 parse_can_frame()
parse_can_frame(0x201, raw_data, 8, data);
printf("电量: %u%%\n", data.rsoc);
```

### 6.3 方式对比

| 方式 | 适用场景 | 依赖 |
|------|---------|------|
| 订阅 MQTT | 任何语言/平台，读取电量 | MQTT 客户端 |
| 链接静态库 | C++ 程序直接解析 CAN 帧 | libmosquitto |
| `power` 命令 | 命令行快速查看 | 无（自动 source） |

---

## 七、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| `power status` 无输出 | power_monitor 节点未启动 | `pm2 start power_monitor` |
| 充电器已拔但显示"正在充电" | CAN 读取线程卡死，MQTT 发布旧数据 | `pm2 restart power_monitor` |
| 数据长时间不更新（冻结） | `CAN_RAW_FD_FRAMES` 导致帧截断/读取失败 | 更新代码后 `colcon build && pm2 restart power_monitor` |
| 一直显示 "No CAN data yet" | CAN 接口未配置或 BMS 未上电 | `sudo ip link set can0 up type can bitrate 500000` |
| MQTT 连接失败 | Mosquitto broker 未运行 | `sudo systemctl start mosquitto` |
| 电量值为 0 | BMS 未连接到 CAN 总线 | 检查 CAN 物理接线 |
| `candump can0` 无输出 | CAN 接口未 up 或硬件问题 | `sudo ip link set can0 up` |
| 电机/电池同时无法通信 | CAN 波特率不匹配 (被改为 1M) | 改回 500k: `sudo ip link set can0 type can bitrate 500000` |

```bash
# 手动检查 CAN 数据 (对比 MQTT 数据是否一致)
candump can0,200:202          # 只看 BMS 帧
mosquitto_sub -t '/car/power' -C 1   # 看 MQTT 数据

# 如果 CAN 数据与 MQTT 不一致，重启恢复
pm2 restart power_monitor
```

---

## 八、重新编译

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
colcon build --packages-select power_monitor
pm2 restart power_monitor
```

---

## 九、已知问题与修复记录

### 9.1 充电状态冻结问题（2026-06-21 修复）

**现象**：充电器拔掉后 `power status` 仍显示"正在充电"，电量卡在 97%，数据与 CAN 总线不一致。

**根因分析**：

1. **`CAN_RAW_FD_FRAMES` 导致帧截断**：`can_reader.cpp` 的 `open()` 中设置了 `CAN_RAW_FD_FRAMES` sockopt，内核将所有 CAN 帧以 `canfd_frame`（72 字节）格式投递。但 `read_loop()` 只用 `struct can_frame`（16 字节）缓冲区读取，导致帧被截断。在某些情况（如系统长时间运行、缓冲区累积）下，BMS 0x202 帧的 charge_flag 字段无法正确更新，数据冻结在拔充电器前的状态。

2. **读线程退出后无感知**：`read_loop()` 在 `select()`/`read()` 出错后 `break` 退出，但 `copy_out()` 只检查 `has_data_` 而不检查 `running_`，导致定时器回调持续发布冻结的旧数据。

**修复内容**（`can_reader.cpp`）：

| 修改 | 说明 |
|------|------|
| 移除 `setsockopt(CAN_RAW_FD_FRAMES)` | CAN 总线只有经典 CAN 帧（DLC=8），不需要 FD 支持 |
| `copy_out()` 增加 `!running_.load()` 检查 | 读线程异常退出时返回 false，不再发布旧数据 |

**验证方法**：
```bash
# 对比 CAN 原始数据与 MQTT 数据是否一致
candump can0,200:202 & sleep 2 && mosquitto_sub -t '/car/power' -C 1

# CAN 0x201 的 RSOC 字段应等于 MQTT power 值
# CAN 0x202 的 charge_flag 应等于 MQTT charge_flag 值
# CAN 0x200 的 current 字段应等于 MQTT current 值
```

### 9.2 `power` 脚本改进（2026-06-21）

- 增加 `未充电` 状态显示（之前只显示充电中和低电量警告）
- 增加放电电流显示（`current` 负值 → `[放电] X.XXA`）
- 充电时显示充电电流（`current` 正值 → `[电流] +X.XXA`）

---

*文档日期：2026-06-21 | Orange Pi 5 Plus / ROS2 Jazzy / BMS CAN 协议*
