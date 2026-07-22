# EXP2 教育无人小车 — 系统架构文档

> Orange Pi 5 Plus | ROS2 Jazzy | 教学用途
> 部署位置：`expressone@192.168.8.30`

---

## 1. 总体架构

小车软件采用 **三层架构**：硬件驱动层 → ROS2 桥接层 → 应用/导航层。

```
┌──────────────────────────────────────────────────────────────────────┐
│                          硬件驱动层 (无 ROS2 依赖)                     │
│                                                                      │
│  hls_canopen (Rust)        power_monitor (C++)                      │
│  · CAN bus → CANopen 协议   · SocketCAN → BMS 电池数据                │
│  · 纯电机底层，不涉及 ROS2   · CAN ID 0x200/0x201/0x202               │
│  · 写入共享内存/pipe         · 发布 MQTT /car/power                   │
│  ❌ stopped                ✅ running (PM2)                          │
│                                                                      │
│  exp2_lights (C)                                                    │
│  · libgpiod 操作 GPIO                                               │
│  · 12 个 LED (4位置×3色)                                            │
│  · 纯 C 静态库，被 bridge 调用                                        │
│  ✅ running (PM2)                                                   │
├──────────────────────────────────────────────────────────────────────┤
│                         ROS2 Bridge 层 (协议转换)                      │
│                                                                      │
│  canopen_ros2 (Rust)       lights_bridge (C++)                      │
│  · 读共享内存 → Twist       · 订阅 /lights/command                   │
│  · 发布 /cmd_vel → CANopen  · 调用 libexp2_lights.a 控制 GPIO         │
│  ❌ stopped                ✅ running (PM2)                          │
├──────────────────────────────────────────────────────────────────────┤
│                         直接 ROS2 层 (原生节点)                        │
│                                                                      │
│  hi12_imu (C++)            keli_lidar (Rust)                        │
│  · /dev/ttyUSB0 读 HI12 IMU · 直接 ROS2，无 bridge                    │
│  · 发布 /imu (100Hz)        · 发布 /scan                             │
│  · 帧头 5A A5, CRC16        · 性能原因：导航强依赖激光雷达，压低延迟     │
│  ✅ running (PM2)           ✅ running (PM2)                         │
│                                                                      │
│  rsp (Python)               ekf (robot_localization)                │
│  · exp2_description 包       · 融合 /imu + /odom → /odom_filtered    │
│  · 发布 TF (URDF → tf2)      · EKF 姿态估计                           │
│  ✅ running (PM2)           ✅ running (PM2)                         │
└──────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
              ROS2 网络：/imu + /scan + /tf + /cmd_vel
                                 │
                                 ▼
┌──────────────────────────────────────────────────────────────────────┐
│                         应用层 (待开发)                               │
│                                                                      │
│  exp2_nav (Python/C++)                                              │
│  · Nav2 导航栈配置                                                   │
│  · SLAM (slam_toolbox / cartographer)                               │
│  · 路径规划 + 局部避障                                                │
│  · rviz2 可视化                                                      │
│  ⏳ 配置就绪，等待电机模块恢复                                          │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 2. PM2 进程清单

| ID | 进程名 | 状态 | 类型 | 脚本/路径 | 内存 |
|----|--------|------|------|---------|------|
| 0 | `hls_canopen` | ❌ stopped | 硬件驱动 | `~/hls_canopen/target/release/hls_canopen` | — |
| 1 | `canopen_ros2` | ❌ stopped | Bridge | `~/ros2_ws/src/canopen_ros2/target/release/canopen_ros2` | — |
| 2 | `keli_lidar` | ✅ online | 直接 ROS2 | `~/ros2_ws/src/keli_lidar/target/release/keli_lidar` | 33.8MB |
| 3 | `rsp` | ✅ online | ROS2 原生 | `ros2 launch exp2_description description.launch.py` | 38.3MB |
| 4 | `ekf` | ✅ online | ROS2 原生 | `~/scripts/start_ekf.sh` | 40.1MB |
| 5 | `lights_bridge` | ✅ online | Bridge | `~/start_lights.sh` | 31.1MB |
| 6 | `hi12_imu` | ✅ online | 直接 ROS2 | `~/start_imu.sh` (设备: /dev/ttyUSB0) | 28.9MB |
| 7 | `power_monitor` | ✅ online | 硬件驱动 | `~/start_power.sh` | 29.6MB |

---

## 3. 进程详解

### 3.1 hi12_imu（IMU 数据链路）✅

```
HI12 传感器 ──UART──→ CP2102N USB 转串口 (/dev/ttyUSB0)
                         │  115200 bps, 8N1
                         ▼
                      hi12_imu (C++, 直接 ROS2)
                         │  解析 5A A5 帧头, CRC16 校验
                         │  发布 sensor_msgs/Imu
                         ▼
                      ROS2 Topic: /imu (100Hz)
```

**当前方案**：`~/ros2_ws/src/imu_4/` 中的 `hi12_imu` 是 C++ 版本，直接在 ROS2 进程内读串口、解析、发布，单进程完成。

**硬件连接**：
- 设备路径：`/dev/ttyUSB0`（CP2102N USB 转串口适配器）
- 也支持 GPIO 直连 UART4：`/dev/ttyS4`（物理引脚 Pin 19(RX) / Pin 23(TX)）
- 串口参数：115200 bps, 8N1

**验证数据**：
```bash
sudo cat /dev/ttyUSB0 | xxd | head -20   # 应看到 5a a5 帧头规律出现
imu data                                   # 快捷命令查看实时数据
imu hz                                     # 查看发布频率（预期 100.0 Hz）
```

**旧方案（已停用）**：Rust 版 `imu_driver` + `imu_bridge` 通过共享内存通信的分层架构，目前已被 C++ 单进程方案替代。

### 3.2 hls_canopen → canopen_ros2（电机控制链路）❌

```
CAN bus 电机 ──CAN──→ hls_canopen (Rust, 纯驱动)
                         │  写入共享内存/pipe
                         ▼
                      canopen_ros2 (Rust, ROS2 封装)
                         │  订阅 /cmd_vel
                         │  发布 /odom
                         ▼
                      ROS2
```

**当前状态**：两个进程都停了。`hls_canopen` 重启计数器显示 2415 次，说明之前不断崩溃重启。

**可能原因**：
- CAN 硬件未连接或 CAN 接口未 up（`ip link set can0 up type can bitrate 500000`）
- CANopen 从站（电机驱动器）未上电或未响应
- 代码本身的 bug 导致进程退出

### 3.3 keli_lidar（激光雷达）✅

```
激光雷达 ──UART/USB──→ keli_lidar (Rust, 直接 ROS2)
                         │  直接发布 sensor_msgs/LaserScan
                         ▼
                      ROS2 Topic: /scan
```

**为什么不做 bridge 层？** 激光雷达是导航的核心传感器（建图、定位、避障都依赖它），对延迟敏感。多加一层共享内存会引入额外的数据拷贝和调度延迟。直接做 ROS2 节点，数据从串口到 `/scan` 话题的路径最短。

### 3.4 rsp（Robot State Publisher）✅

```
exp2.urdf.xacro ──→ robot_state_publisher
                       │  发布 /tf (static + dynamic)
                       │  发布 /robot_description
                       ▼
                    ROS2 TF 树
```

`rsp` 是 ROS2 标准节点，读取 URDF 机器人模型文件，持续广播各关节/传感器的坐标系变换关系。其他节点（Nav2、rviz2）依赖这些 TF 信息把激光数据、里程计等统一到同一个坐标系下。

---

## 4. 代码包结构

```
~/ros2_ws/src/
├── exp2_description/     # 机器人 URDF 模型 + rsp 启动
│   ├── urdf/
│   │   └── exp2.urdf.xacro      # 小车 3D 模型：底盘、轮子、传感器安装位置
│   ├── launch/
│   │   └── description.launch.py # 启动 robot_state_publisher
│   └── CMakeLists.txt
│
├── exp2_nav/             # 导航配置 (Theta* + RPP + SLAM Toolbox)
│   ├── config/                   # Nav2 参数文件
│   ├── launch/                   # 导航启动文件
│   ├── rviz/                     # rviz2 可视化配置
│   └── CMakeLists.txt
│
├── imu_4/                # IMU 驱动 C++ 版 (我们写的)
│   ├── src/
│   │   ├── protocol.cpp          # 帧解析、CRC 校验
│   │   └── hi12_driver.cpp       # ROS2 节点
│   ├── include/hi12_imu/
│   │   ├── protocol.hpp
│   │   └── hi12_driver.hpp
│   ├── launch/
│   │   └── hi12_imu.launch.py
│   └── config/
│
├── canopen_ros2/         # 电机 ROS2 封装 (Rust)
│   └── target/release/canopen_ros2
│
├── imu_bg/               # IMU Bridge (Rust) —— 注意：目录名是 imu_bg
│   └── target/release/imu_bridge
│
├── exp2_lights/          # 灯光 GPIO 驱动 (纯 C)
│   ├── lights.h
│   ├── lights.c
│   └── CMakeLists.txt
│
├── exp2_lights_bridge/   # 灯光 ROS2 桥接 (C++)
│   ├── src/lights_bridge.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
└── keli_lidar/           # 激光雷达驱动 (Rust, 直接 ROS2)
    └── target/release/keli_lidar

~/ (非 ROS2 的独立驱动)
├── hls_canopen/          # 电机底层驱动 (Rust, 不依赖 ROS2)
│   └── target/release/hls_canopen
│
└── imu_driver/           # IMU 底层驱动 (Rust, 不依赖 ROS2)
    └── target/release/imu_driver
```

---

## 5. 数据流全图（端到端）

```
                           ┌──────────────┐
                           │  HI12 IMU    │
                           │  传感器       │
                           └──────┬───────┘
                                  │ UART (/dev/ttyUSB0)
                                  ▼
                           ┌──────────────┐
                           │  hi12_imu    │  (C++, 直接 ROS2)
                           └──────┬───────┘
                                  │ /imu (100Hz)
                                  ▼
┌──────────────┐           ┌─────────────────────────────────┐
│  激光雷达      │  /scan    │                                 │
│  keli_lidar   │ ────────→│          ROS2 网络               │
└──────────────┘           │                                 │
                           │   /imu  +  /scan  +  /tf   │
┌──────────────┐  /tf      │         +  /odom  +  /cmd_vel   │
│  rsp         │ ────────→│                                 │
│  (URDF→TF)   │           └────────────┬────────────────────┘
└──────────────┘                        │
                                        ▼
                           ┌──────────────────────┐
                           │  Nav2 / SLAM         │
                           │  · 建图              │
                           │  · 定位              │
                           │  · 路径规划           │
                           │  · 避障              │
                           └──────────┬───────────┘
                                      │ /cmd_vel (Twist)
                                      ▼
                           ┌──────────────────────┐
                           │  canopen_ros2        │  (Rust, ROS2)
                           └──────────┬───────────┘
                                      │ 共享内存
                                      ▼
                           ┌──────────────────────┐
                           │  hls_canopen         │  (Rust, 纯驱动)
                           └──────────┬───────────┘
                                      │ CAN bus (CANopen)
                                      ▼
                           ┌──────────────────────┐
                           │  电机驱动器            │
                           │  (轮毂电机 ×2/4)       │
                           └──────────────────────┘
```

---

## 6. 当前状态总结

| 模块 | 状态 | 说明 |
|------|------|------|
| IMU 数据链路 | ✅ 正常 | hi12_imu (C++) 在线，/dev/ttyUSB0，/imu 100Hz 发布 |
| 激光雷达 | ✅ 正常 | keli_lidar 在线，发布 /scan |
| TF 坐标变换 | ✅ 正常 | rsp 在线，URDF 模型发布中 |
| 电机控制 | ❌ 停摆 | hls_canopen 和 canopen_ros2 都停了 |
| 灯光系统 | ✅ 正常 | exp2_lights_bridge PM2 管理，快捷命令 `light` |
| 电池监控 | ✅ 正常 | power_monitor 通过 CAN 读取电量，MQTT 发布 /car/power，快捷命令 `power` |
| 导航 | ⏳ 待命 | SLAM Toolbox + Theta* (全局) + RPP (局部)，等电机恢复即可调试 |
| IMU 驱动 (C++) | ✅ 正常 | hi12_imu (imu_4)，PM2 管理，/dev/ttyUSB0，/imu 100Hz，快捷命令 `imu` |
| IMU 驱动 (Rust) | ❌ 停用 | 旧版 imu_driver + imu_bridge，已被 C++ 版替代 |

---

## 7. 常用命令

```bash
# SSH 到小车
ssh expressone@192.168.8.30

# 查看所有进程状态
pm2 list

# 查看某个进程日志
pm2 logs hi12_imu
pm2 logs keli_lidar

# 启停进程
pm2 start hls_canopen
pm2 stop hi12_imu

# 查看 ROS2 话题
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /scan
ros2 topic hz /imu        # 查看实际发布频率

# 查看 TF 树
ros2 run tf2_tools view_frames

# 构建所有包
cd ~/ros2_ws && colcon build

# 修复电机可能需要
ip link set can0 up type can bitrate 500000   # 启用 CAN 接口
```

---

## 8. 各模块之间的通信机制

```
┌──────────────────────────────────────────────────────────┐
│  进程间通信方式一览                                        │
│                                                          │
│  硬件驱动 ←→ Bridge:  共享内存 (shm) 或 pipe              │
│                       · 低延迟                            │
│                       · 单机内，不走网络栈                  │
│                                                          │
│  Bridge → ROS2:      DDS (Fast-DDS / Cyclone DDS)       │
│                       · 发布/订阅模型                      │
│                       · 支持分布式（多机）                  │
│                                                          │
│  ROS2 → Nav2:        ROS2 Action / Service / Topic       │
│                       · Nav2 是标准 ROS2 导航框架          │
│                                                          │
│  CAN bus ←→ 电机:    CANopen 协议 (CiA 301/402)          │
│                       · 工业标准电机控制协议                 │
│                       · 位置/速度/力矩模式                  │
└──────────────────────────────────────────────────────────┘
```

---

*文档更新日期：2026-06-16 | 基于 Orange Pi 5 Plus 实际部署环境*
