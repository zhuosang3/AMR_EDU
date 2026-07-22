# keli_lidar — 科力 LSE-2027DE 激光雷达 ROS2 驱动

> Rust + r2r 实现，纯 UDP 协议，无 ROS1 依赖。

## 硬件要求

| 项目 | 规格 |
|------|------|
| 雷达型号 | 科力 LSE-2027DE（RJ45 以太网口） |
| 通信协议 | UDP，默认端口 **2112** |
| 数据帧长 | 1622 字节（4 子包拼接） |
| 测距范围 | 0.05 ~ 30m（可配置） |
| 扫描角度 | 270°（可裁剪） |

## 物理接线与网络

雷达用网线直连 Orange Pi 的以太网口。雷达默认 IP 为 `192.168.0.10`，需要改到与 Orange Pi 同一网段。

**如果 Orange Pi 在 `192.168.8.x` 网段**，有两种方式：

### 方式 A：改雷达 IP（推荐）

雷达上电后，用任意设备连接雷达的 UDP 端口发命令改 IP（具体协议参看 PDF 文档）。改完后雷达 IP 与 Orange Pi 同网段（如 `192.168.8.11`）。

### 方式 B：双网卡

用 Orange Pi 的第二个网口（或 USB 网卡）直连雷达，配置静态 IP `192.168.0.x`，雷达保持默认 `192.168.0.10`。

## 快速启动

```bash
# 1. 连接雷达并确认网络可达
ping 192.168.8.11

# 2. 启动驱动（默认连接 192.168.8.11:2112）
~/keli_lidar.sh

# 3. 另一个终端查看数据
ros2 topic echo /scan
```

### 启动脚本说明

`~/keli_lidar.sh` 封装了常用参数，直接执行即可：

```bash
# 使用默认 IP/端口启动
~/keli_lidar.sh

# 指定不同 IP
~/keli_lidar.sh --hostname 192.168.0.10

# 调试模式（输出更多日志）
~/keli_lidar.sh --debug
```

## 全部 CLI 参数

直接运行二进制支持以下参数（`--help` 查看）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--hostname` | `192.168.0.10` | 雷达 IP |
| `--port` | `2112` | 雷达 UDP 端口 |
| `--frame-id` | `laser` | LaserScan 消息的 frame_id |
| `--range-min` | `0.05` | 最小测距（米） |
| `--range-max` | `10.0` | 最大测距（米） |
| `--time-increment` | `0.00004` | 每个扫描点的时间间隔（秒） |
| `--time-offset` | `0.0` | 时间戳偏移（秒） |
| `--min-ang` | `-π` | 最小角度（弧度） |
| `--max-ang` | `π` | 最大角度（弧度） |
| `--intensity` | `true` | 是否包含强度数据 |
| `--timeout` | `5` | UDP 读取超时（秒） |
| `--skip` | `0` | 跳帧数（每 N 帧跳 1 帧） |
| `--debug` | `false` | 调试日志模式 |

示例：

```bash
# 改最大测距为 30m
keli_lidar --hostname 192.168.8.11 --range-max 30.0

# 只保留 -90°~90° 扫描范围
keli_lidar --hostname 192.168.8.11 --min-ang '-1.57' --max-ang '1.57'

# 跳半帧率（每帧跳 1 帧 → ~13.5 Hz）
keli_lidar --hostname 192.168.8.11 --skip 1
```

## 编译

```bash
# 确保 ROS2 环境
source /opt/ros/jazzy/setup.zsh

# 编译
cd ~/ros2_ws/src/keli_lidar
cargo build --release

# 二进制在 target/release/keli_lidar
```

**依赖**：
- ROS2 Jazzy（编译和运行都需要）
- Rust nightly（~1.98+）
- r2r 0.9+

## 验证

```bash
# 查看扫描频率
ros2 topic hz /scan

# 查看测距数据（前 10 个点）
ros2 topic echo /scan --once --field ranges

# 查看话题信息
ros2 topic info /scan

# rviz2 可视化
rviz2
# → Add → LaserScan → topic: /scan
```

正常输出频率约 **27~28 Hz**。

## 后台运行

```bash
# 方法 1：nohup
nohup keli_lidar --hostname 192.168.8.11 > /tmp/keli_lidar.log 2>&1 &

# 查看日志
tail -f /tmp/keli_lidar.log

# 方法 2：启动脚本 + nohup
nohup ~/keli_lidar.sh > /tmp/keli_lidar.log 2>&1 &
```

## 停止

```bash
killall keli_lidar
```

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 激光扫描数据（811 个测距点） |

## 文件结构

```
~/ros2_ws/src/keli_lidar/
├── Cargo.toml
├── README.md              ← 本文件
├── launch/
│   └── keli_lidar.launch.py
└── src/
    ├── main.rs            ← ROS2 节点入口
    ├── transport.rs       ← UDP 通信 + 分包重组
    └── protocol/
        ├── mod.rs
        ├── constants.rs       ← 协议常量（帧头、命令）
        ├── frame.rs           ← 帧结构解析
        └── ls1207de_parser.rs ← 解析 → LaserScan
```

## 常见问题

### Q: `ros2 topic echo /scan` 没数据？
- 确认雷达已上电
- 确认网络连通：`ping <雷达 IP>`
- 检查日志：`tail -f /tmp/keli_lidar.log`
- 确认 ROS2 环境：`source /opt/ros/jazzy/setup.zsh`

### Q: `Cannot connect to LiDAR`？
- 雷达 IP 和 Orange Pi 是否在同一子网？
- 雷达端口是否为 2112？
- 检查网线连接

### Q: 频率只有 ~23 Hz，文档说 27 Hz？
代码中扫描频率公式为 `(1000 / 43) * 100 = 2325`（单位 0.01Hz），即 23.25 Hz。实际实测约 **27~28 Hz**，差异来自雷达固件的实际扫描周期，不影响使用。

### Q: 移植来源？
此驱动从科力官方 ROS1 驱动（`sdkeli_ls_udp`）的`协议层`翻译为 Rust，ROS2 通信层用 r2r 实现。
