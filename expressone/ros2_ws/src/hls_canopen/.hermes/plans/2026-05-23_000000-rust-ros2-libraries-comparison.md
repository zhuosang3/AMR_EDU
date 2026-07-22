# Rust 接入 ROS2 的库选型调研

> 创建时间：2026-05-23
> 触发问题：rust 支持 ros2 的框架就只有 rclrs 包吗？没有别的库了吗？

---

## 目标

调研现有 Rust 语言接入 ROS2 生态的所有可选方案，给出对比评估和建议，为 HLS_canopen 接入 Nav2 铺路。

---

## 现状

- HLS_canopen 是纯 Rust + CANopen 电机控制项目，通过 WebSocket JSON 对外暴露 cmd_vel 输入和 odom 输出
- 当前不依赖任何 ROS2 库，未接入 ROS2 话题体系
- 需要发 `/odom`（nav_msgs/Odometry）+ `/tf`（odom→base_link），订阅 `/cmd_vel`（geometry_msgs/Twist）

---

## 调研发现：不止 rclrs，有三个主流选项

| 库 | 定位 | 核心机制 | 构建方式 | 维护方 |
|---|---|---|---|---|
| **rclrs** (ros2-rust) | 官方社区客户端库 | 封装 rcl C API + rosidl 代码生成 | 必须走 colcon + cargo | ros2-rust 社区（51+贡献者） |
| **r2r** | 轻量异步绑定 | 封装 rcl，绕过 .idl 管线，用已生成的 C 代码 | `cargo build` 即可 | Sequence Planner 团队 |
| **ros2-client** | 纯 Rust 原生客户端 | RustDDS（纯 Rust DDS），零 C/C++ 依赖 | `cargo build` 即可 | Atostek（芬兰） |

此外还有：
- **roslibrust** — 通过 rosbridge WebSocket 协议桥接，非原生 ROS2，适用轻量场景
- **rclrust** — 另一个 ROS2 Rust 客户端，维护活跃度低
- **HORUS** — 重写 ROS2 替代品，不与标准 ROS2 兼容

---

## 详细对比

### 1. rclrs (ros2-rust/ros2_rust)

- **GitHub**: `ros2-rust/ros2_rust`
- **状态**: FOSDEM 2026 确认将成为 ROS2 官方第三语言（继 C++/Python）
- **优点**:
  - 最接近"官方"地位，已进入 ROS 2 rolling 发行版
  - 社区最大（J&J、Intrinsic、Clearpath、Open-RMF 等在用）
  - API 设计与 rclcpp/rclpy 一致，ROS 用户迁移成本低
  - rosidl_generator_rs 自动生成所有 ROS2 消息类型的 Rust 绑定
- **缺点**:
  - **必须走 colcon 构建体系**，侵入 `~/.cargo`，不能独立 `cargo build`
  - 依赖 ROS2 完整工具链（rosdep、ament 等）
  - 文档仍较薄弱，示例少
  - 对 Humble/Jazzy 的包支持需要确认

### 2. r2r

- **GitHub**: `sequenceplanner/r2r`
- **crates.io**: `r2r` v0.6.4
- **论文**: ECRTS 2025 — "A First Look at ROS 2 Applications Written in Asynchronous Rust"
- **优点**:
  - **`cargo build` 即可**，不需要 colcon 侵入
  - 异步原生设计（futures + streams），适合实时系统
  - 绕过 .idl/.msg 管线，直接复用已安装的 ROS2 C 消息定义（编译快得多）
  - API 更 Rust 风格（而非 C++ 翻译）
- **缺点**:
  - 仍需系统安装 ROS2（依赖 rcl 共享库）
  - 社区较小，文档比 rclrs 更少
  - 可能不支持所有 ROS2 功能（Actions 支持有限？需验证）
  - 维护频率不如 rclrs 稳定

### 3. ros2-client

- **GitHub**: `Atostek/ros2-client`
- **crates.io**: `ros2-client`
- **ROSCon 2022 演讲**
- **优点**:
  - **纯 Rust**，零 C/C++ 依赖，不需要装 ROS2 工具链
  - 底层用 RustDDS（纯 Rust 的 DDS/RTPS 实现）
  - 完全独立的 cargo 项目，部署简单
  - 适合嵌入式 / 资源受限环境
- **缺点**:
  - **社区最小**，几乎只有 Atostek 一家维护
  - 消息类型支持有限（需要手工定义，无代码生成）
  - 不与标准 ROS2 工具链（ros2cli、rviz2 等）集成
  - DDS 实现可能存在互操作性问题（与 Fast-DDS / CycloneDDS 的兼容性）
  - 缺少 Nav2 所需的 nav_msgs、geometry_msgs 的完整支持（需验证）

---

## 快速决策矩阵

| 维度 | rclrs | r2r | ros2-client |
|------|-------|-----|-------------|
| 成熟度 | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| 构建独立性 | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| API 友好度 | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| 消息类型覆盖 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |
| 社区支持 | ⭐⭐⭐⭐ | ⭐⭐ | ⭐ |
| 实时/嵌入式友好 | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

---

## 选定方案：r2r ✅

**最终选择 r2r**，理由：
1. HLS_canopen 已有独立的 `cargo build` 体系，r2r 不需要 colcon 侵入，改动最小
2. 已熟悉 async Rust（tungstenite），r2r 的 async 设计吻合
3. nav_msgs/Odometry、geometry_msgs/Twist、tf2_msgs/TFMessage 这些基础消息类型 r2r 都支持
4. 比 rclrs 轻得多，编译快

---

## r2r 依赖清单（重要）

> "You need to source your ROS2 installation before building/running." — crates.io

### 机器上必须有的

| 依赖 | 说明 | 安装方式 |
|------|------|----------|
| **ROS2 base** | 提供 `librcl.so` + DDS 中间件 | `apt install ros-humble-ros-base` |
| **ROS2 消息包** | 提供 geometry_msgs、nav_msgs、tf2_msgs 等的 `.so` | 随 ros-base 自带 |
| **source setup.bash** | 构建和运行前必须执行 | `source /opt/ros/humble/setup.bash` |

### 不需要的

| 不依赖 | 说明 |
|--------|------|
| ❌ colcon | 不需要 rosdep / ament_cmake / package.xml |
| ❌ rosidl 代码生成 | 绕过 .msg/.idl 管线，复用已生成的 C 代码 |
| ❌ ros-humble-desktop | ros-base（约 200MB）就够了，不用装 GUI 工具 |

### r2r 工作原理

```
你的 Rust 代码
    ↓ (Cargo 依赖 r2r crate)
r2r (Rust FFI)
    ↓ (动态链接)
librcl.so ← /opt/ros/humble/lib/
    ↓
librmw_*.so ← DDS 实现（Fast-DDS / CycloneDDS）
    ↓
网络（DDS/RTPS 协议）
```

### 构建 & 运行流程

```bash
# 1. 一次性安装 ROS2 运行时
sudo apt install ros-humble-ros-base

# 2. 每次构建/运行前
source /opt/ros/humble/setup.bash

# 3. Cargo.toml 加依赖
# r2r = "0.6.4"

# 4. 构建
cargo build

# 5. 运行（运行时也需要 source 过的环境）
cargo run
```

### 部署到实车

实车（Jetson / 工控机）只需装 `ros-humble-ros-base`，不需要 colcon 那一套。编译产物是单个 Rust 二进制，`scp` 过去，确保 `LD_LIBRARY_PATH` 能找到 `/opt/ros/humble/lib` 就行。

---

## 下一步行动

1. 在开发机上 `apt install ros-humble-ros-base`，确认环境就绪
2. 写一个最小原型：r2r 节点发布 `/odom`（nav_msgs/Odometry） + 订阅 `/cmd_vel`（geometry_msgs/Twist），打印到终端
3. 将 HLS_canopen 的 `send_odom()` 输出桥接到 r2r 的 Odometry publisher
4. 部署到实车验证与 Nav2 联调

---

## 参考

- rclrs FOSDEM 2026: https://fosdem.org/2026/schedule/event/J8ZLKG-introducing_rclrs_the_official_ros_2_client_library_for_rust
- r2r crates.io: https://crates.io/crates/r2r
- ros2-client docs: https://docs.rs/ros2-client
- robotics.rs 汇总: https://robotics.rs
