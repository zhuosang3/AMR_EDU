# 改造计划：HLS_canopen → ROS cmd_vel 接口

> 将当前 CLI REPL（RPM 指令）改造为标准 AMR 接口：线速度 (m/s) + 角速度 (rad/s)
> 日期: 2026-05-23

---

## 一、现状分析

当前 6 个源文件的架构：

```
main.rs         ← CLI REPL（go/left/right/back 等 RPM 命令）
motor.rs        ← 单轮控制（set_velocity(node_id, rpm)）
kinematics.rs   ← 差速运动学（inverse(v, omega) → left/right RPM）
sdo_client.rs   ← SDO 协议
can_driver.rs   ← CanBus trait + SocketCanBus + MockCanBus
can_setup.rs    ← CAN 接口配置
```

**好消息：`kinematics.rs` 已经是标准接口！**

`DiffDrive::inverse(v: f64, omega: f64) -> (f64, f64)` 直接把 (m/s, rad/s) 转 (left_rpm, right_rpm)。但 `main.rs` **从未调用过它** — 当前 CLI 用粗糙的 `do_left/do_right`（一个轮=0，一个轮=rpm）而不是运动学模型。

---

## 二、需要改造的文件

### ① `kinematics.rs` — 补充参数 + 速度限幅

| 改动 | 说明 |
|------|------|
| 暴露 `wheel_radius` / `wheel_base` 为 `pub` | 可能需要运行时可调（不同底盘参数不同） |
| 添加 `clamp_rpm()` 方法 | 限制输出 RPM 在 `[-max_rpm, max_rpm]` 内（电机额定 200rpm） |
| 添加真实底盘参数 | 轮径、轮距需要从 EXP1 底盘获取（当前测试值 0.1m/0.5m 是假的） |

**影响范围：** `DiffDrive` 结构体 + 2~3 个方法

### ② `main.rs` — 全量重写

当前 `main.rs` 是一个 CLI REPL，替换为 cmd_vel 输入循环。核心改动：

| 删除 | 新增 |
|------|------|
| `BANNER`, `HELP` 常量 | `cmd_vel` 输入解析 |
| `parse_rpm`, `do_go`, `do_left`, `do_right`, `do_back` | `DiffDrive` 实例化 + `inverse()` 调用 |
| `hls-canopen>` REPL prompt | 周期性发送速度指令到电机 |
| `status` 命令（保留但改为周期性上报） | 速度平滑（梯形加减速） |

**新 main.rs 流程：**

```
1. CAN 初始化（不变）
2. NMT 启动 + PV 模式 + 加减速 + 使能（不变）
3. 初始化 DiffDrive(wheel_radius, wheel_base)
4. 进入主循环：
   a. 从输入源读取 cmd_vel (v, ω)
   b. DiffDrive::inverse(v, ω) → (left_rpm, right_rpm)
   c. 限幅 clamp_rpm()
   d. 速度平滑（可选：ramp 到目标值）
   e. Motor::set_velocity(left, left_rpm)
   f. Motor::set_velocity(right, right_rpm)
   g. 周期性读取实际速度 + 状态（供 odom 上报）
```

### ③ 新增 `controller.rs`（可选，视复杂度）

如果主循环逻辑超过 ~80 行，抽取为独立模块：

```
controller.rs
  - VelocityController 结构体
  - 持有 DiffDrive + Motor
  - set_target(v, omega) → 运动学 → 限幅 → 平滑 → CAN 发送
  - read_odometry() → (v_actual, omega_actual)
  - 周期性 tick()（被 main 循环调用）
```

---

## 三、不需要改的文件

| 文件 | 原因 |
|------|------|
| `motor.rs` | `set_velocity(node_id, rpm)` 已经是标准 RPM 接口，无需改动 |
| `sdo_client.rs` | 纯 SDO 协议层，与 cmd_vel 无关 |
| `can_driver.rs` | CanBus trait 抽象正确，无需改动 |
| `can_setup.rs` | CAN 接口配置，无需改动 |

---

## 四、cmd_vel 输入源 —— 待决策的关键问题

需要确认「标准 ROS 框架接口」的具体含义：

### 方案 A：stdin JSON 流（最简）

```
echo '{"v": 0.5, "omega": 0.0}' | ./hls_canopen
```

- 改动最小：main.rs 从读 REPL 命令改为读 stdin JSON 行
- 无外部依赖
- 适合简单的上位机控制（Python/ROS node 通过 pipe 驱动）

### 方案 B：MQTT 订阅 cmd_vel

```
topic: /amr/cmd_vel   payload: {"v": 0.5, "omega": 0.0}
```

- 与公司现有 MQTT 架构一致（192.168.8.66:1883，`/amr/status/{id}`）
- 需要新增 `rumqttc` 依赖
- 与数字孪生 Bevy 项目对齐

### 方案 C：完整 ROS 2 节点（rclrs）

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...
```

- 真正的 ROS 标准接口，可与 nav2/move_base 等对接
- 依赖重量级（rclrs + ROS 2 运行时），不适合资源受限的开发板
- 需要交叉编译 ROS 2 库到 musl 目标（难度极高）

### 方案 D：UDP/TCP socket

- 轻量，无 broker 依赖
- 需要自定义协议

### 建议

**优先 B（MQTT）**，理由：
1. 公司架构已用 MQTT，复用现有 broker (192.168.8.66)
2. `rumqttc` 是纯 Rust，可 musl 静态编译，不增加系统依赖
3. 数字孪生 Bevy 项目正好订阅同一 topic，形成闭环
4. 比 ROS 2 轻 10 倍以上

同时保留方案 A（stdin JSON）作为调试/本地测试的备选输入。

---

## 五、底盘参数（待确认）

| 参数 | 当前测试值 | 实际值（需从 EXP1 底盘获取） |
|------|-----------|---------------------------|
| wheel_radius | 0.1m | ? |
| wheel_base | 0.5m | ? |
| max_rpm | 无限制 | 200 rpm（电机额定） |

EXP1 底盘参数需要从机械图纸或实际测量获取。

---

## 六、速度平滑策略（可选增强）

当前代码直接写目标速度到驱动器，没有 ramp。如果 cmd_vel 输入跳变（如从 0→1.0m/s），电机会以驱动器配置的加减速（1000 rpm/s）来响应。

是否需要控制器层做额外平滑取决于：
- 驱动器的加减速参数是否足够（当前 1000 rpm/s，约合 1.05 m/s² 对 0.1m 轮径）
- 是否需要独立于驱动器的 ramp 控制

先不改 — 依赖驱动器内置加减速，后续按需添加。

---

## 七、预估工作量

| 任务 | 改动量 | 优先级 |
|------|-------|--------|
| kinematics.rs 加限幅 + 真实参数 | ~20 行 | P0 |
| main.rs 重写（cmd_vel 循环） | ~100 行（替换 200 行） | P0 |
| 新增 MQTT 订阅 | ~50 行 + Cargo.toml | P1 |
| controller.rs 抽取（可选） | ~80 行 | P2 |
| 测试更新（cmd_vel 集成测试） | ~30 行 | P1 |

**总计：~170-280 行净增代码**（大部分是替换而非新增）

---

## 八、风险

1. **轮径/轮距未知** — 实测底盘参数之前运动学输出不准
2. **MQTT 与 CAN 线程模型冲突** — rumqttc 是异步的，CAN 操作是同步的。需要 mpsc channel 桥接（参考 Bevy 数字孪生的做法）
3. **musl + rumqttc 兼容性** — 需要验证 musl target 下 TLS 可用（rumqttc 依赖 native-tls/rustls）
