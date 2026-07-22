# WebSocket 库选型 — HLS_canopen cmd_vel 接口

> 日期: 2026-05-23
> 目标：为 HLS_canopen 选一个 WebSocket 库，接收上位机的 cmd_vel 指令

---

## 一、约束条件

| 约束 | 说明 |
|------|------|
| musl 静态编译 | 不能有 C 依赖或 glibc 绑定 |
| 运行平台 | 嵌入式 Linux（arm64/amd64） |
| 无 async runtime | 当前代码是同步的（阻塞 CAN socket），引入 tokio 会增加大幅复杂度 |
| 轻量 | 开发板资源有限，不能引入几十个依赖 |
| 频率要求 | 20 Hz cmd_vel，延迟容忍 50ms 以内 |

---

## 二、候选库对比

### tungstenite（推荐）

| 维度 | 评价 |
|------|------|
| 下载量 | ★★★★★ 26M+ total downloads |
| 依赖数 | ★★★★★ 7 个（几乎全是自己的子 crate） |
| 纯 Rust | ★★★★★ 是，零 C 依赖 |
| musl 兼容 | ★★★★★ 是 |
| 同步/异步 | ★★★★★ 默认同步，可选异步 |
| read timeout | ★★★★★ `stream.set_read_timeout()` 原生支持 |
| API 简洁度 | ★★★★☆ `read_message()` / `write_message()` 两步 |
| 维护状态 | ★★★★★ 活跃（2026 年仍在更新） |

**示例（同步模式，带超时）：**

```rust
use tungstenite::{accept, Message};
use std::net::TcpListener;
use std::time::Duration;

let server = TcpListener::bind("0.0.0.0:9090")?;
server.set_nonblocking(true)?;  // accept 不阻塞

loop {
    // 尝试 accept
    if let Ok((stream, _)) = server.accept() {
        stream.set_read_timeout(Some(Duration::from_millis(50)))?;
        let mut ws = accept(stream)?;

        // 主循环：读 cmd_vel → 控制电机
        loop {
            match ws.read_message() {
                Ok(Message::Text(json)) => {
                    // 解析 {"v": 0.5, "omega": 0.0}
                    // kinematics::inverse() → motor::set_velocity()
                }
                Err(tungstenite::Error::Io(ref e))
                    if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // 超时，继续下一轮（维持 20Hz 心跳）
                    continue;
                }
                Err(_) => break, // 连接断开
                _ => {}
            }
        }
    }
    // 无连接时 sleep 一会
    std::thread::sleep(Duration::from_millis(10));
}
```

### tokio-tungstenite

| 维度 | 评价 |
|------|------|
| 依赖数 | ★★☆☆☆ 引入整个 tokio 生态（~30+ crates） |
| 复杂度 | ★★☆☆☆ 需要 async/await + tokio runtime |
| 线程模型 | ★★★☆☆ 需要 mpsc channel 桥接到同步 CAN 线程 |
| musl | ★★★★☆ tokio 支持 musl，但首次编译慢 |

**不推荐原因：** 当前代码是同步的。引入 tokio 意味着需要 mpsc channel 在线程间传递 cmd_vel，且 tokio runtime + CAN 线程的协调增加了不必要的复杂度。除非未来整个项目迁移到 async，否则不值得。

### 其他库

| 库 | 问题 |
|----|------|
| `websocket` (2016) | 已归档，无人维护 |
| `ewebsock` | 下载量低，文档少 |
| `warp` / `actix-web` | Web 框架级，太重 |
| `axum` + ws | 同上，需要 tokio |

---

## 三、结论：tungstenite

唯一的选择。纯 Rust 同步模式，7 个依赖，零 C binding，musl 原生兼容。`set_read_timeout(50ms)` 天然支持 20 Hz 控制循环 — 读不到消息就跳过，不影响主循环节奏。

Cargo.toml 一行：

```toml
tungstenite = "0.26"
```

---

## 四、架构设计

### 线程模型

```
┌────────────────────────────────────────────┐
│                  main 线程                   │
│                                            │
│  ┌──────────────────────────────────────┐  │
│  │  1. CAN 初始化（can_setup + NMT）    │  │
│  │  2. 电机使能（CiA 402 state machine）│  │
│  │  3. 初始化 DiffDrive                 │  │
│  └──────────────────────────────────────┘  │
│                   ↓                        │
│  ┌──────────────────────────────────────┐  │
│  │         主控制循环 (50ms = 20Hz)      │  │
│  │                                      │  │
│  │  ws.read_message() ← 50ms timeout    │  │
│  │       ↓ (如果有新消息)               │  │
│  │  解析 JSON {"v","omega"}             │  │
│  │       ↓                              │  │
│  │  DiffDrive::inverse(v, omega)        │  │
│  │       ↓                              │  │
│  │  motor.set_velocity(L, l_rpm)        │  │
│  │  motor.set_velocity(R, r_rpm)        │  │
│  │       ↓                              │  │
│  │  周期性 odom 上报（可选）            │  │
│  │  ws.write_message(odom_json)         │  │
│  └──────────────────────────────────────┘  │
└────────────────────────────────────────────┘
```

单线程，无 channel，无锁。好处：
- 调试简单（一个线程的事）
- WebSocket 连接断开自动重连（外层 loop）
- 50ms 读超时保证即使没有 cmd_vel 消息，循环也能持续 tick

### JSON 协议

**输入（cmd_vel，上位机 → AMR）：**
```json
{"v": 0.5, "omega": 0.0}
```

**输出（odom，AMR → 上位机，周期性）：**
```json
{"v": 0.48, "omega": 0.01, "left_rpm": 542, "right_rpm": -538, "status": "enabled"}
```

---

## 五、需要改动的文件

| 文件 | 改动 |
|------|------|
| `Cargo.toml` | 添加 `tungstenite = "0.26"` |
| `src/main.rs` | 全量实现：WebSocket 服务 + 控制循环 |
| `src/kinematics.rs` | 不需改（已就绪） |
| `src/motor.rs` | 不需改（已就绪） |

---

## 六、确认问题

1. **WebSocket 端口？** 建议 `9090`（AMR 端），和 MQTT 的 1883 区分
2. **是否需要同时保留 MQTT odom 上报？** 当前公司架构是 MQTT `/amr/status/{id}`，可以在 WebSocket 外额外发 MQTT（需加 `rumqttc` 依赖，但那是可选增强）
3. **JSON 字段名：`{"v": ..., "omega": ...}` 还是 ROS 标准 `{"linear": {"x": ...}, "angular": {"z": ...}}`？** 建议用简版，前端/数字孪生更好解析

---

## 七、风险

| 风险 | 缓解 |
|------|------|
| tungstenite 在 musl 下 TLS（wss://）不可用 | 使用 ws://（明文），局域网控制不需要加密 |
| WebSocket 连接断开期间丢失 cmd_vel | 保持最后一次速度指令；超时 N 秒后自动停车 |
| 单线程阻塞在 CAN SDO 导致 ws 读超时 | SDO 操作 <5ms，远小于 50ms 周期，不影响 |
