# HLS_canopen CLI 使用说明

## 启动流程（自动）

程序启动后自动完成初始化，无需手动操作：

```
[init] Configuring CAN interface...     → ip link set can0 type can bitrate 1000000
[init] Opening CAN socket...            → 打开 socketcan 阻塞模式
[init] NMT start left motor             → NMT 启动左轮节点 (0x01)
[init] NMT start right motor            → NMT 启动右轮节点 (0x02)
[init] Setting PV mode...               → 0x6060 ← 3 (PV 速度模式)
[init] Setting acceleration...          → 0x6083/0x6084 ← 1000 rpm/s
[init] Setting initial velocity to 0... → 0x60FF ← 0
[init] Enabling motors...               → CiA 402 状态机: 0x06→0x07→0x0F
```

启动完成后进入 REPL，**电机已处于使能状态，速度=0，可以直接发运动指令**。

## 命令详解

### 运动控制（使能后才有效）

```
go <rpm>        直行：两轮同速，正=前进，负=后退
                例: go 500     → 两轮 500 r/min 前进
                例: go -300    → 两轮 300 r/min 后退

left <rpm>      原地左转：左轮=0，右轮=<rpm>
                例: left 300   → 右轮 300 r/min，左轮不动

right <rpm>     原地右转：右轮=0，左轮=<rpm>
                例: right 300  → 左轮 300 r/min，右轮不动

back <rpm>      后退：等价于 go -<rpm>
                例: back 500   → 两轮 500 r/min 后退

stop            停止：两轮速度设 0（不断使能）
                停后仍可继续 go/left/right，无需重新 enable
```

### 状态控制

```
enable          使能两轮电机（CiA 402 状态机）
                - 只在 disable 之后才需要手动 enable
                - 启动时已自动 enable，平时不用管

disable         失能两轮电机（控制字写 0x0000）
                - 失能后 go/left/right 等运动命令无效
                - 恢复用 enable 重新走状态机
```

### 查询

```
status          显示当前状态
                输出示例:
                  Left:   234 r/min, status=0x0237 (enabled)
                  Right:  234 r/min, status=0x0237 (enabled)

help / ?        显示帮助
quit / exit / q 退出（电机保持当前状态，不断电不失能）
```

## 典型操作流程

### 场景 1：直行 → 停止 → 直行

```
hls-canopen> go 300        ← 直接发，已使能
  Both wheels: 300 r/min
hls-canopen> stop
  Motors stopped
hls-canopen> go 500        ← 停止后可以直接再走
  Both wheels: 500 r/min
```

### 场景 2：失能 → 使能 → 运动

```
hls-canopen> disable
  Motors disabled
hls-canopen> go 300
  ERROR: SDO error: node=0x01, ...   ← 失能后不能运动
hls-canopen> enable
  Motors enabled OK
hls-canopen> go 300                   ← 重新使能后才能动
  Both wheels: 300 r/min
```

### 场景 3：组合运动

```
hls-canopen> go 200
  Both wheels: 200 r/min
hls-canopen> left 300
  Left turn at 300 r/min
hls-canopen> stop
  Motors stopped
hls-canopen> back 200
  Backward at 200 r/min
hls-canopen> stop
  Motors stopped
```

### 场景 4：查看状态

```
hls-canopen> go 500
  Both wheels: 500 r/min
hls-canopen> status
  Left:  498 r/min, status=0x0237 (enabled)
  Right: 501 r/min, status=0x0237 (enabled)
```

## 状态机说明

```
         +--------+
启动时    │ 失能   │ ← disable 命令回到这里
自动走    +--------+
  ↓         ↑ enable 命令
+--------+  │
│ 使能   │──┘
+--------+
  ↓ go/left/right/back 任意时刻可用
+--------+
│ 运动中  │ stop 回到速度=0，仍在使能状态
+--------+
```

- **使能状态** = 电机通电、可以响应速度指令
- **stop** 只把速度设 0，不改变使能状态
- **disable** 失能断电，之后必须 enable 才能再动
- **退出程序** 不会自动失能，电机保持当前状态

## 硬件要求

- USB-CAN 适配器 + Ubuntu 22+ (amd64 或 arm64)
- `iproute2` 已安装（Ubuntu 22+ 默认有）
- 可能需要 `sudo` 或 `CAP_NET_ADMIN` 权限来配置 CAN 接口
- DS20270DA 驱动器需要在 CANopen 模式，节点 ID 左=1 右=2

## 运行

```bash
# Debug 模式
cargo run

# Release 模式（arm64 开发板上推荐）
cargo build --release
./target/release/hls_canopen

# 需要权限时
sudo cargo run
# 或
sudo setcap cap_net_admin+ep target/debug/hls_canopen
./target/debug/hls_canopen
```
