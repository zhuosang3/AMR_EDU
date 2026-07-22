# EXP2 无人小车导航系统 — 完整工作原理

> 机器人: Orange Pi 5 Plus / Armbian Noble / ROS2 Jazzy  
> 底盘: 前驱差速（前轴驱动 + 后万向轮），525mm×600mm×200mm，15kg  
> 文档基于实际部署代码撰写，涵盖从传感器到执行器的完整链路

---

## 如何使用本文档

这份文档是为**学习**而写的，不是速查手册。每个章节都解释"为什么这么设计"，而不只是"参数设了什么"。

**阅读建议**：
- 第一遍：按 5 天学习路径走，每天搞懂一个模块
- 第二遍：对照你本地的源文件（`D:\projects\expressone\`），边读文档边看源码
- 第三遍：尝试改一个参数，在脑子里推演小车行为会怎么变化

下面每个问题都值得你在看到对应代码时停下来想一想，想通了就是真正的理解。

---

## 5 天学习路径

### 第 1 天：理解系统骨架

> **目标**：搞清楚这套导航系统由哪些模块组成，数据怎么在模块间流动。

**核心文件**：`config/nav2_params.yaml`

**阅读方法**：
从头到尾扫一遍这个 YAML 文件，不要纠结每个参数的具体数值（那是第 3 天的任务），而是关注**有哪些模块**（以顶层 key 为标志）：

| 模块 | YAML key | 一句话职责 |
|------|---------|-----------|
| 定位 | `amcl` | 用激光+地图回答"我在哪" |
| 全局规划器 | `planner_server` | 在地图上找到从 A 到 B 的路线 |
| 路径平滑 | `smoother_server` | 把折线变成小车能跑的曲线 |
| 控制器 | `controller_server` | 沿着路线走，方向盘油门怎么打 |
| 全局代价地图 | `global_costmap` | 整张地图的障碍物视图 |
| 局部代价地图 | `local_costmap` | 车身周围 8m×8m 的近景 |
| 行为树 | `bt_navigator` | "卡住了怎么办"的决策逻辑 |
| 行为服务器 | `behavior_server` | 旋转/后退/等待等具体动作 |
| 速度平滑 | `velocity_smoother` | 急加速急减速的缓冲器 |
| 碰撞监视 | `collision_monitor` | 最后一道安全闸门 |
| 充电对接 | `docking_server` | 自动回充 |
| 重定位 | `reloc_manager` | 开机时的 360° 旋转定位 |
| 生命周期管理 | `lifecycle_manager_*` | 确保所有节点正常激活 |

**停下来想一想**：
1. planner 和 controller 的分工是什么？为什么不能合并成一个模块？
2. 为什么需要两张代价地图（global + local），一张不够吗？
3. velocity_smoother 和 collision_monitor 都在 `/cmd_vel` 链路上，谁先谁后？为什么 collision_monitor 必须在最后？

**完成后检验**：合上文件，默画出从 `/scan` 到 `/cmd_vel` 的数据流经哪些模块。

---

### 第 2 天：理解核心算法 — AMCL 定位 + 重定位策略

> **目标**：搞懂小车怎么从"我不知道自己在哪"变成"我知道自己在 (1.23, 4.56)，朝向 45°"。

**核心文件**：`scripts/reloc_manager.py`（不到 200 行）

**阅读方法**：逐行读这个 Python 文件，配合文档第 5、6 章。关注以下问题：

**reloc_manager.py 的关键函数调用链**：
```
__init__()              → 创建发布器/订阅器/定时器
_initial_pose_cb()      → 用户点 RViz 后触发
  publish_initial_pose() → 大协方差注入 AMCL
  _enter_localizing()    → 状态切换 + 启动旋转
    _start_rotation()    → 0.2 rad/s 逆时针
    _rotation_step()     → 每 50ms 检测：转够 360° 没？超时没？
_amcl_pose_cb()          → 每收到 /amcl_pose 检查协方差
                           cov[0]<0.1 且 cov[7]<0.1 → CONVERGED
_timer_cb()              → 每秒检查：超时 60s 没？
```

**停下来想一想**：
1. `publish_initial_pose` 里的协方差 `(1.0, 1.0, 0.5)` 意味着什么？如果改成 `(0.01, 0.01, 0.001)` 小车会怎样？
2. 为什么旋转是基于**里程计累积角度**而不是简单的计时 `sleep(31)`？
3. 为什么旋转过程中**不中断**，即使已经判定 CONVERGED 也让转完？如果提前停转会有什么风险？
4. 如果小车在一个完全对称的十字路口中心，360° 旋转能看到四个方向相同的走廊，粒子能收敛吗？（提示：想想 likelihood_field 是否只在局部做匹配）
5. `DEBOUNCE_SEC = 0.5` 是干什么的？去掉它会怎样？

**完成后检验**：用你自己的话解释——用户在 RViz 点 2D Pose Estimate 之后，到 "定位完成" 之间，香橙派内部发生了哪几件事？

---

### 第 3 天：理解参数调优 — 实战踩坑记录

> **目标**：学会从"现象"倒推"根因"，然后知道改哪个参数。

**核心文件**：`config/改动清单.md`（约 30KB，记录了从 2026-05-29 到 2026-06-17 的每一次参数修改）

**阅读方法**：**倒着读**——最新改动在最上面。每条记录都包含"现象→原因→修改"三要素：

**必读案例 1**：AMCL 定位漂移修复
```
现象: 定位标志在地图上到处乱飘
原因: z_hit=0.5 → 只有 50% 的激光点被当成"测到了墙"
      z_rand=0.5 → 另外 50% 被当成纯随机噪声
      → 两个正确位姿的粒子可能因为某些激光线"恰好是噪声"而被误杀
修改: z_hit=0.95, z_rand=0.05
      z_hit + z_rand ≈ 1.0 是标准约束
```

**必读案例 2**：local_costmap 的 `global_frame` 在 odom 和 map 之间切换
```
初始用 odom → 里程计漂移, 代价地图跟着歪
切到 map    → AMCL 收敛时 map→odom 跳变, 障碍物瞬移
EKF 上线后  → odom 又平滑了, 切回来
核心权衡: 平滑性(odom) vs 绝对准确性(map)
```

**必读案例 3**：碰撞多边形缩小
```
v1: 多边形比底盘大很多 → 窄通道不断误触发急停
v2: base_link 原点前移 215mm → 多边形偏移
v3: 精确量测底盘, 前留 200mm 后留 50mm
原则: 太小=碰撞, 太大=不敢走, 这是反复试出来的
```

**停下来想一想**：
1. 如果小车在一个堆满纸箱的仓库里，哪些参数可能需要改？为什么？
2. `inflation_radius: 0.1` 如果改成 `1.0` 会发生什么？
3. `cost_scaling_factor: 2.5` 控制什么？改大改小分别什么效果？

**完成后检验**：给你一个故障现象——"小车在走廊走到一半突然停下，不断原地旋转但就是不继续走"，你能列出至少 3 个可能的原因和对应的排查方向吗？

---

### 第 4 天：理解 EKF 传感器融合

> **目标**：搞懂里程计和 IMU 的数据是怎么组合的，以及"组合什么不组合什么"的判断标准。

**核心文件**：`config/ekf.yaml`

**关键概念**：EKF 融合的不是"所有数据"，而是"信得过的数据"。

```
筛选逻辑：

里程计提供:
  速度 vx, vyaw  → ✅ 信！瞬时测量，不漂移
  位置 x, y, yaw → ❌ 不信！速度积分出来的，漂移严重

IMU 提供:
  陀螺仪 vyaw  → ✅ 信！瞬时测量，比里程计转向更准
  磁力计 yaw   → ❌ 不信！电机磁场干扰，室内乱跳
  加速度 ax,ay → ❌ 不信！双重积分位姿 = 噪声爆炸
```

**阅读方法**：对照 `odom0_config` 和 `imu0_config` 的 15 个 bool 值，每一个 `true/false` 都问"为什么"。

**停下来想一想**：
1. 如果 IMU 安装歪了 5°，EKF 还能正常工作吗？会有什么后果？
2. `process_noise_covariance` 第 7 行（vx 的过程噪声 = 0.025）如果改大到 0.25，意味着什么？
3. 为什么 `world_frame: odom` 而不是 `map`？如果改成 `map` 会发生什么？
4. `two_d_mode: true` 做了什么？关掉会怎样？

**完成后检验**：画出 EKF 的预测-更新循环。标出每一步的输入输出。

---

### 第 5 天：串联全局 — Launch 文件 + 完整时序

> **目标**：能看着 launch 文件说出启动后每一秒发生了什么。

**核心文件**：`launch/navigation.launch.py`、`launch/mapping.launch.py`

**阅读方法**：

1. 先看 `navigation.launch.py` 的三个组件：
   - `localization_launch` — 从 nav2_bringup 引入，做什么？
   - `navigation_launch` — 从 nav2_bringup 引入，做什么？
   - `reloc_manager` — 自定义节点，为什么不在 nav2_bringup 里？

2. 对比 `mapping.launch.py` 和 `navigation.launch.py`：
   - mapping 多了什么？（slam_toolbox + map_saver）
   - mapping 少了什么？（localization_launch — 为什么？）

3. 理解 `lifecycle_manager` 的角色：Nav2 的节点是"有生命周期的"（unconfigured→inactive→active），lifecycle_manager 负责自动把所有节点拉到 active 状态。

**停下来想一想**：
1. 为什么 `localization_launch` 和 `navigation_launch` 用 `IncludeLaunchDescription` 而不是直接在 launch 文件里写 Node？
2. `GroupAction` 包了三个 launch 是什么作用？
3. `use_composition: "False"` 是什么？设成 True 会怎样？
4. 如果我想在导航启动时自动加载一张不同的地图，改哪里？

**完成后检验**：自己写一个最小化的 launch 文件，只启动 AMCL + map_server（不加导航栈），然后想象一下启动后的话题列表会是什么样。

---

## 重点概念速查

这些概念是理解整套系统的钥匙，按重要程度排列：

| # | 概念 | 对应章节 | 一句话 |
|---|------|---------|--------|
| 1 | AMCL 粒子滤波 | 5 | 用 5000 个"猜我在哪"的粒子，激光匹配淘汰猜错的 |
| 2 | TF 树 (map→odom→base_link) | 3 | 三层坐标系，每层谁发布、谁消费 |
| 3 | 协方差与收敛 | 5, 6 | 协方差大=不确定，小=确定；reloc_manager 盯着它判断定位成功 |
| 4 | EKF 融合策略 | 4 | 信速度、不信积分量；信陀螺仪、不信磁力计 |
| 5 | Hybrid-A\* 规划 | 8 | A\* 的升级版，考虑最小转弯半径 |
| 6 | Pure Pursuit 控制 | 9 | 朝前视点画圆弧，前驱差速要降速防甩尾 |
| 7 | 代价地图双层设计 | 7 | global 负责路线规划，local 负责避障缓刹 |
| 8 | Likelihood Field | 5 | 比 beam model 更平滑的激光匹配模型 |
| 9 | 回环检测 | 12 | SLAM 发现"来过这里"→全局优化修正漂移 |
| 10 | Lifecycle 节点管理 | 13 | Nav2 节点需要按顺序激活，manager 自动管 |

---

## 可以略读的部分

以下内容对学习导航原理帮助不大，知道存在即可：

- `rviz/exp2.rviz` — RViz 的可视化布局配置，纯显示层面
- `CMakeLists.txt` — 只做文件安装，没有编译逻辑
- `package.xml` — 依赖声明，`<depend>nav2_bringup</depend>` 一行就够了
- `savemap.zsh` — 一行 `ros2 service call` 封装的保存地图脚本
- `reloc_status.zsh` — 一行 `ros2 topic echo` 看状态

---

## 目录

1. [系统架构总览](#1-系统架构总览)
2. [硬件与传感器层](#2-硬件与传感器层)
3. [TF 坐标变换树](#3-tf-坐标变换树)
4. [传感器融合 — EKF](#4-传感器融合--ekf)
5. [定位 — AMCL](#5-定位--amcl)
6. [重定位管理器 — 360° 旋转策略](#6-重定位管理器--360-旋转策略)
7. [代价地图](#7-代价地图)
8. [路径规划 — SmacPlanner2D](#8-路径规划--smacplanner2d)
9. [路径跟踪 — Regulated Pure Pursuit](#9-路径跟踪--regulated-pure-pursuit)
10. [行为树与恢复机制](#10-行为树与恢复机制)
11. [速度平滑与碰撞监视](#11-速度平滑与碰撞监视)
12. [建图 — SLAM Toolbox](#12-建图--slam-toolbox)
13. [完整启动流程](#13-完整启动流程)
14. [关键参数调优历史](#14-关键参数调优历史)

---

## 1. 系统架构总览

```
┌──────────────────────────────────────────────────────────────────┐
│                      硬件驱动层（PM2 常驻）                         │
│                                                                  │
│  imu_driver (串口)  hls_canopen (CAN)  keli_lidar (UDP:2112)    │
│       │                   │                   │                   │
│   WS:8765             WS:9090               /scan (直接ROS2)      │
│       │                   │                                      │
│       ▼                   ▼                                      │
│  imu_bridge         canopen_ros2                                 │
│       │                   │                                      │
│     /imu            /odom + /tf(odom→base_link)                   │
│                                                                  │
│  exp2_description ──→ /tf_static (URDF 静态变换)                  │
│       │                                                          │
│  base_link→laser, base_link→imu_link, ...                        │
└──────────────────────┬───────────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│                     融合层（ros2 launch 启动）                     │
│                                                                  │
│  ┌─────────────────────────────────────┐                         │
│  │  EKF (robot_localization)           │                         │
│  │  ───────────────────────────────    │                         │
│  │  输入: /odom (vx, vyaw)             │                         │
│  │        /imu  (vyaw only, 陀螺仪)     │                         │
│  │  输出: /odometry/filtered            │                         │
│  │        TF odom→base_link            │                         │
│  └─────────────────────────────────────┘                         │
│                                                                  │
│  ┌─────────────────────────────────────┐                         │
│  │  AMCL (自适应蒙特卡洛定位)             │                         │
│  │  ───────────────────────────────    │                         │
│  │  输入: /scan (激光雷达)              │                         │
│  │        /odometry/filtered (运动更新)  │                         │
│  │        预建地图 (map_server)         │                         │
│  │  输出: TF map→odom                  │                         │
│  │        /amcl_pose (协方差+位姿)       │                         │
│  └─────────────────────────────────────┘                         │
│                                                                  │
│  ┌─────────────────────────────────────┐                         │
│  │  reloc_manager (自定义重定位管理器)    │                         │
│  │  ───────────────────────────────    │                         │
│  │  拦截 /initialpose → 大协方差注入    │                         │
│  │  启动 360° 主动旋转                  │                         │
│  │  监控 /amcl_pose 协方差 → 收敛判定    │                         │
│  └─────────────────────────────────────┘                         │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│                     导航决策层（Nav2 栈）                          │
│                                                                  │
│  ┌──────────┐   ┌────────────┐   ┌───────────────┐              │
│  │ Planner  │──▶│ Smoother   │──▶│ Controller    │              │
│  │ Server   │   │ Server     │   │ Server        │              │
│  │          │   │            │   │               │              │
│  │ Smac2D   │   │ Simple     │   │ Regulated     │              │
│  │ Hybrid-A*│   │ Smoother   │   │ Pure Pursuit  │              │
│  └──────────┘   └────────────┘   └───────┬───────┘              │
│                                          │                       │
│  ┌──────────────────────────────────────┐│                       │
│  │ Costmaps                             ││                       │
│  │ ┌──────────────┐ ┌─────────────────┐ ││                       │
│  │ │ Global       │ │ Local           │ ││                       │
│  │ │ frame: map   │ │ frame: odom     │ ││                       │
│  │ │ static+obstacle │ voxel+inflation│ ││                       │
│  │ └──────────────┘ └─────────────────┘ ││                       │
│  └──────────────────────────────────────┘│                       │
│                                          ▼                       │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ Velocity Smoother → Collision Monitor → /cmd_vel → 电机     │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

### 分层设计原则

硬件驱动层**不依赖 ROS**（纯 Rust cargo build），通过 WebSocket 与 ROS2 桥接层通信。ROS2 桥接层负责将驱动数据转换到标准 ROS2 话题。驱动可脱离 ROS 独立测试。

---

## 2. 硬件与传感器层

### 2.1 科力 LSE-2027DE 激光雷达

| 项目 | 参数 |
|------|------|
| 安装位置 | x=+298mm, z=+200mm（车头正前方） |
| 通信 | UDP 192.168.8.11:2112，专属二进制协议 |
| 测距 | 0.05m ~ 10m，360° 扫描，~10Hz |
| ROS 话题 | `/scan` (sensor_msgs/LaserScan) |
| 节点 | `keli_lidar`（纯 Rust，直接发布 ROS2 话题） |

### 2.2 JY901 IMU（WitMotion 10 轴）

| 项目 | 参数 |
|------|------|
| 安装位置 | x=+290mm, z=+200mm |
| 通信链路 | 串口 → `imu_driver`(Rust) → WS:8765 → `imu_bridge`(r2r) → `/imu` |
| ROS 话题 | `/imu` (sensor_msgs/Imu, ~100Hz) |
| EKF 使用 | **仅用陀螺仪 yaw 角速度**，磁力计室内不可信，加速度不双重积分 |

### 2.3 CANopen 电机系统

| 项目 | 参数 |
|------|------|
| CAN 接口 | can0, 1Mbps |
| 通信链路 | CAN bus → `hls_canopen`(Rust) → WS:9090 → `canopen_ros2` → `/odom`, `/tf` |
| 里程计原理 | 差分驱动航迹推算（0x6064 位置差分），~20Hz |
| 速度命令 | 订阅 `/cmd_vel`，通过 CANopen 下发到电机驱动器 |

### 2.4 机器人运动学参数

```
驱动类型: 前驱差速（前轴 2 驱动轮 + 后 2 万向轮）
驱动轮距: 450mm (y=±225mm)
驱动轮前后位置: x=+215mm（前轴）
轮半径: 84.5mm
Nav2 机器人半径: 0.35m（代价地图碰撞检测用）
原点/base_link: 底盘几何中心
坐标约定: x→前, y→左, z→上
```

---

## 3. TF 坐标变换树

### 3.1 完整 TF 链

```
map ──(AMCL 动态广播)──▶ odom ──(EKF 动态广播)──▶ base_link
                                                     │
                                      ┌──────────────┼──────────────┐
                                      │              │              │
                                   laser         imu_link        (其他部件)
                              (URDF 静态变换)  (URDF 静态变换)
```

### 3.2 各帧的职责

| 帧 | 发布者 | 类型 | 含义 |
|-----|--------|------|------|
| `map` | map_server + AMCL | 世界原点 | 地图坐标系，绝对参考，全局唯一 |
| `odom` | EKF (robot_localization) | 里程计原点 | 里程计累积坐标系，平滑连续但会漂移 |
| `base_link` | EKF | 机器人本体 | 固定在底盘几何中心，x 前 y 左 z 上 |
| `laser` | robot_state_publisher | 传感器 | 激光雷达安装位姿（x=+298, z=+200） |
| `imu_link` | robot_state_publisher | 传感器 | IMU 安装位姿（x=+290, z=+200） |

### 3.3 map → odom 变换的物理意义

这是整个定位系统最核心的概念：

```
map → odom = AMCL 估计的"里程计原点在地图上的位姿"

当 AMCL 确定机器人在 map 中的位姿 P_map，且 EKF 告诉机器人
在 odom 中的位姿 P_odom 时：
  Transform(map→odom) = P_map - P_odom

这个变换不是直接从传感器量测的，而是 AMCL 通过激光匹配
推算出来的"修正量"——它补偿了里程计的长期漂移。
```

### 3.4 为什么局部代价地图用 odom 帧而不是 map 帧

AMCL 收敛过程中 `map→odom` 变换可能有跳变（粒子群突然收敛到新位置）。如果局部代价地图用 `map` 帧，障碍物位置会跟着跳。用 `odom` 帧——EKF 的输出是平滑的——代价地图稳定，不受 AMCL 收敛抖动影响。

---

## 4. 传感器融合 — EKF

### 4.1 算法：扩展卡尔曼滤波（robot_localization 包）

EKF 是贝叶斯滤波器的一种，核心思想是**预测-更新**循环：

```
预测步骤:
  x̂ₖ⁻ = f(x̂ₖ₋₁, uₖ)          ← 用里程计速度推算下一时刻状态
  Pₖ⁻ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ      ← 协方差传播（不确定性增大）

更新步骤:
  Kₖ = Pₖ⁻ Hₖᵀ (Hₖ Pₖ⁻ Hₖᵀ + Rₖ)⁻¹   ← 卡尔曼增益
  x̂ₖ = x̂ₖ⁻ + Kₖ (zₖ - h(x̂ₖ⁻))        ← 用传感器测量修正状态
  Pₖ = (I - Kₖ Hₖ) Pₖ⁻                 ← 协方差收缩（不确定性减小）
```

### 4.2 状态向量（15 维，2D 模式）

```
状态 x = [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
               忽略（2D 模式）              ▲              ▲
                                           │              │
                                    里程计提供线速度    IMU 提供角速度
```

### 4.3 传感器配置矩阵

**轮式里程计 (`/odom`)**：
```
odom0_config:
  位置 [x, y, z]:            [false, false, false]  ← 不融合！积分漂移太大
  姿态 [roll, pitch, yaw]:    [false, false, false]  ← 不融合！
  线速度 [vx, vy, vz]:        [true,  false, false]  ← ✅ 只信 vx（前向速度）
  角速度 [vroll, vpitch, vyaw]: [false, false, true]  ← ✅ 只信 vyaw（转向角速度）
  线加速度 [ax, ay, az]:      [false, false, false]  ← 轮式里程计不提供
```

**IMU (`/imu`)**：
```
imu0_config:
  姿态 [roll, pitch, yaw]:    [false, false, false]  ← ❌ 磁力计在室内/电机旁不可信
  线速度:                      [false, false, false]  ← IMU 不提供速度
  角速度 [vroll, vpitch, vyaw]: [false, false, true]  ← ✅ 只信 vyaw（陀螺仪 z 轴）
  线加速度:                    [false, false, false]  ← 轮式机器人不积分加速度
```

### 4.4 设计思想

**只融合"测量量"（速度），不融合"积分量"（位姿）**。

- 里程计的速度测量是瞬时的、可信的 → 融合
- 里程计通过速度积分得到的位姿，长时间会漂移 → 不融合，留给 AMCL 用激光匹配来修正
- IMU 陀螺仪的角速度测量是瞬时的、可信的 → 融合
- IMU 的磁力计在室内受电机磁场干扰严重 → 不融合
- 加速度双重积分 → 位置，噪声指数放大 → 轮式机器人不这样做

### 4.5 过程噪声协方差

```
process_noise_covariance (15×15 对角阵):
  [x, y, z]:      0.05, 0.05, 0.06    ← 位置过程噪声
  [roll,pitch,yaw]: 0.03, 0.03, 0.06   ← 姿态过程噪声
  [vx, vy, vz]:   0.025, 0.025, 0.04   ← 线速度过程噪声（较小=更信任运动模型）
  [ωx, ωy, ωz]:   0.01, 0.01, 0.02     ← 角速度过程噪声
  [ax, ay, az]:   0.01, 0.01, 0.015    ← 线加速度过程噪声
```

---

## 5. 定位 — AMCL

### 5.1 算法原理：自适应蒙特卡洛定位

AMCL（Adaptive Monte Carlo Localization）是粒子滤波在移动机器人定位中的应用。

```
┌─────────────────────────────────────────────────────────────────┐
│                      粒子滤波定位循环                             │
│                                                                 │
│  初始化                                                        │
│  ┌──────────────────────────────────────┐                      │
│  │  5000 个粒子散布在地图上               │                      │
│  │  每个粒子 = (x, y, yaw, weight)       │                      │
│  │  •  •     •  •                       │                      │
│  │    •   🤖   •  •   ← 大协方差→撒满全图 │                      │
│  │  •     •       •                      │                      │
│  └──────────────────────────────────────┘                      │
│         │                                                      │
│         ▼  收到 /odometry/filtered                              │
│  ┌──────────────────────────────────────┐                      │
│  │  运动更新 (Motion Update)             │                      │
│  │  ────────────────────────────         │                      │
│  │  对每个粒子:                          │                      │
│  │    x' = x + Δx + noise_x              │                      │
│  │    y' = y + Δy + noise_y              │                      │
│  │    yaw' = yaw + Δyaw + noise_yaw      │                      │
│  │                                       │                      │
│  │  噪声模拟里程计不确定性                 │                      │
│  │  alpha1~4 控制噪声大小                  │                      │
│  └──────────────────────────────────────┘                      │
│         │                                                      │
│         ▼  收到 /scan (激光雷达)                                 │
│  ┌──────────────────────────────────────┐                      │
│  │  测量更新 (Measurement Update)        │                      │
│  │  ────────────────────────────────     │                      │
│  │  对每个粒子:                          │                      │
│  │    1. 假设机器人在粒子位姿处            │                      │
│  │    2. 将每条激光线从粒子位姿投影到地图   │                      │
│  │    3. 在地图上查找该点是否命中障碍物     │                      │
│  │    4. 用 likelihood_field 模型计算     │                      │
│  │       该激光点命中的概率                │                      │
│  │    5. weight = Π 所有激光点的概率       │                      │
│  │                                       │                      │
│  │  高 weight → 该粒子位姿解释了激光数据    │                      │
│  │  低 weight → 该粒子位姿与激光矛盾       │                      │
│  └──────────────────────────────────────┘                      │
│         │                                                      │
│         ▼                                                      │
│  ┌──────────────────────────────────────┐                      │
│  │  重采样 (Resampling) + KLD 自适应      │                      │
│  │  ────────────────────────────────     │                      │
│  │  高权重粒子 → 复制（分裂）              │                      │
│  │  低权重粒子 → 淘汰                     │                      │
│  │                                       │                      │
│  │  KLD 采样: 根据粒子分布自动调节数量      │                      │
│  │    粒子散开时 → 保留更多（最多 5000）    │                      │
│  │    粒子集中时 → 减少（最少 500）        │                      │
│  │                                       │                      │
│  │  •••  •  •  🤖•  •   ← 粒子逐渐收敛    │                      │
│  └──────────────────────────────────────┘                      │
│         │                                                      │
│         ▼  循环                                                 │
│  收敛后: AMCL 发布 TF map→odom + /amcl_pose                     │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 两种激光模型对比

| 特性 | Beam Model | Likelihood Field |
|------|-----------|-----------------|
| 工作原理 | 沿射线追踪，逐点计算距离概率 | 查表法：预先计算地图每个栅格"到最近障碍物的距离" |
| 计算量 | 高（每条射线都做 raycast） | 低（O(1) 查表 + 高斯计算） |
| 非连续环境 | 尖锐概率分布，易发散 | 平滑的概率场，更鲁棒 |
| 无特征走廊 | 容易多模态（粒子分团） | 概率场平滑→粒子更易收敛 |
| 本车使用 | — | ✅ `laser_model_type: likelihood_field` |

**likelihood_field 如何工作**：

```
1. 预先计算: 对地图上每个格点，计算到最近障碍物的欧氏距离 d
   得到一张"距离场"（likelihood field）

2. 对每条激光线:
   - 将激光终点投影到地图坐标
   - 查表得到该坐标的 d
   - 概率 = z_hit × Gaussian(d, σ_hit) + z_rand × (1/z_max)

3. 关键参数:
   z_hit = 0.95   → 95% 的激光点是真实测到了障碍物
   z_rand = 0.05  → 5% 是随机噪声（防止因少数异常点导致正确粒子被淘汰）
   sigma_hit = 0.2 → 高斯标准差 20cm（容忍激光测量噪声和地图不精确）
```

### 5.3 关键参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `max_particles` | 5000 | 全局定位时用满，KLD 自适应降到 ~500 |
| `min_particles` | 500 | 跟踪模式最小粒子数 |
| `laser_model_type` | `likelihood_field` | 平滑概率场模型 |
| `max_beams` | 200 | 每帧最多用 200 条激光线（降计算量） |
| `update_min_d` | 0.25 | 平移 25cm 才做测量更新（防原地抖动） |
| `update_min_a` | 0.1 | 旋转 ~6° 才做测量更新 |
| `recovery_alpha_slow` | 0.01 | 长期未收敛时，以 1% 概率注入随机粒子 |
| `recovery_alpha_fast` | 0.1 | 短期未收敛时，以 10% 概率注入随机粒子 |
| `resample_interval` | 1 | 每帧都重采样 |
| `z_hit` | 0.95 | 95% 激光点视为真实障碍物测量（其余 5% 为随机） |
| `sigma_hit` | 0.2 | 测量噪声标准差 0.2m |
| `pf_err` | 0.05 | KLD 误差阈值 |
| `pf_z` | 0.99 | KLD 置信概率 |

---

## 6. 重定位管理器 — 360° 旋转策略

这是本系统最精妙的自定义组件（`scripts/reloc_manager.py`），不到 200 行，解决了 AMCL 在特征稀疏环境下的全局定位难题。

### 6.1 问题背景

标准的 AMCL 全局定位（粒子撒满全地图）有两个痛点：

1. **静止时信息不足**：激光只看到前方一小片区域，如果前方是走廊/对称结构，粒子团会产生多模态分布（多个可能的位姿都匹配当前激光数据）
2. **小范围移动收敛慢**：没有明确的旋转动作，粒子团需要积累足够的平移+旋转才能逐步收敛

### 6.2 reloc_manager 的解决方案

```
        用户在 RViz 点 2D Pose Estimate
                    │
                    ▼
    ┌───────────────────────────────────┐
    │  reloc_manager._initial_pose_cb() │
    │                                   │
    │  1. 发布 /initialpose              │
    │     协方差 = (1.0, 1.0, 0.5)      │
    │     含义: "我在 (~1m, ~1m) 范围内" │
    │     → AMCL 粒子撒满全图            │
    │                                   │
    │  2. 启动 360° 逆时针旋转            │
    │     cmd_vel.angular.z = 0.2        │
    │     基于里程计累积角度控制           │
    │     转满 360° 自动停止              │
    └───────────────┬───────────────────┘
                    │
        同时监控    │   每 50ms 发布一次旋转指令
                    │
    ┌───────────────▼───────────────────┐
    │  监控 /amcl_pose 协方差            │
    │                                   │
    │  cov[0] = 位置 x 方向不确定度       │
    │  cov[7] = 位置 y 方向不确定度       │
    │                                   │
    │  当 cov[0] < 0.1 且 cov[7] < 0.1  │
    │  → 状态变为 CONVERGED             │
    │  → 旋转允许自然完成（不停车打断）    │
    │                                   │
    │  超时 60s → 告警 "重新点 2D Pose"   │
    └───────────────────────────────────┘
```

### 6.3 三种重定位模式

| 模式 | 触发方式 | 协方差设置 | 适用场景 |
|------|---------|-----------|---------|
| **全局定位** | RViz 点 2D Pose → 自动 360° 旋转 | (1.0, 1.0, 0.5) | 日常开机，不知道自己在哪 |
| **充电桩标定** | `./reloc_dock.zsh` → `/reloc/dock_calib` | (0.01, 0.01, 0.001) | 精确知道自己在充电桩上 |
| **手动设点** | `./reloc_manual.zsh X Y YAW` | (1.0, 1.0, 0.5) | 已知精确位姿 |

### 6.4 为什么旋转能让粒子收敛

```
初始状态（静止，只看到前方墙壁）:
┌─────────────────────────────────────┐
│  走廊环境中，多个位置看到相似的墙壁    │
│                                     │
│  粒子团 A: "我在 (1, 2)，看到左墙"    │
│  粒子团 B: "我在 (5, 2)，看到左墙"    │  ← 多模态！激光数据无法区分
│  粒子团 C: "我在 (3, 4)，看到左墙"    │
└─────────────────────────────────────┘

旋转 180° 后（看到后方墙壁）:
┌─────────────────────────────────────┐
│  后方墙壁的独特特征区分了三种假设:     │
│                                     │
│  粒子团 A: 后方 2m 有墙壁 → 权重高    │  ← 匹配！粒子存活
│  粒子团 B: 后方 10m 无墙壁 → 权重低    │  ← 矛盾！粒子淘汰
│  粒子团 C: 后方 1m 有墙壁 → 权重低    │  ← 矛盾！粒子淘汰
└─────────────────────────────────────┘

旋转 360° 后:
  所有方向的障碍物信息都累积到粒子权重中
  → 只有与真实位姿一致的粒子存活
  → 粒子群迅速收敛到唯一正确位姿
```

### 6.5 旋转控制的工程细节

```python
# 不是简单发一个 cmd_vel 就完事，而是基于里程计精确闭环:

def _rotation_step(self):
    # 1. 里程计累积角度（而非简单计时）
    delta = norm_angle(current_odom_yaw - prev_odom_yaw)
    total_rotated += abs(delta)

    # 2. 转满 360° 自动停止
    if total_rotated >= swing_angle:  # 2π
        stop_rotation()

    # 3. 安全超时 (150% 预期时间)
    if elapsed > rotation_timeout:
        stop_rotation()

    # 4. 每 50ms (20Hz) 发一次指令，保证连续旋转
    twist.angular.z = 0.2  # 0.2 rad/s ≈ 31秒转完一圈
    cmd_vel_pub.publish(twist)
```

### 6.6 去抖动设计

```python
DEBOUNCE_SEC = 0.5  # 0.5 秒内重复点击 RViz 的 2D Pose 被忽略

if now - last_initial_pose_time < DEBOUNCE_SEC:
    return  # 跳过
```

---

## 7. 代价地图

Nav2 的代价地图（Costmap）是一个 2D 栅格，每个格点存储一个 `[0, 254]` 的代价值：

```
代价值含义:
  0        = 自由空间（FREE）
  1-252    = 膨胀区域（距离障碍物越近代价越高）
  253      = 致命障碍物（LETHAL，不可通过）
  254      = 未知空间（UNKNOWN，全局代价地图考虑，局部不进入）
  255      = 保留值（NO_INFORMATION）
```

### 7.1 全局代价地图 vs 局部代价地图

| 特性 | 全局代价地图 (global_costmap) | 局部代价地图 (local_costmap) |
|------|------------------------------|------------------------------|
| **坐标系** | `map` | `odom` |
| **尺寸** | 整张地图的大小 | 8m × 8m 滚动窗口 |
| **更新频率** | 1Hz | 5Hz |
| **窗口类型** | 固定窗口 | 滚动窗口（随机器人移动） |
| **障碍物层** | ObstacleLayer（2D 投影） | VoxelLayer（3D 体素） |
| **插件** | StaticLayer + ObstacleLayer + InflationLayer | VoxelLayer + InflationLayer |
| **用途** | 规划全局路径 | 避障 + 路径跟踪 |
| **未知空间** | 视为可通过（`track_unknown_space: true`） | 不进入未知区域 |

### 7.2 各层详解

**StaticLayer** — 从 map_server 加载预建地图，提供静态障碍物信息。

**ObstacleLayer** — 订阅 `/scan`，将激光点投射到地图上：
- Marking: 激光点位置标记为障碍物
- Clearing: 激光线穿过的区域标记为自由空间
- `obstacle_max_range: 2.5m` — 只标记 2.5m 内的障碍物（远处数据不可靠）
- `raytrace_max_range: 3.0m` — 只清除 3.0m 内的自由空间

**VoxelLayer**（仅局部代价地图）— 3D 体素层，可以在 z 轴区分地面和悬空物：
- `z_voxels: 16` — 纵向 16 个体素
- `max_obstacle_height: 2.0m` — 高于 2m 的物体不算障碍物（通过性考虑）

**InflationLayer** — 在障碍物周围生成膨胀区域，使路径规划避开障碍物：
- `inflation_radius: 0.1m` — 障碍物周围 10cm 开始产生代价
- `cost_scaling_factor: 2.5` — 代价衰减率

### 7.3 机器人半径与膨胀的配合

```
机器人物理半径: 0.35m
膨胀半径: 0.1m
────────────────────
有效安全半径: 0.35m + 0.1m 膨胀区域

实际避障效果:
  障碍物
    │
    ├── 0.1m 膨胀带（代价 253→1 渐变）  ← 规划器倾向于避开
    │
    ├── 0.35m 机器人半径               ← 碰撞边界
    │
    [机器人中心]
```

---

## 8. 路径规划 — SmacPlanner2D

### 8.1 算法：Hybrid-A\*

SmacPlanner2D 是 Nav2 中基于 Smac（State Lattice-based Motion Planning）框架的 2D 规划器，核心是 **Hybrid-A\*** 算法。

**Hybrid-A\* 与传统 A\* 的区别**：

| | A\* | Hybrid-A\* |
|------|-----|------------|
| 状态空间 | (x, y) 离散格点 | (x, y, θ) 连续位姿 + 离散角度 |
| 运动学约束 | 无（8 邻域移动） | 有（Reeds-Shepp 曲线） |
| 生成路径 | 折线（需后处理） | 平滑曲线（可直接跟踪） |
| 完备性 | 分辨率完备 | 分辨率完备 + 概率完备 |

### 8.2 搜索过程

```
┌──────────────────────────────────────────────────────────────┐
│  Hybrid-A* 搜索                                               │
│                                                              │
│  1. 代价函数: f(n) = g(n) + h(n)                             │
│     g(n) = 实际累积代价 = travel_cost × cost_multiplier      │
│     h(n) = 启发式代价 = 到目标的 ObstacleHeuristic 估计距离    │
│                                                              │
│  2. 节点扩展（Motion Primitives）:                             │
│     - 从当前 (x, y, θ)，应用 Reeds-Shepp 运动基元              │
│     - 角度离散化为 72 bins（每 5° 一个）                        │
│     - 每个运动基元生成一个新节点 (x', y', θ')                   │
│                                                              │
│  3. 解析扩展 (Analytic Expansion):                            │
│     - 每 3.5 个节点 → 尝试用 Reeds-Shepp 曲线直连目标           │
│     - 如果曲线无碰撞 → 路径找到！大幅加速搜索                    │
│                                                              │
│  4. 终止条件: 到达目标 tolerance=0.5m 内，或 max_iterations    │
└──────────────────────────────────────────────────────────────┘
```

### 8.3 Reeds-Shepp 运动模型

Reeds-Shepp 曲线允许**前进和倒车**，由以下基本动作组成：

```
基本动作:
  S (Straight)  = 直行
  C (Curve)     = 以最小转弯半径转向（左 L 或右 R）
  
倒车动作用 | 标记，例如:
  C|C|C  = 前进-倒车-前进 三段弧线
  CSC    = 弧线-直线-弧线

48 种组合 → 取最短的无碰撞曲线
```

对前驱差速小车的意义：允许倒车意味着可以从死胡同里退出来，不需要原地掉头空间。

### 8.4 关键参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `motion_model_for_search` | `REEDS_SHEPP` | 允许倒车的运动模型 |
| `angle_quantization_bins` | 72 | 360° / 72 = 5° 分辨率 |
| `analytic_expansion_ratio` | 3.5 | 每 3.5 个节点尝试直连目标 |
| `max_iterations` | 10000 | 最大搜索节点数 |
| `max_planning_time` | 8.0s | 规划超时 |
| `tolerance` | 0.5m | 到达目标 0.5m 内即认为找到路径 |
| `cost_travel_multiplier` | 2.0 | 运动代价 ×2（鼓励探索更优路径而非贪婪） |

### 8.5 路径平滑（Smoother Server）

规划器生成的路径虽然是 Reeds-Shepp 曲线，但可能不够平滑。Smoother Server 用**非线性优化**进一步平滑：

```yaml
simple_smoother:
  plugin: "nav2_smoother::SimpleSmoother"
  tolerance: 1.0e-10    # 收敛精度
  max_its: 1000         # 最大迭代
  do_refinement: true   # 开启细化
```

---

## 9. 路径跟踪 — Regulated Pure Pursuit

### 9.1 算法原理

Pure Pursuit 是一种几何路径跟踪算法。从机器人当前位置，沿参考路径向前找"前视点"（carrot），然后计算一个圆弧使机器人朝前视点运动。

```
                   参考路径
    ─────────────────────●───────
                        /  前视点 (carrot)
                       /
                      /  lookahead_dist
                     /
                    /
                   🤖 机器人当前位置
```

**Regulated** 的改进（相比标准 Pure Pursuit）：

1. **速度缩放前视距离**：速度快 → 看更远（避免急转弯）；速度慢 → 看更近（精确跟踪）
2. **代价地图调速**：靠近障碍物时自动降低速度
3. **接近目标时降速**：距目标 4m 内线性减速到 0.05 m/s
4. **转弯最小半径约束**：小半径转弯时强制降速，防止翻车/甩尾

### 9.2 前视距离的自适应调节

```python
# 速度关联前视距离:
lookahead = max(
    min_lookahead_dist,       # 0.4m
    min(max_lookahead_dist,   # 1.0m
        current_speed * lookahead_time  # v × 1.5s
    )
)
# 低速 (0.1m/s): 前视 = max(0.4, min(1.0, 0.15)) = 0.4m
# 高速 (1.0m/s): 前视 = max(0.4, min(1.0, 1.5)) = 1.0m
```

### 9.3 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `desired_linear_vel` | 1.0 m/s | 巡航速度 |
| `lookahead_dist` | 0.6m | 标称前视距离 |
| `min_lookahead_dist` | 0.4m | 最小前视 |
| `max_lookahead_dist` | 1.0m | 最大前视 |
| `lookahead_time` | 1.5s | 速度→前视的时间常数 |
| `rotate_to_heading_angular_vel` | 0.3 rad/s | 原地转向速度（前驱降级，防甩尾） |
| `max_angular_accel` | 1.0 rad/s² | 角加速度上限 |
| `max_linear_accel` | 1.0 m/s² | 线加速度上限 |
| `approach_velocity_scaling_dist` | 4.0m | 距目标多远开始降速 |
| `min_approach_linear_velocity` | 0.05 m/s | 最低接近速度 |
| `xy_goal_tolerance` | 0.03m | 位置到达容忍度 |
| `yaw_goal_tolerance` | 0.05rad | 朝向到达容忍度 |
| `use_collision_detection` | true | 开启碰撞检测 |

---

## 10. 行为树与恢复机制

### 10.1 行为树结构

本系统使用 Nav2 内置的行为树 `navigate_w_replanning_only_if_path_becomes_invalid.xml`：

```
NavigateToPose
  ├── 检查是否到达目标
  │   └── 距离+朝向都在容忍度内 → 成功
  │
  ├── 计算全局路径 (ComputePathToPose)
  │   └── SmacPlanner2D → 失败则重试
  │
  └── 跟踪路径 (FollowPath)
      ├── RegulatedPurePursuit 正常跟踪
      │
      ├── 路径阻塞？→ 重规划 (Replan)
      │
      └── 卡住了？→ 恢复行为序列:
          ├── Spin (原地旋转 360°，清除周围障碍物)
          ├── BackUp (后退一段距离)
          └── Wait (等待一会)
```

### 10.2 可用恢复行为

| 行为 | 插件 | 作用 |
|------|------|------|
| `spin` | `nav2_behaviors::Spin` | 原地旋转，更新代价地图 |
| `backup` | `nav2_behaviors::BackUp` | 后退，离开被卡区域 |
| `drive_on_heading` | `nav2_behaviors::DriveOnHeading` | 沿指定方向直行 |
| `assisted_teleop` | `nav2_behaviors::AssistedTeleop` | 人工接管辅助 |
| `wait` | `nav2_behaviors::Wait` | 暂停等待 |

### 10.3 Progress Checker

```yaml
progress_checker:
  required_movement_radius: 0.5   # 20s 内必须移动超过 0.5m
  movement_time_allowance: 20.0   # 否则判定为"卡住"→ 触发恢复行为
```

---

## 11. 速度平滑与碰撞监视

### 11.1 Velocity Smoother

控制器的输出是"希望的速度"，不能直接发给电机——需要平滑过渡，防止急加速/急减速。

```yaml
velocity_smoother:
  smoothing_frequency: 20.0      # 20Hz
  feedback: "OPEN_LOOP"          # 开环（不求实际速度反馈）
  max_velocity: [0.8, 0.0, 0.23]    # 最大 [vx, vy, ωz]
  min_velocity: [-0.3, 0.0, -0.3]   # 最小（允许倒车 -0.3 m/s）
  max_accel: [0.8, 0.0, 0.6]        # 最大加速度
  max_decel: [-1.0, 0.0, -1.0]      # 最大减速度（Jazzy 必须负数！）
  odom_topic: "odometry/filtered"   # 用于开环前馈
```

### 11.2 Collision Monitor

在 cmd_vel 发出前做最后一道安全闸门。使用多边形定义机器人轮廓，如果激光点进入多边形范围 → 立即急停。

```yaml
collision_monitor:
  PolygonStop:
    type: "polygon"
    # 定义了机器人底盘的简化多边形（单位 m, base_link 坐标系）
    # 前余量: 270mm, 后余量: 580mm, 宽余量: ±312.5mm
    points: "[[0.27, 0.3125], [0.27, -0.3125],
              [-0.58, -0.3125], [-0.58, 0.3125]]"
    action_type: "stop"           # 动作: 紧急停止
    min_points: 4                 # 至少 4 个激光点进入多边形才触发
    observation_sources: ["scan"]
```

**设计意图**：多边形比机器人实际尺寸稍大（前+7cm, 侧+12.5cm, 后+13cm），提供最后的安全缓冲。如果这个触发了，说明前面的代价地图避障已经失败了。

---

## 12. 建图 — SLAM Toolbox

### 12.1 算法概述

建图模式使用 `slam_toolbox`（`async_slam_toolbox_node`），基于**稀疏位姿图优化（Pose Graph SLAM）**。

```
┌──────────────────────────────────────────────────────────┐
│  SLAM Toolbox (online_async)                              │
│                                                          │
│  前端 (Scan Matching):                                    │
│    - 当前 scan 与已有地图做扫描匹配                        │
│    - 使用相关性扫描 (Correlation Scan Matcher)            │
│    - 输出: 机器人在当前子图中的相对位姿                     │
│                                                          │
│  后端 (Pose Graph Optimization):                         │
│    - 构建位姿图: 节点=机器人关键帧位姿, 边=扫描匹配约束    │
│    - Ceres Solver (LM 优化) → 最小化全局误差              │
│    - 发现回环 → 添加回环约束 → 全局优化修正漂移             │
│                                                          │
│  回环检测:                                                │
│    - 搜索范围: 3m 内找之前访问过的位置                     │
│    - 匹配条件: 粗匹配响应 > 0.35, 精匹配响应 > 0.45       │
│    - 最少间隔: 10 个节点以上                               │
└──────────────────────────────────────────────────────────┘
```

### 12.2 关键参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `minimum_travel_distance` | 0.2m | 平移 20cm 才加新节点 |
| `minimum_travel_heading` | 0.2rad | 旋转 11° 才加新节点 |
| `do_loop_closing` | true | 开启回环检测 |
| `loop_search_maximum_distance` | 3.0m | 回环搜索半径 |
| `loop_match_minimum_chain_size` | 10 | 回环两端至少间隔 10 个节点 |
| `correlation_search_space_dimension` | 0.5m | 扫描匹配搜索窗 |
| `correlation_search_space_resolution` | 0.01m | 搜索分辨率 |
| `resolution` | 0.05m | 地图分辨率 5cm/cell |
| `mode` | `mapping` | 建图模式（可改为 `localization` 做纯定位） |

---

## 13. 完整启动流程

### 13.1 开机后的系统状态

PM2 在系统启动时自动拉起 6 个常驻进程：

```
启动顺序 (PM2):
  1. rsp               → /tf_static                  (robot_state_publisher)
  2. keli_lidar        → /scan                       (激光雷达)
  3. hls_canopen       → CAN→WS:9090                 (电机驱动层)
  4. canopen_ros2      → /odom + /tf(odom→base_link) (电机 ROS2 桥接)
  5. imu_driver        → 串口→WS:8765                (IMU 驱动层)
  6. imu_bridge        → /imu                        (IMU ROS2 桥接)
```

此时 TF 树中已存在：
- `odom → base_link`（canopen_ros2 的原始里程计）
- `base_link → laser`（URDF 静态变换）
- `base_link → imu_link`（URDF 静态变换）

但 `map → odom` 尚不存在 —— 需要启动导航。

### 13.2 `nav.zsh` 执行流程

```
nav.zsh
  └── ros2 launch exp2_nav navigation.launch.py
        │
        ├── [localization_launch]
        │     ├── map_server: 加载 /home/expressone/maps/my_map.yaml
        │     │              发布 /map 话题 (latched)
        │     ├── AMCL: 加载 5000 个粒子，等待 /initialpose
        │     └── lifecycle_manager: 管理 AMCL + map_server 生命周期
        │
        ├── [navigation_launch]
        │     ├── planner_server: SmacPlanner2D
        │     ├── controller_server: RegulatedPurePursuit
        │     ├── behavior_server: Spin/Backup/DriveOnHeading/Wait
        │     ├── smoother_server: SimpleSmoother
        │     ├── velocity_smoother: 速度平滑
        │     ├── collision_monitor: 碰撞急停
        │     ├── global_costmap + local_costmap
        │     ├── waypoint_follower: 航点跟随
        │     └── lifecycle_manager_navigation: 管理上述所有节点
        │
        └── [reloc_manager]
              └── 等待用户 RViz 点击 2D Pose Estimate
```

### 13.3 用户操作时序

```
t=0    用户在 RViz 点击 "2D Pose Estimate"
         ├── 在大致位置点击，拖拽箭头指示车头方向
         │
t=0.1  reloc_manager 拦截 /initialpose
         ├── 用 (1.0, 1.0, 0.5) 大协方差重新发布
         └── 启动 360° 逆时针旋转 (0.2 rad/s)
         
t=0~31  AMCL 每收到一帧 /scan:
         ├── 运动更新: 所有粒子根据 /odometry/filtered 移动
         ├── 测量更新: 用 likelihood_field 计算每个粒子的权重
         └── 重采样: KLD 自适应调节粒子数

t≈31   reloc_manager 里程计累积 ≥ 360°
         ├── 停止旋转 (发送零 cmd_vel)
         └── 如果已收敛 → 状态 CONVERGED
            如果未收敛 → 状态 UNLOCALIZED，提示重新点击

定位完成后:
  map → odom 变换已稳定发布
  /amcl_pose 发布机器人在 map 中的精确位姿
  小车可以接受 /navigate_to_pose 导航目标
```

### 13.4 发送导航目标

```bash
# 方式1: RViz 点击 "Nav2 Goal" 按钮
# 方式2: 命令行
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"
# 方式3: 脚本
./goto.zsh       # 预设目标点 A
./come_here.zsh  # 导航到"当前位置"（测试/停靠）
```

---

## 14. 关键参数调优历史

来自 `config/改动清单.md` 的实战经验总结：

### 14.1 AMCL 定位漂移修复（2026-06-17）

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `z_hit` | 0.5 | 0.95 | 旧值仅 50% 激光点被视为真实测量，粒子群发散漂移 |
| `z_rand` | 0.5 | 0.05 | 旧值 50% 视为随机噪声，AMCL 无法收敛 |

**教训**：`z_hit + z_rand ≈ 1.0`，标准配置 `z_hit=0.95, z_rand=0.05`。

### 14.2 局部代价地图帧切换

局部代价地图的 `global_frame` 在 `odom` 和 `map` 之间反复切换：

- **最初用 `odom`**：EKF 未上线，odom 由 canopen_ros2 发布，漂移严重 → 与 map 对不齐
- **切到 `map`**：用 AMCL 修正后的坐标，但收敛过程中有跳变
- **EKF 上线后切回 `odom`**：EKF 输出的 odom 平滑无跳变，不再依赖 AMCL 修正

**教训**：代价地图帧的选择取决于你的里程计质量。有 EKF/好里程计 → 用 `odom`；里程计差 → 用 `map`。

### 14.3 碰撞多边形迭代

碰撞监视器定义的多边形经过 3 次调整，从最初的粗估计逐步精确到实际底盘尺寸：

```
v1: [[0.55,0.45], [0.55,-0.45], [-0.45,-0.45], [-0.45,0.45]]
    → 过于保守，窄通道误触发

v2: [[0.335,0.45], [0.335,-0.45], [-0.665,-0.45], [-0.665,0.45]]
    → base_link 原点前移 215mm 后偏移

v3: [[0.27,0.3125], [0.27,-0.3125], [-0.58,-0.3125], [-0.58,0.3125]]
    → 根据实际底盘收窄，前余量 200mm, 后 50mm, 侧 ±50mm
```

### 14.4 前驱差速底盘的速度限制

因为是**前驱**（只有前轮有动力），原地旋转时车尾会甩动，所以需要：
- 旋转角速度从标准的 1.0 降到 `0.3 rad/s`
- 角加速度降到 `1.0 rad/s²`
- 角速度上限降到 `0.23 rad/s`

---

## 附录：学习建议

如果要从零理解这套系统，建议按以下顺序阅读代码：

1. **`nav2_params.yaml`** — 从上到下读每个模块的参数和注释，理解每个参数控制什么行为
2. **`reloc_manager.py`** — 不到 200 行，包含完整的定位启动逻辑，展示 AMCL 如何被操控
3. **`ekf.yaml`** — 理解传感器融合的"信什么不信什么"策略
4. **`navigation.launch.py`** — 看看如何把 Nav2 的各组件拼在一起
5. **`改动清单.md`** — 实战调试经验，理解参数调优的思路
6. **`nav2_params.yaml`（重读）** — 带着"为什么这么设"的问题回来看

每条路径、每个参数的设定都有明确的工程原因——理解这些"为什么"比记住参数值重要得多。
