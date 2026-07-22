# EXP2 无人小车导航算法文档

> 源码位置：`D:\Projects\repo\ros2_ws\src\exp2_nav\`
> 生成日期：2026-07-01

---

## 概述

`exp2_nav` 是 EXP2 无人配送小车的导航配置/集成包，**不包含自研算法代码**，所有算法均来自 ROS2 Jazzy 标准生态：

- [Nav2](https://navigation.ros.org/) — 导航框架（定位、规划、控制）
- [robot_localization](https://docs.ros.org/en/jazzy/p/robot_localization/) — 传感器融合（EKF）
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) — SLAM 建图

该包的核心工作在于**参数调优**和**传感器集成架构设计**，所有修改记录见 `config/改动清单.md`。

---

## 1. 定位算法

### 1.1 AMCL（自适应蒙特卡洛定位）

**配置文件**：`config/nav2_params.yaml`（第 5–44 行）

AMCL 是概率定位算法，用粒子滤波器在已知地图上估计机器人位姿。

| 参数 | 值 | 说明 |
|------|-----|------|
| 运动模型 | `DifferentialMotionModel` | 差速驱动机器人模型 |
| 最小粒子数 | 500 | KLD 采样自适应下限 |
| 最大粒子数 | 2000 | KLD 采样自适应上限 |
| 激光模型 | `likelihood_field` | 似然场模型（高斯模糊预计算查表，比 beam model 高效） |
| `z_hit` | 0.95 | 95% 激光点命中已知障碍物 |
| `z_rand` | 0.05 | 仅 5% 建模为随机噪声 |
| `alpha1` | 0.4 | 旋转噪声（故意偏高，抑制差速旋转时的里程计漂移） |
| `alpha2`~`alpha5` | 0.05 | 其余运动噪声参数 |
| 激光最大距离 | 10m | — |
| 里程计来源 | `/odometry/filtered` | **EKF 融合输出**，非原始轮式里程计 |

**关键设计**：`z_hit=0.95` 是经过实测修正的值（早期 `z_hit=0.5, z_rand=0.5` 曾导致粒子发散）。

### 1.2 EKF（扩展卡尔曼滤波）

**配置文件**：`config/ekf.yaml`
**ROS 包**：`robot_localization`（`ekf_filter_node`）

在 `odom` 坐标系下运行，融合两类传感器数据：

| 传感器 | 融合的数据 | 不融合的数据及原因 |
|--------|-----------|-------------------|
| 轮式里程计 `/odom` | `vx`（线速度）、`vyaw`（角速度） | 绝对位姿 x/y/yaw — 避免与 AMCL 双重计数 |
| IMU `/imu` | 陀螺仪 Z 轴角速度 | 线加速度 — 双重积分导致噪声爆炸；方向角 — 磁力计在电机附近不可靠 |

| 参数 | 值 | 说明 |
|------|-----|------|
| 频率 | 20 Hz | 与里程计输入匹配 |
| 模式 | `two_d_mode: true` | 忽略 z/roll/pitch（平面机器人） |
| 坐标系 | `odom` → `base_link` | EKF 独占 odom→base_link 的 TF 发布 |

**关键设计**：canopen_ros2 以 `--publish-tf false` 运行，避免双重 TF 发布。

---

### 1.3 定位架构总览

```
轮式编码器 (DS20270DA, 20Hz)
  ── CANopen 0x6063 位置计数 ── canopen_ros2 ── /odom (vx, vyaw) ──┐
                                                                      ├── EKF ── /odometry/filtered ── AMCL ── map→odom TF
IMU (JY901, 100Hz)                                                    │
  ── imu_driver ── imu_bridge ── /imu (gyro_z) ──────────────────────┘
                                                                      
LiDAR (Keli LSE-2027DE, 360°, ~10Hz)
  ── /scan ─────────────────────────────────────────── AMCL 激光匹配 ─┘
```

- **EKF** 负责短期航迹推算质量
- **AMCL** 负责全局定位（粒子滤波 + 激光-地图匹配）
- 两者级联：EKF 输出作为 AMCL 的运动模型输入

---

## 2. 路径规划算法

### 2.1 全局规划器：SmacPlanner2D（Hybrid-A\*）

**配置文件**：`config/nav2_params.yaml`（第 213–232 行）

| 参数 | 值 | 说明 |
|------|-----|------|
| 插件 | `nav2_smac_planner::SmacPlanner2D` | 2026-06-03 从 NavfnPlanner (Dijkstra) 切换 |
| 运动基元 | `REEDS_SHEPP` | 支持前进/后退/原地旋转 |
| 角度量化 | 72 bins | 5°/bin |
| 最大迭代 | 10000 | — |
| 最大规划时间 | 8.0 秒 | — |
| 解析扩展比率 | 3.5 | 向目标做解析路径扩展的频率 |
| 代价旅行乘数 | 2.0 | 惩罚靠近障碍物的路径 |
| 未知空间 | `allow_unknown: false` | 未知区域 = 不可通行 |
| 降采样 | `downsample_costmap: false` | 保持 0.05m 完整分辨率 |
| 规划频率 | 2.0 Hz | — |

**Hybrid-A\* 原理**：在连续空间搜索（非栅格中心），用 Reed-Shepp 曲线做启发式，兼顾最优性和运动学可行性。

### 2.2 路径平滑器：SimpleSmoother

**配置文件**：`config/nav2_params.yaml`（第 234–242 行）

| 参数 | 值 |
|------|-----|
| 容差 | 1e-10 |
| 最大迭代 | 1000 |
| 优化 | 启用 |

在全局规划器和局部控制器之间对路径做平滑处理。

### 2.3 行为树

**配置文件**：`config/nav2_params.yaml`（第 46–61 行）

- **主行为树**：`navigate_w_replanning_only_if_path_becomes_invalid.xml`
  - 仅在全局代价图变化导致路径失效时才重规划（而非每次控制器失败都重规划）
  - 减少重复全局路径发布
- **恢复行为**：`spin`（原地旋转）、`backup`（后退）、`drive_on_heading`（定向行驶）、`assisted_teleop`（辅助遥操）、`wait`（等待）

---

## 3. 控制算法

### 3.1 Regulated Pure Pursuit（RPP，受调节纯追踪）

**配置文件**：`config/nav2_params.yaml`（第 87–113 行）
**插件**：`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

**纯追踪算法原理**：

1. 在全局路径上找到前方一个**前视点**（lookahead = 0.6m，自适应 0.4~1.0m）
2. 计算从当前位置到前视点的**圆弧曲率**
3. 将曲率转换为线速度 / 角速度指令

**三层速度调节（"Regulated"的含义）**：

| 调节类型 | 机制 | 参数 |
|---------|------|------|
| 曲率调节 | 弯道减速 | 最小曲率半径 0.5m，最低速度 0.1 m/s |
| 代价调节 | 前方有障碍物减速 | 扫描前视点前方 1.5m，增益 1.5 |
| 接近调节 | 接近目标时减速 | 从全速线性降至 0.05 m/s（缩放距离 4.0m） |

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| 期望线速度 | 1.0 m/s | — |
| 前视距离 | 0.6m（0.4~1.0m 自适应） | 随速度缩放 |
| 前视时间 | 1.5 秒 | — |
| 目标容差 xy | 0.03m | 对接精度要求 |
| 目标容差 yaw | 0.05 rad（~2.9°） | 对接精度要求 |
| 旋转到朝向 | 启用 | 角度误差 > 0.4 rad（~23°）时触发 |
| 碰撞检测 | 启用 | 沿规划路径检测潜在碰撞 |
| 最大线加速度 | 1.0 m/s² | — |
| 最大角加速度 | 1.0 rad/s² | — |
| 故障容忍时间 | 3.0 秒 | 无有效轨迹 3 秒后才报故障（避让行人） |
| 控制器频率 | 20 Hz | — |

### 3.2 速度平滑器（Velocity Smoother）

**配置文件**：`config/nav2_params.yaml`（第 282–296 行）

| 参数 | 值 | 说明 |
|------|-----|------|
| 反馈类型 | `OPEN_LOOP` | 无闭环速度 PID 跟踪 |
| 最大线速度 | 0.8 m/s | x 方向 |
| 最小线速度 | -0.3 m/s | 允许后退 |
| 最大角速度 | 0.23 rad/s | — |
| 最大加速度（线） | 0.8 m/s² | — |
| 最大减速度（线） | -1.0 m/s² | — |
| 最大加速度（角） | 0.6 rad/s² | — |
| 频率 | 20 Hz | — |
| 里程计来源 | `/odometry/filtered` | EKF 输出 |

功能：加速度/减速度限制 + 最大速度钳位 + 死区滤波。

### 3.3 碰撞监视器（Collision Monitor）

**配置文件**：`config/nav2_params.yaml`（第 298–322 行）

额外安全层，独立于控制器运行：

- 在机器人多边形内（前 +0.27m、后 -0.58m、宽 ±0.3125m）检测激光点
- ≥4 个激光点落入多边形 → **紧急停止**（阻塞 cmd_vel）
- 最后一道安全防线

---

## 4. 建图算法

### 4.1 SLAM Toolbox — Online Async 模式

**配置文件**：`config/mapper_params_online_async.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| 节点 | `async_slam_toolbox_node` | 在线异步 SLAM |
| 求解器 | Ceres Solver | — |
| 线性求解器 | `SPARSE_NORMAL_CHOLESKY` | — |
| 预处理器 | `SCHUR_JACOBI` | — |
| 信任域策略 | `LEVENBERG_MARQUARDT` | — |
| 扫描匹配 | 相关性扫描匹配 | 搜索空间 0.5m，分辨率 0.01m |
| 涂抹偏差 | 0.1 | — |
| 回环检测 | 启用 | 搜索空间 8.0m，分辨率 0.05m |
| 最小回环链长 | 10 帧 | 至少 10 帧才考虑回环 |
| 位姿图更新间隔 | 2.0 秒 | — |
| 地图分辨率 | 0.05m | — |
| 激光距离 | 0.0–12.0m | 栅格化范围 |
| 最小移动 | 0.2m 或 0.2 rad | 不足则跳过该帧 |
| 地图保存路径 | `/home/expressone/maps/map` | 通过 nav2_map_server 保存 |

**启动方式**：`mapping.launch.py`（导航栈 + SLAM Toolbox + 地图保存服务）

---

## 5. 传感器融合总览

### 5.1 硬件传感器

| 传感器 | 型号 | 频率 | 数据 |
|--------|------|------|------|
| 2D LiDAR | Keli LSE-2027DE | ~10 Hz | 360° 激光扫描，0.05–10m |
| IMU | JY901 | 100 Hz | 加速度 + 角速度 + 方向角 |
| 轮式编码器 | DS20270DA (CANopen 0x6063) | 20 Hz | 位置计数（5600 线/转，4 倍频解码） |

### 5.2 融合架构

```
                        ┌─────────────────────────────────────────────┐
                        │                AMCL（全局定位）               │
┌──────────┐            │  粒子滤波器 + 似然场激光匹配 + 静态地图匹配    │
│ 轮式编码器 │            │                                             │
│ (CANopen) │── vx,vyaw ─┤                                             │
└──────────┘      │      └──────────┬──────────────────────────────────┘
                  │                 │ map→odom TF
                  ▼                 │
            ┌──────────┐            │
            │   EKF    │────────────┘
            │ (20Hz)   │  /odometry/filtered
            └──────────┘
                  ▲
┌──────────┐      │
│   IMU    │─gyro_z┘
└──────────┘

┌──────────┐      ┌─────────────────────────────────────────┐
│  LiDAR   │──/scan──► 代价图（VoxelLayer + ObstacleLayer）   │
└──────────┘      │  组合方式: max  保持时间: 2.0s            │
                  └─────────────────────────────────────────┘
```

**三层融合**：

| 层 | 融合内容 | 输出 |
|----|---------|------|
| EKF | 轮式里程计微分 (vx,vyaw) + IMU 陀螺仪 (gyro_z) | `/odometry/filtered` |
| AMCL | EKF 输出 + 2D LiDAR 扫描 + 静态地图 | `map→odom` TF, `/amcl_pose` |
| 代价图 | LiDAR 扫描 + 静态地图（局部用 VoxelLayer 3D 体素） | 局部/全局代价图 |

---

## 6. 自主对接

**配置文件**：`config/nav2_params.yaml`（第 336–344 行）

| 参数 | 值 | 说明 |
|------|-----|------|
| 插件 | `opennav_docking::SimpleChargingDock` | 简易充电桩对接 |
| 暂存偏移 | -0.3m (x) | 对接前在充电桩前 0.3m 处暂存 |

---

## 7. 算法清单速查

| 类别 | 算法 | ROS 包 | 原理 |
|------|------|--------|------|
| 定位 | AMCL | `nav2_amcl` | 蒙特卡洛粒子滤波 + 似然场模型 |
| 传感器融合 | EKF | `robot_localization` | 扩展卡尔曼滤波（级联架构） |
| 全局规划 | SmacPlanner2D | `nav2_smac_planner` | Hybrid-A\* + Reed-Shepp 曲线 |
| 路径平滑 | SimpleSmoother | `nav2_smoother` | 迭代平滑优化 |
| 局部控制 | Regulated Pure Pursuit | `nav2_regulated_pure_pursuit_controller` | 几何纯追踪 + 曲率/代价/接近三层调节 |
| 速度平滑 | Velocity Smoother | `nav2_velocity_smoother` | 开环加减速限制 + 钳位 |
| 碰撞避免 | Collision Monitor | `nav2_collision_monitor` | 多边形内激光点计数 → 紧急停止 |
| 建图 | SLAM Toolbox | `slam_toolbox` | 在线异步图优化 SLAM + Ceres 求解 |
| 对接 | SimpleChargingDock | `opennav_docking` | 充电桩对接控制 |
| 行为管理 | Behavior Tree | `nav2_bt_navigator` | 行为树 + 恢复行为 |

---

## 8. 关键设计决策

1. **EKF 只融合速度微分，不融合绝对位姿** — 避免与 AMCL 粒子传播双重计数
2. **IMU 只融合陀螺仪角速度，不融合线加速度和方向角** — 双重积分噪声爆炸 + 磁力计不可靠
3. **AMCL 使用 likelihood_field 而非 beam model** — 计算效率更高，对非结构化环境更鲁棒
4. **未知空间 = 不可通行** — 防止规划器穿越未建图区域
5. **Hybrid-A\* + Reed-Shepp** — 生成运动学可行的路径，支持前进/后退/原地旋转
6. **开环速度控制** — 无闭环 PID，依赖速度平滑器的加减速限制保证平顺性
7. **碰撞监视器独立于控制器** — 最后一道硬件级安全防线
