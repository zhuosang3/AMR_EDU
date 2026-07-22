# 重定位管理器完整文档

> 项目: EXP2 无人小车 AMR 导航系统
> 需求: 2.1 多模式自主重定位
> 平台: Orange Pi 5 Plus / ROS2 Jazzy / Nav2
> 部署路径: `~/ros2_ws/src/exp2_nav/`
> 日期: 2026-07-14（初始实现）→ 2026-07-16（实测诊断 + 改造）→ 2026-07-18（360°旋转）→ 2026-07-22（代码级核查 + 刹车修复）
> Git 状态: ⚠️ **尚未提交**，所有重定位改动均为 working directory 未暂存/未跟踪文件

---

## 1. 最终效果（用户操作流程）

```
启动硬件驱动（雷达/底盘/IMU — start_*.sh 逐个启动）
  → 启动 nav：ros2 launch exp2_nav navigation.launch.py
  → 打开 rviz2，Fixed Frame = map，看到地图 + 激光（激光在地图原点占位）
  → 点 rviz 工具栏的「2D Pose Estimate」（快捷键 P）
    在地图上点选小车的真实位置 + 拖出车头朝向
  → 小车自动 360° 原地旋转（0.2rad/s，约33秒）
    AMCL 粒子滤波快速收敛，协方差达标
  → 自动停车归位（回到起始朝向），状态 CONVERGED
  → 点「2D Goal Pose」选目标点开始导航
```

**全程不需要推车、不需要手动发命令。** 点一下，车转一圈，收敛。

---

## 2. 架构

```
reloc_manager（自定义节点，~195行 Python）
    │
    ├─→ /initialpose（PoseWithCovarianceStamped）──→ AMCL ←── /scan（激光 28Hz，frame_id=laser）
    │                                                        ←── /odom（底盘）
    │                                                        ←── /map（地图）
    │
    ├─← /amcl_pose（监控协方差 cov[0]/cov[7]，判断收敛 <0.1）
    │
    ├─→ /cmd_vel（Twist，旋转时直驱底盘）──→ canopen_ros2
    │
    ├─← /odometry/filtered（里程计 yaw，精确累计旋转角度 360°）
    │     ↑ EKF(robot_localization) 20Hz 输出，融合轮速
    │
    ├─→ /reloc/status（String，1Hz 周期发布状态）
    │
    ├─← /initialpose（rviz 2D Pose Estimate 在此触发）
    ├─← /reloc/set_manual_pose（模式 3 手动设点）
    └─   /reloc/dock_calib（Trigger 服务，模式 2 充电桩标定）

TF 树: map → odom（AMCL 28Hz）→ base_link（EKF 20Hz）→ laser / imu_link（static）
```

---

## 3. 三种模式

状态机: `UNLOCALIZED → LOCALIZING → CONVERGED`

| 模式 | 触发方式 | 协方差 | 旋转 | 适用场景 |
|---|---|---|---|---|
| **rviz 点选**（主模式） | rviz `2D Pose Estimate` 工具 | x/y=1.0, yaw=0.5 | ✅ 360°单向旋转 | 日常启动，容许 ~1m 点选误差 |
| **充电桩标定** | `ros2 service call /reloc/dock_calib` | 0.01（精确） | ❌ | 车已停入充电桩，座标准确已知 |
| **手动设点** | `/reloc/set_manual_pose` 话题 | x/y=1.0, yaw=0.5 | ✅ 360°单向旋转 | 脚本/命令行给定坐标 |

**为什么点选模式用 cov=1.0（σ=1m, 95% 覆盖 ±2m）**: 用户对照地图点选有 ~1m 误差，1.0 的协方差覆盖这个误差范围，360° 旋转提供 ~870 次 scan matching 机会足以收敛。原 cov=0.15 要求误差 <0.5m，太苛刻。

**为什么充电桩标定不旋转**: 坐标精确已知，协方差 0.01（σ≈0.1m），直接 CONVERGED，无需辅助运动。

---

## 4. 代码级实现细节（基于 2026-07-22 香橙派实测代码）

### 4.1 `reloc_manager.py`（~195行）

路径: `~/ros2_ws/src/exp2_nav/scripts/reloc_manager.py`
状态: **未跟踪文件**（`git status` 显示 `scripts/` 目录为 Untracked）

**核心类 `RelocManager(Node)`**:

```python
class RelocManager(Node):
    DEBOUNCE_SEC = 0.5  # 去抖窗口

    # 参数（从 nav2_params.yaml 读取，有代码默认值）
    def __init__(self):
        # dock 坐标、manual_cov、converged_xy_cov、localize_timeout_sec
        # swing_enabled/swing_speed/swing_angle_deg/swing_tolerance_deg
        ...
```

**关键设计点**:

1. **去抖机制（0.5s 时间窗口）**: 
   - `_last_initial_pose_time` 记录最后一次发布/收到 initialpose 的时间
   - `_initial_pose_cb` 中如果 `now - _last_initial_pose_time < 0.5s` 则忽略
   - `publish_initial_pose` 也会刷新时间戳，阻止自回响
   - 解决了 rviz 拖拽连发 + DDS 自回响风暴问题

2. **里程计反馈精确控角**:
   - `_odom_cb` 从 `/odometry/filtered` 提取 yaw
   - `_rotation_step` 每 50ms 累计 `delta = norm_angle(yaw - prev_yaw)`
   - `total_rotated >= swing_angle` 时精确停车
   - 安全兜底: `rotation_timeout = swing_angle / speed * 1.5`（约 47s）

3. **收敛后继续转完**:
   - `_amcl_pose_cb` 中 cov<0.1 时设 `_converged = True` + `state = 'CONVERGED'`
   - 但 `_rotation_step` 继续运行直到 `total_rotated >= swing_angle`
   - 保证车回到起始朝向

4. **1Hz 周期状态发布**:
   - `timer = create_timer(1.0, _timer_cb)` 每秒发布状态
   - 同时也做超时检查: `LOCALIZING > timeout → UNLOCALIZED`

5. **`/cmd_vel` 直发底盘**:
   - 绕过了 nav2 的 controller/velocity_smoother/collision_monitor 链路
   - 原因: 重定位时 controller 空闲，nav2 链路无 cmd_vel 输出

### 4.2 `nav2_params.yaml` 配置

路径: `~/ros2_ws/src/exp2_nav/config/nav2_params.yaml`
状态: **已修改未暂存**（`modified: config/nav2_params.yaml`）

**reloc_manager 段**（YAML 中显式配置的参数）:

```yaml
reloc_manager:
  ros__parameters:
    use_sim_time: false
    dock:
      x: 1.768
      y: 3.188
      yaw: 2.361          # ≈135°
    manual_cov_xx: 1.0    # σ=1m
    manual_cov_yy: 1.0
    manual_cov_yyaw: 0.5  # σ≈0.7rad≈40°
    converged_xy_cov: 0.1 # 收敛阈值 σ≈0.316m
    localize_timeout_sec: 60.0
```

**摆动参数未写在 YAML 中**，使用代码层默认值:
| 参数 | 默认值 | 说明 |
|---|---|---|
| `swing_enabled` | `True` | 是否启用旋转辅助 |
| `swing_speed` | `0.2` rad/s | 旋转角速度 |
| `swing_angle_deg` | `360.0` | 旋转角度 |
| `swing_tolerance_deg` | `4.0` | 角控容差（代码中声明但未实际使用） |

**AMCL 段关键参数**:

```yaml
amcl:
  ros__parameters:
    max_particles: 5000     # 500→5000，全局定位需要高粒子数
    min_particles: 500
    laser_model_type: likelihood_field
    recovery_alpha_slow: 0.001   # 恢复机制
    recovery_alpha_fast: 0.1
    set_initial_pose: true
    initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}
    robot_model_type: nav2_amcl::DifferentialMotionModel
    tf_broadcast: true
    update_min_a: 0.1
    update_min_d: 0.25
```

### 4.3 `navigation.launch.py`

路径: `~/ros2_ws/src/exp2_nav/launch/navigation.launch.py`
状态: **已修改未暂存**

**实际功能**（注：顶部 docstring 已过时，仍写"3s 自动全局定位"和"PM2 需先启动"）:

```python
# IncludeLaunchDescription 引入 Nav2 的 localization_launch + navigation_launch
# GroupAction 内包含: localization + navigation + reloc_manager Node
reloc_manager = Node(
    package="exp2_nav",
    executable="reloc_manager.py",
    name="reloc_manager",
    output="screen",
    parameters=[params_file],
)
```

### 4.4 `CMakeLists.txt` / `package.xml`

**CMakeLists.txt** 改动: 新增 `install(PROGRAMS scripts/reloc_manager.py DESTINATION lib/${PROJECT_NAME})`

**package.xml** 新增依赖: `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`

### 4.5 辅助脚本

| 脚本 | 路径 | 日期 | 功能 |
|---|---|---|---|
| `reloc_dock.zsh` | `~/reloc_dock.zsh` | 2026-07-20 | 充电桩标定：先 MQTT 确认 charge_flag=1，再调 `/reloc/dock_calib` |
| `reloc_manual.zsh` | `~/reloc_manual.zsh` | 2026-07-14 | 手动设点：`./reloc_manual.zsh X Y YAW_DEG` |
| `reloc_status.zsh` | `~/reloc_status.zsh` | 2026-07-14 | 查看状态：`ros2 topic echo --once /reloc/status` |

**`reloc_dock.zsh` 的 MQTT 安全校验**（2026-07-20 更新）:
- 用 `mosquitto_sub -t /car/power -C 1 -W 5` 取一条电源消息
- 检查 `charge_flag == 1` 才触发标定
- 防止未插电时误触发

---

## 5. 全量改动演进

### 5.1 会话一（2026-07-14，初始实现）

从零搭建三模式重定位系统。初始设计: 启动 3s 自动大协方差(10,10,1)撒全图 → AMCL 粒子滤波收敛。

| 文件 | 改动 |
|---|---|
| `scripts/reloc_manager.py` | 新建，~256行，三模式状态机 + init_timer 自动触发 |
| `config/nav2_params.yaml` | 新增 AMCL + reloc_manager 参数段 |
| `launch/navigation.launch.py` | 引入 nav2 localization/navigation + reloc_manager Node |
| `CMakeLists.txt` / `package.xml` | 安装脚本 + 4个新依赖 |
| `~/reloc_*.zsh` | 3个辅助脚本 |

✅ 编译通过，⏳ 待实测

### 5.2 会话二（2026-07-16，实测诊断 + 改造）

**根因**: 大协方差撒全图 + 静止 = AMCL 协方差从 10 只降到 ~5.94 卡死（实测 `/amcl_pose` cov[0]=5.94）。粒子滤波在静止时协方差有物理下限 ~0.1~0.12，运动才是收敛驱动力。

**改造**:
| 改动 | 从 | 到 |
|---|---|---|
| 定位触发 | 启动 3s 自动 | rviz 点选触发 |
| 协方差 | 10/10/1 | 0.15/0.15/0.05 |
| 辅助运动 | 无（静止等60s） | 自动 ±60° 摆动 |
| 收敛阈值 | 0.04 | 0.1 |
| 去抖 | stamp 集合匹配 | 0.5s 时间窗口 |
| 状态发布 | 仅变化时 | 1Hz 周期 |

**实测验证**:
| 阶段 | cov_x | 结果 |
|---|---|---|
| 大协方差 + 静止 | 10→5.94 | 超时 |
| 小协方差 + 静止 | 0.15→0.12 | 不到阈值 |
| 小协方差 + 推车 | 0.15→0.002 | 运动后极速收敛 |
| **小协方差 + 自动摆动** | 0.15→<0.10 (~2s) | ✅ 最终方案 |

### 5.3 会话三（2026-07-18，360° 大协方差旋转）

**问题**: ±60° 摆动 + cov=0.15 对点选精度要求极高（<0.5m），实际点选 >1m 偏差无法收敛。

**改造**:
| 参数 | 旧值 | 新值 |
|---|---|---|
| 摆动幅度 | ±60° | 360° 单向 |
| cov_xx/yy | 0.15 | **1.0** |
| cov_yyaw | 0.05 | **0.5** |
| 控角方式 | 时间控制 | **里程计反馈精确控角** |
| 停止条件 | 收敛即停 | **收敛后继续转完一整圈** |

**修过的 bug**:
1. `self.rotation_duration = 0.0` 写在 `_load_params()` 之后覆盖了正确值 → 改为在 `_load_params()` 内部计算
2. 时间控制导致 overshoot 5-10° → 改为里程计累计转角

### 5.4 会话四（2026-07-22，急刹车导致粒子云偏移修复）

**问题**: 旋转 360° 完成后，刹车瞬间粒子云偏移约 10°。转完半圈已收敛、粒子云和墙壁吻合，但刹车时 cmd_vel 从 0.2 rad/s 瞬间跳 0，急刹车导致激光扫描运动畸变不均匀 → AMCL scan matching 产生错误修正。

**根因**: `_stop_rotation()` 瞬间发 `Twist()` + 取消定时器，物理底盘急刹车导致：
1. 激光雷达（28Hz，~35ms/帧）扫描畸变不均匀（前半帧快、后半帧慢）
2. 轮子微滑，里程计与实际位移不一致
3. IMU 角加速度尖峰，EKF 输出短暂抖动

**修复**: 用 **500ms 线性减速斜坡 + 300ms 零速保持**替代瞬间刹车。

| 改动 | 说明 |
|---|---|
| 新增 `DECEL_DURATION = 0.5` `DECEL_HOLD = 0.3` | 减速阶段参数 |
| `__init__` 新增 `self.decelerating = False` | 减速状态标志 |
| `_start_rotation` 重置 `decelerating` | 每次旋转重置 |
| `_rotation_step` **顶部**新增 Phase 2 减速逻辑 | 先检查 `decelerating`，若是则执行斜坡，不走正常旋转 |
| `_stop_rotation` **重写** | 不再瞬间停，改设 `decelerating=True` + 保留定时器由 `_rotation_step` 接管减速 |
| `rotation_timeout` +1.0s | 补偿减速耗时 |

**效果**: 匀速旋转 + 平滑减速 → 每帧激光扫描畸变始终均匀 → AMCL 粒子云丝滑到位，不再跳变。

**源码**: 仅修改 `scripts/reloc_manager.py`（~314 行，从 ~195 行增至 ~314 行），备份 `.bak`

**其他讨论**:
- 失能推车点云不跟：AMCL 正常行为，推车轮滑导致运动/观测不一致，不影响实际导航
- linear.y 恒为 0：差速底盘物理限制，非 bug

### 5.5 Git 状态（2026-07-22 核查）

⚠️ **所有重定位改动均未提交到 git**:

```
未暂存修改 (modified):
  - CMakeLists.txt          (新增 install scripts)
  - config/nav2_params.yaml (+123 -34 行，新增 AMCL + reloc_manager 段)
  - launch/navigation.launch.py (+19 -5 行)
  - package.xml             (+4 行新依赖)

未跟踪文件 (untracked):
  - scripts/reloc_manager.py    (~195行，核心节点)
  - config/nav2_params.yaml.bak (备份文件)
```

Git 历史中没有任何 reloc/重定位相关 commit。最近的 commit 都是导航调参相关（goal tolerance、velocity_smoother、costmap、planner频率等）。

---

## 6. 诊断命令速查

```bash
# 重定位状态（1Hz 周期发布，一次必收到）
ros2 topic echo --once /reloc/status

# AMCL 协方差（收敛指标，<0.1 即为 CONVERGED）
ros2 topic echo --once /amcl_pose --field pose.covariance 2>/dev/null \
  | grep -oE '[0-9]+\.[0-9]+' | head -1

# 查看 reloc_manager 完整日志（点选触发 → 旋转 → 收敛）
grep reloc_manager /tmp/nav_s.log | tail -10

# TF 树完整性（应有 map→odom→base_link→laser）
ros2 run tf2_tools view_frames

# 雷达数据正常
ros2 topic hz /scan

# 底盘在线
ros2 topic info /cmd_vel | grep canopen

# 查看所有 nav 节点（确认单实例，无残留）
ros2 node list | grep -E 'amcl|reloc_manager'
```

---

## 7. 已知限制与待做

1. **`/cmd_vel` 直发底盘**，绕过 collision_monitor。原地旋转风险低，但不在 nav2 安全链内。

2. **点选偏差 >2m 可能不收敛**：cov=1.0 覆盖 ±2m，但粒子密度在边缘低。360° 旋转已大幅改善，极端情况仍需重新点选。

3. **旋转耗时 ~33 秒**：比之前 ±60° 摆动（~10s）慢，但换来更高的收敛成功率和对点选误差的容忍。

4. **轮距/里程计标定影响转角精度**：转角由 `/odometry/filtered` 累计计算，轮距不准会导致实际转角偏离 360°。

5. **launch docstring 过时**：仍写"3s 自动全局定位"和"PM2 需先启动"，不影响运行但易误导。

6. **PM2 未安装 / 无 systemd 自启**：重启 Orange Pi 后所有驱动需手动逐个拉起。

7. **⚠️ 代码未提交 git**：所有重定位改动仅在香橙派本地 working directory，无版本控制保护。建议尽早 commit + push。

---

## 8. 文件清单（完整）

```
~/ros2_ws/src/exp2_nav/
├── scripts/
│   └── reloc_manager.py          ← 核心节点（~195行，未跟踪）
├── config/
│   ├── nav2_params.yaml          ← 含 reloc_manager + AMCL 段（已修改未暂存）
│   ├── nav2_params.yaml.bak      ← 备份文件（未跟踪）
│   ├── ekf.yaml
│   ├── mapper_params_online_async.yaml
│   └── 改动清单.md
├── launch/
│   ├── navigation.launch.py      ← 含 reloc_manager Node（已修改未暂存）
│   └── mapping.launch.py
├── rviz/
│   └── exp2.rviz
├── CMakeLists.txt                ← 含 install scripts（已修改未暂存）
├── package.xml                   ← 含4个新依赖（已修改未暂存）
└── 资料整理.md

~/ (home)
├── reloc_dock.zsh                ← 充电桩标定脚本（MQTT安全校验版）
├── reloc_manual.zsh              ← 手动设点脚本
└── reloc_status.zsh              ← 状态查看脚本
```
