# EXP2 导航参数优化记录

> **日期**: 2026-07-21
> **文件**: `/home/expressone/ros2_ws/src/exp2_nav/config/nav2_params.yaml`
> **备份**: `nav2_params.yaml.bak`
> **编译**: `colcon build --packages-select exp2_nav` ✅ 通过
> **机器人**: 香橙派5 Plus 教育版无人小车，差分驱动
> **需求文档**: `快递运送智能小车技术需求说明书（与人形机器人协同赛项）.docx`

---

## 1. 问题描述

实地测试发现：
- **速度突变**：行驶中速度不平滑，加速/减速生硬
- **障碍物附近突然拐弯**：遇到障碍物时出现大幅急转

## 2. 根因分析

| 根因 | 严重度 | 现象 |
|---|---|---|
| 车体用圆形模型(半径0.35m)，实际为50cm方形车体，后缘超出圆外7.3cm | 🔴 严重 | 车尾裸露在代价地图覆盖范围外 |
| 膨胀半径仅10cm，角落安全余量仅9.6cm < 定位精度要求10cm | 🔴 严重 | 路径贴脸走，一次定位抖动就可能擦碰 |
| 代价衰减因子2.5，cost从致命到自由仅45cm过渡带 | 🔴 严重 | 控制器频繁大幅调速 |
| 规划器使用REEDS_SHEPP运动模型允许倒车 | 🟠 中等 | 路径包含cusp尖点，原地倒车体验为"突然拐弯" |
| BT被动重规划(仅路径失效时触发) | 🟠 中等 | 障碍物堵死后才重规划，新路径可能是90°急转 |
| 速度平滑器OPEN_LOOP模式 | 🟡 较轻 | 不感知真实速度，无法补偿底盘延迟 |

## 3. 车体几何数据

```
实测: 50cm × 50cm 正方形（俯视图）
base_link: 前轮轴心，离前缘76.5mm，离左右沿各25cm

坐标系 (base_link原点):
        前缘 +0.077m
    ┌──────────────────────┐
    │                      │  ← 宽 ±0.25m
    │         ●            │  ← base_link
    │                      │
    └──────────────────────┘
        后缘 -0.423m

关键距离 (center→)：
  前缘: 0.077m    后缘: 0.423m    侧缘: 0.250m
  前角: 0.262m    后角: 0.491m
```

## 4. 修改清单（共20处）

### 4.1 车体建模（9处）

| # | 参数路径 | 旧值 | 新值 | 理由 |
|---|---|---|---|---|
| 1 | `local_costmap.ros__parameters` | `robot_radius: 0.35` | 删除 | 圆模型不适用于方形车体 |
| 2 | `local_costmap.ros__parameters` | — | `footprint: "[[0.08,0.25],[-0.42,0.25],[-0.42,-0.25],[0.08,-0.25]]"` | 按实际尺寸50×50cm精确多边形 |
| 3 | `global_costmap.ros__parameters` | `robot_radius: 0.35` | 删除 | 同上 |
| 4 | `global_costmap.ros__parameters` | — | `footprint: (同local)` | 两costmap一致 |
| 5 | `local_costmap.inflation_layer.inflation_radius` | `0.1` | `0.3` | 车体外30cm均匀安全缓冲 |
| 6 | `global_costmap.inflation_layer.inflation_radius` | `0.1` | `0.3` | 同上 |
| 7 | `local_costmap.inflation_layer.cost_scaling_factor` | `2.5` | `1.0` | 代价衰减更平缓，降速更渐进 |
| 8 | `global_costmap.inflation_layer.cost_scaling_factor` | `2.5` | `1.0` | 同上 |
| 9 | `collision_monitor.PolygonStop.points` | `[[0.27,0.3125],[-0.58,0.3125],[-0.58,-0.3125],[0.27,-0.3125]]` (85×62.5cm) | `[[0.08,0.25],[-0.42,0.25],[-0.42,-0.25],[0.08,-0.25]]` (50×50cm) | 与车体一致 |

### 4.2 速度平滑（4处）

| # | 参数路径 | 旧值 | 新值 | 理由 |
|---|---|---|---|---|
| 10 | `controller_server.FollowPath.cost_scaling_dist` | `1.5` | `0.8` | 障碍物降速更渐进 |
| 11 | `controller_server.FollowPath.cost_scaling_gain` | `1.5` | `1.0` | 降速幅度减至默认值 |
| 12 | `controller_server.FollowPath.max_allowed_time_to_collision_up_to_carrot` | `1.0` | `1.5` | 碰撞预警从0.8m延至1.2m(以0.8m/s计) |
| 13 | `velocity_smoother.enable_odometry_subscription` | `true` | 删除 | Nav2源码中不存在此参数，写了无效 |

> **注**: `velocity_smoother.feedback` 保持 `OPEN_LOOP` 不动。先验证基础参数，若速度仍不平滑再切 `CLOSED_LOOP`（只需改一行）。

### 4.3 转弯平滑（7处）

| # | 参数路径 | 旧值 | 新值 | 理由 |
|---|---|---|---|---|
| 14 | `planner_server.GridBased.motion_model_for_search` | `"REEDS_SHEPP"` | `"DUBIN"` | 需求"无倒退"，消除cusp尖点 |
| 15 | `bt_navigator.default_nav_to_pose_bt_xml` | `...replanning_only_if_path_becomes_invalid.xml` | `...replanning_and_recovery.xml` | 主动重规划，满足需求"2分钟防卡死" |
| 16 | `controller_server.FollowPath.regulated_linear_scaling_min_radius` | `0.5` | `0.9` | 不急刹，有inflation 0.3兜底 |
| 17 | `controller_server.FollowPath.rotate_to_heading_min_angle` | `0.4` (23°) | `0.6` (34°) | 小角度偏差行驶中微调，不原地自转 |
| 18 | `controller_server.FollowPath.max_angular_accel` | `1.0` | `0.5` | 转弯启停更柔和 |
| 19 | `controller_server.FollowPath.lookahead_dist` | `0.6` | `0.8` | 提前看更远，转弯预判更充分 |
| 20 | `planner_server.GridBased.analytic_expansion_ratio` | `3.5` | `3.0` | 路径略松弛，给smoother留余量 |

### 4.4 全局视野（1处——已计入4.1）

| # | 参数路径 | 旧值 | 新值 | 理由 |
|---|---|---|---|---|
| — | `global_costmap.obstacle_layer.scan.obstacle_max_range` | `2.5` | `5.0` | 全局规划提前感知远处障碍，避免临头急拐 |

## 5. 未修改的关键参数

| 参数 | 值 | 原因 |
|---|---|---|
| `velocity_smoother.feedback` | `OPEN_LOOP` | 先验证基础参数效果；若仍不平滑再切CLOSED_LOOP |
| `desired_linear_vel` | `1.0` | smoother max_velocity已限至0.8，实际生效上限即0.8 |
| `local_costmap.obstacle_max_range` | `2.5` | 局部代价地图仅需近处障碍物 |
| `velocity_smoother.max_velocity` | `[0.8, 0.0, 0.23]` | 线速度0.8m/s适合室内配送场景 |
| EKF参数 | 不变 | 20Hz融合轮式里程计+IMU角速度，配置合理 |

## 6. 安全边界对比

```
改前 (robot_radius 0.35 + inflation 0.1 + cost_scaling 2.5):
┌─────────────────────────────────────────┐
│ 前方余量: 37cm (多余)                     │
│ 后方余量: 2.7cm (不够! 车尾几乎裸露)       │
│ 后角余量: -4.1cm (超出! costmap不知道存在) │
│ 代价梯度: 45cm内 致命→自由 (陡峭)          │
└─────────────────────────────────────────┘

改后 (footprint + inflation 0.3 + cost_scaling 1.0):
┌─────────────────────────────────────────┐
│ 所有方向: 车体外扩30cm均匀缓冲             │
│ 前方总半径: 0.38m                        │
│ 后方总半径: 0.72m (车尾不再裸露!)         │
│ 代价梯度: 30cm内 致命→自由 (平缓)          │
└─────────────────────────────────────────┘
```

## 7. 与需求文档对照

| 需求条款 | 对应的修改 |
|---|---|
| 2.2 导航精度≤10cm | inflation 0.3 > 10cm×2，安全余量充足 |
| 2.2 弯道定位误差≤10cm | footprint多边形精确建模，后角不再裸露 |
| 2.3 防卡死/无倒退 | DUBIN运动模型 + replanning_and_recovery BT |
| 2.3 2分钟自恢复 | BT主动周期性重规划，不等堵死才触发 |
| 4 精度越优分越高 | 代价梯度平缓→控制器输出稳定→定位轨迹更平滑 |

## 8. 待验证

- [ ] 实地测试：速度是否平滑、拐弯是否自然
- [ ] 窄通道/拐角：是否会卡死（需求2.3）
- [ ] 若速度仍不平滑：切 `feedback: "CLOSED_LOOP"` 再测
- [ ] 若窄通道无法通过：渐进回调 `inflation_radius`（0.3→0.25→0.2）直到找到平衡点
- [ ] 若绕行过于保守：降低 `cost_scaling_factor` 或略微增大 `max_velocity`
- [ ] 充电桩对接精度测试（需求2.2目标点位≤10cm）

## 9. 回滚方法

```bash
ssh expressone@192.168.8.30
cd ~/ros2_ws/src/exp2_nav/config/
cp nav2_params.yaml.bak nav2_params.yaml
cd ~/ros2_ws && colcon build --packages-select exp2_nav
```
