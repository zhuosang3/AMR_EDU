# 重定位管理器完整文档

> 项目: EXP2 无人小车 AMR 导航系统
> 需求: 2.1 多模式自主重定位
> 平台: Orange Pi 5 Plus / ROS2 Jazzy / Nav2
> 部署路径: `~/ros2_ws/src/exp2_nav/`（ament_cmake symlink-install，改 src 即生效）
> 演进: 2026-07-14 初始 → 07-16 实测改造 → 07-18 360°旋转 → 07-22 刹车修复 → 07-23 opencv/bbs → **07-24 mh 多假设（当前默认）+ 删 opencv/bbs**
> Git 状态: ⚠️ **所有重定位改动仍未提交**（working directory 未暂存/未跟踪）

---

## 0. TL;DR（当前状态）

- **默认模式 = mh（多假设）**：rviz 点选 → 发协方差 initialpose → **不动车**，AMCL 静止 1~5 秒收敛。
- **opencv / bbs 两套单帧匹配方案已删代码**（实测本质多解，不可靠）。现只剩 **mh（默认）+ rotate（v1 360°旋转兜底）**。
- **mh 实测成功率 1/3~1/2**，点得越准越高；已做"收紧协方差"优化（cov_xy 2.0→0.5），**进一步优化留待以后**。
- 日常用法：开机 → `./nav.zsh` → rviz `2D Pose Estimate` 点选 → mh 自动定位。

---

## 1. 日常使用流程（怎么用）

```
1. 小车开机
   → 驱动自动起来(keli_lidar 激光 / IMU / CAN 等, 开机自启, 不用管)

2. 终端执行  ./nav.zsh        # 内容 = ros2 launch exp2_nav navigation.launch.py
   → 起完整 nav2 栈: amcl + map_server + controller/planner/... + reloc_manager(自动 mh 模式)
   → 此时 AMCL 初始化在地图原点 (0,0,0), 还没定位

3. 打开 rviz2 → 工具栏 "2D Pose Estimate"(快捷键 P)
   → 在小车真实位置点一下, 按住拖出车头朝向

4. mh 自动定位: 车不动, 1~5 秒粒子云收紧 → /reloc/status=CONVERGED → 定位完成
   (点的位置/方向越准成功率越高; 偶尔锁错就重点一次)

5. 之后用 "2D Goal Pose" 发导航目标即可
```

> 切换模式：改 `config/nav2_params.yaml` 的 `reloc_mode: mh`（或 `rotate`），重启 `./nav.zsh`。

---

## 2. 两种模式

状态机: `UNLOCALIZED → LOCALIZING → CONVERGED`（mh/rotate 共用）

| 模式 | 触发 | initialpose 协方差 | 旋转 | 原理 | 状态 |
|---|---|---|---|---|---|
| **mh**（默认） | rviz 2D Pose Estimate | cov_xy=0.5, cov_yaw=0.3（点击为中心） | ❌ 不动车 | 宽/中协方差覆盖多模态 + 动态降 AMCL 运动门限 → 静止 sensor update 收敛 | 当前默认，成功率 1/3~1/2 |
| **rotate**（v1 兜底） | 同上（reloc_mode=rotate 时） | cov_xx/yy=1.0, cov_yaw=0.5 | ✅ 360° 单向（~33s）+ 减速斜坡 | 旋转提供大量 scan matching 机会，AMCL 必收敛 | 最稳但慢、动车 |

> 充电桩标定（`/reloc/dock_calib` 服务，cov=0.01 直接 CONVERGED）和手动设点（`/reloc/set_manual_pose`）两种辅助方式两种模式下都可用。

---

## 3. mh 算法（核心）+ 为什么这么设计

### 3.1 关键诊断（2026-07-24，推翻"打分函数有问题"）

用真实数据（AMCL CONVERGED 作真值 + 带 46% 箱子杂物的真实 /scan）穷尽分析：

- **真值处墙是对得上的**（中位偏 4.8cm，63% 点贴墙 <0.1m）→ 地图在该点没坏。
- 但**单帧扫描在杂物多 + 结构重复的办公室本质多解**：真值处打分地形下，1.4m 和 3.7m 外的假吸引子比真值还贴。
- 穷举 **trimmed-mean / 内点计数 / 似然场求和** 三种打分，**没有一种把真值稳定顶到第一**（似然场下真值连前 8 都进不去）。
- **结论：单帧匹配欠定，调 trim/打分无效。** bbs 取单最优解 + 紧协方差(0.1)立即判 CONVERGED → 选错就锁死 = "偏差很大"根因。

### 3.2 mh 思路

不盲信单帧 matcher 单解：

1. `scan_matcher.match_topn()` 跑 top-N 多解**仅记日志**（让多解可见，定位决策不靠它）。
2. 发**协方差 initialpose**（中心=点击）覆盖窗口内真值 + 假吸引子多模态。
3. **不旋转**，**动态把 AMCL 运动门限 `update_min_d/a` 临时设 0** → 静止也每帧 sensor update。
4. AMCL 粒子滤波收敛到真值模态 → 协方差 <0.1 → `CONVERGED`。
5. 收敛/超时后**恢复 AMCL 门限**（0.25/0.1），避免全局 0 的 CPU + 粒子过度收敛突刺。

### 3.3 关键参数（`nav2_params.yaml` reloc_manager 段，可调）

| 参数 | 值 | 含义 |
|---|---|---|
| `reloc_mode` | `mh` | 选 mh / rotate |
| `mh_cov_xy` | 0.5（σ≈0.7m） | initialpose xy 协方差；点偏多→调大，还锁错→调小 |
| `mh_cov_yyaw` | 0.3（σ≈31°） | yaw 协方差；方向点不准→调大，"稍微转了一点"多→调小 |
| `mh_topk` | 5 | top-N 多解诊断候选数 |
| `mh_window_m / mh_coarse_step_m / mh_coarse_yaw_step_deg / mh_trim_frac` | 2.0/1.0/10/0.85 | 诊断 matcher 粗搜参数（不决定位姿） |
| `scan_map_yaml / scan_cap_m` | my_map.yaml / 8.0 | 诊断 matcher 用的地图 + 渲染半径上限 |

---

## 4. 架构

```
reloc_manager（~456行 Python，mh/rotate 双模式）
    │
    ├─→ /initialpose（PoseWithCovarianceStamped）──→ AMCL ←── /scan（keli_lidar 激光 27Hz）
    │     [mh: 点击为中心 cov_xy/yaw; rotate: cov 1.0]        ←── /odometry/filtered（EKF 20Hz）
    │                                                          ←── /map
    ├─← /amcl_pose（监控 cov[0]/cov[7] <0.1 → CONVERGED）
    ├─→ /cmd_vel_smoothed（Twist，rotate 模式旋转时发；collision_monitor 输入）
    ├─→ AMCL 参数（AsyncParameterClient 动态调 update_min_d/a）  ← mh 专用
    ├─→ /reloc/status（1Hz）
    ├─← /initialpose（rviz 2D Pose Estimate 在此触发）
    ├─← /reloc/set_manual_pose / /reloc/dock_calib
    └─ scan_matcher（~150行，纯计算）match_topn() → mh 诊断用

TF: map → odom（AMCL）→ base_link（EKF）→ laser / imu_link（static）
```

**rotate 模式发 `/cmd_vel_smoothed` 而非 `/cmd_vel`**：collision_monitor 空闲时往 `/cmd_vel` 发刹车 0 会盖掉旋转指令；发它的输入 `/cmd_vel_smoothed`，原地旋转（线速度=0）穿得过 collision_monitor。

---

## 5. 代码级实现

### 5.1 `reloc_manager.py`（~456 行）

- **mh 模式**：`_handle_mh_click` → `_amcl_set_gate(0,0)` 降门限 → `publish_initial_pose(cov=mh_*)` → `_enter_localizing(do_rotation=False)` → `_amcl_pose_cb` 收敛 → `_amcl_restore_gate()`。
  - 动态门限：`AsyncParameterClient(self, 'amcl')`（⚠️ jazzy 是单数 `AsyncParameterClient`，非 humble 的复数）。`_amcl_set_gate` 设 `update_min_d/a`；`_amcl_restore_gate` 在 CONVERGED / 超时 / dock 标定时恢复 0.25/0.1。
- **rotate 模式**：`_initial_pose_cb` else 分支 → cov 1.0 + `_enter_localizing()` → `_start_rotation` → `_rotation_step`（里程计累计控角 + Phase2 减速斜坡）。
- **去抖** `DEBOUNCE_SEC=0.5`：防 rviz 连发 + 自回响。
- **诊断**：mh 跑 `match_topn` 打 top-5 多解日志；第 1/2 解 score 接近时 WARN。

### 5.2 `scan_matcher.py`（~150 行，纯计算无 ROS 依赖）

仅剩 `match_topn()`：点击 ±window 全 yaw 粗搜（距离场 + trimmed mean），返回空间互异(>0.5m)的 top-N 候选。
- 距离场：`cv2.distanceTransform`（每像素到最近墙距离）；打分 = 候选位姿下扫描点到墙的 trimmed 均距。
- trim_frac=0.85：取贴合最好 85%（扔动态杂物离群）。
- CLI 自测：`python3 scan_matcher.py /tmp/scan.json clickx clicky [truex truey]`。

### 5.3 `nav2_params.yaml`

`reloc_manager` 段：`reloc_mode: mh` + `mh_*` + `scan_*`（见 §3.3）。
`amcl` 段关键：`max_particles:5000` `laser_model_type:likelihood_field` `update_min_d:0.25` `update_min_a:0.1`（mh 收敛期临时设 0）`recovery_alpha_fast:0.1` `set_initial_pose:true`(0,0,0)。

### 5.4 `navigation.launch.py`

`GroupAction([localization_launch, navigation_launch, reloc_manager])`，reloc_manager `parameters=[params_file]`。`./nav.zsh` = 此 launch。

---

## 6. 全量演进

| 会话 | 日期 | 内容 |
|---|---|---|
| 1 | 07-14 | 从零搭建三模式（rviz点选/充电桩/手动），启动 3s 大协方差撒全图。编译过待测。 |
| 2 | 07-16 | 实测大协方差+静止卡死(cov 10→5.94)。改 rviz 点选触发 + 小协方差 + ±60°摆动。 |
| 3 | 07-18 | ±60° 对点选精度要求太高。改 360° 单向 + cov 1.0 + 里程计反馈控角 + 收敛后转完一圈。 |
| 4 | 07-22 | 急刹车致粒子云偏移~10°。改 500ms 减速斜坡 + 300ms 零速保持。 |
| 5 | 07-23 | 调研不动车方案。实现 **opencv**（激光→图→模板匹配，虚拟旋转）+ **bbs**（全 yaw 两级粗到细 trimmed 匹配）。两者实测均**偏差大/不收敛**。 |
| 6 | 07-24 | **诊断证明单帧匹配本质多解**（§3.1），调打分无效。实现 **mh 多假设**（宽协方差 + AMCL 收敛 + 动态运动门限），在线验证静止收敛。 |
| 7 | 07-24 | 用户实测 mh 成功率 1/3~1/2。**删 opencv/bbs 全部代码**，mh 复用参数改名 mh_*/scan_*。 |
| 8 | 07-24 | 优化 A：**收紧协方差** cov_xy 2.0→0.5、cov_yyaw 2.0→0.3。验证仍收敛。进一步优化（收敛校验/小旋转兜底）留待以后。 |

---

## 7. 已知限制与待做

1. **mh 成功率 1/3~1/2**（用户实测 ~10 次）：宽/中协方差仍让假吸引子模态参与竞争，AMCL 静止收敛偶发锁错。**优化方向（待做）**：
   - B：收敛后校验（pos 离点击太远 / matcher 评分差 → 判失败重点，防"自信报错"）。
   - C：可疑时小幅旋转（60~90°，非 360°）给 AMCL parallax 消歧（最鲁棒但动车）。
   - 继续调 `mh_cov_xy/yyaw`。
2. **rotate 模式独立兜底未实测 cmd_vel_smoothed 修复**（会动车，留用户）。
3. **launch docstring 过时**（仍提"3s 全局定位/PM2"）。
4. **⚠️ 代码未提交 git**：所有改动仅在 working directory，无版本控制保护。建议尽早 commit。
5. **2m 窗口远点击**：mh 诊断 matcher 在 ±2m 内搜；用户保证点击 2m 内（远点击会搜错区域）。
6. **stale .bak 备份**：`reloc_manager.py.bak/.bak.20260723/.bak.bbs`、`scan_matcher.py.bak.bbs` 留在 scripts/，可清理。

---

## 8. 诊断命令速查

```bash
# 重定位状态（1Hz）
ros2 topic echo --once /reloc/status

# AMCL 协方差（<0.1 = CONVERGED）
ros2 topic echo --once /amcl_pose --field pose.covariance

# reloc_manager 日志（看 mh top-5 多解诊断 + Converged）
tail -f /tmp/reloc_*.log        # standalone 测试时；./nav.zsh 时看终端

# AMCL 运动门限（mh 收敛期应=0，平时 0.25/0.1）
ros2 param get /amcl update_min_d; ros2 param get /amcl update_min_a

# 确认单实例无残留（曾出现双 launch 冲突）
ros2 node list | grep -E 'amcl|reloc'
ps aux | grep navigation.launch.py | grep -v grep

# 激光
ros2 topic hz /scan

# ROS env（/tmp/rosenv.sh 重启会清空，改用）
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash
```

诊断脚本（机器人 `/tmp/`，可复用）：`grab.py`（抓真值+scan）、`repro.py`、`landscape.py/2/3`（打分地形/内点/似然场）、`test_mh.py`（点击触发 mh + 协方差监听）。

---

## 9. 文件清单

```
~/ros2_ws/src/exp2_nav/
├── scripts/
│   ├── reloc_manager.py          ← ~456行，mh/rotate 双模式（未跟踪）
│   ├── scan_matcher.py           ← ~150行，match_topn 诊断（未跟踪）
│   └── *.bak*                    ← 旧备份（可清理）
├── config/
│   ├── nav2_params.yaml          ← reloc_mode: mh + mh_*/scan_* + amcl 段（已改未提交）
│   └── ...
├── launch/navigation.launch.py   ← GroupAction 含 reloc_manager（已改未提交）
├── CMakeLists.txt / package.xml  ← install scripts + 依赖（已改未提交）

~/ (home)
├── nav.zsh                       ← = ros2 launch exp2_nav navigation.launch.py
├── reloc_dock.zsh / reloc_manual.zsh / reloc_status.zsh   ← 辅助脚本
└── maps/my_map.{pgm,yaml}        ← 地图（381KB, 0.05m/px）
```
