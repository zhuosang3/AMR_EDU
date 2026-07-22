# EXP2 小车定位漂移问题诊断与修复

**日期**: 2026-06-23
**设备**: Orange Pi 5 Plus (192.168.8.30)
**作者**: Claude Code 诊断，luzhuo 确认

---

## 问题现象

在香橙派上启动导航栈后，RViz 中小车定位胡乱飘动，`map→odom` 变换在 10-30 米范围内随机跳跃。

### 启动流程

```bash
# 终端1
./nav.zsh                          # ros2 launch exp2_nav navigation.launch.py

# 终端2
./set_initial_pose.zsh             # 发布初始位姿到 /initialpose

# 终端3
rviz2 -d ~/exp2.rviz
```

---

## 诊断过程

### 1. 检查节点和话题

所有节点正常运行：`/amcl`, `/ekf_filter_node`, `/canopen_ros2`, `/keli_lidar`, `/hi12_imu` 等。

### 2. 检查 TF 树

TF 树结构正确：`map → odom → base_link → (imu_link, laser)`

### 3. 发现关键异常

连续采样 `/tf` 中 `odom→base_link` 变换，发现**值在两个来源之间交替跳变**：

| 采样 | `odom→base_link` 位置 | 来源 |
|------|----------------------|------|
| 1 | `(-1.79, -6.99)` | ekf_filter_node |
| 2 | `(-1.79, -6.99)` | ekf_filter_node |
| 3 | `(-0.09, 0.02)` | **canopen_ros2** |
| 4 | `(-1.79, -6.99)` | ekf_filter_node |
| 5 | `(-0.09, 0.02)` | **canopen_ros2** |

### 4. 确认 TF 发布者

`/tf` 话题有 **4 个发布者**：

| 发布者 | 发布内容 | 是否正确 |
|--------|---------|---------|
| `robot_state_publisher` | `base_link→imu_link`, `base_link→laser` (static) | ✓ |
| `amcl` | `map→odom` | ✓ |
| `ekf_filter_node` | `odom→base_link` | ✓ |
| **`canopen_ros2`** | **`odom→base_link`** | **✗ 冲突！** |

### 5. 确认 canopen_ros2 启动参数

```bash
$ cat /proc/1645/cmdline | tr "\0" " "
/home/expressone/ros2_ws/src/canopen_ros2/target/release/canopen_ros2
```

**缺少 `--publish-tf false` 参数**！而 `ekf.yaml` 中早有警告：

> ⚠️ canopen_ros2 必须以 --publish-tf false 启动, 否则双 TF 打架!

### 6. 确认 AMCL 崩溃

由于 `odom→base_link` 在两个值之间跳变（差值约 7 米），AMCL 粒子滤波认为机器人在瞬移，导致 `map→odom` 变换大幅跳动：

| 采样 | `map→odom` | 与上次的偏移 |
|------|-----------|------------|
| 1 | `(-5.39, 11.14)` | — |
| 2 | `(1.69, 14.67)` | 8m |
| 3 | `(-0.20, -1.47)` | 16m |
| 4 | `(-6.40, 25.53)` | 27m |
| 5 | `(2.60, 8.85)` | 19m |

**RViz 中小车模型就是跟着这个 `map→odom` 变换在跳。**

---

## 根因

`canopen_ros2` 和 `ekf_filter_node` **同时发布 `odom→base_link` 到 `/tf`**。

`tf2` 对同一 `parent→child` 允许多个发布者，不做冲突检测。订阅方读到的是最新到达的那条消息，于是 `odom→base_link` 在两个值之间交替跳变。AMCL 依赖这个变换计算机器人运动增量，看到瞬移后粒子滤波崩溃。

---

## 修复

### 步骤 1：停掉 canopen_ros2，用 `--publish-tf false` 重启

```bash
# 停掉旧的 PM2 进程（可能有多条，包括名称打错的 canipen_ros2）
pm2 stop canopen_ros2
pm2 delete canopen_ros2
pm2 stop canipen_ros2
pm2 delete canipen_ros2

# 创建启动脚本（确保 ROS2 环境正确加载）
cat > ~/start_canopen.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
exec /home/expressone/ros2_ws/src/canopen_ros2/target/release/canopen_ros2 --publish-tf false
EOF
chmod +x ~/start_canopen.sh

# 用 PM2 启动
pm2 start ~/start_canopen.sh --name canopen_ros2
pm2 save
```

### 步骤 2：重启 EKF 清零协方差

```bash
pm2 restart ekf
```

### 步骤 3：重启导航栈

```bash
# 杀掉所有 nav2 节点后重新启动
./nav.zsh
./set_initial_pose.zsh
```

### 修复后验证

- `/tf` 只有 **3 个发布者**（`canopen_ros2` 不再发布 TF）
- `odom→base_link` 只有一个来源（EKF），不再跳变
- EKF 位置/协方差从零开始，正常
- AMCL 粒子滤波正常收敛

---

## 附：EKF 协方差爆炸分析

修复前 EKF 位置协方差高达 **1.6 万亿**，这不是 bug，而是配置决定的正常行为。

### 原因

EKF 配置只融合速度（vx, vyaw），不融合绝对位置：

```yaml
odom0_config: [false, false, false,   # x, y, z → 不融合
               false, false, false,   # roll, pitch, yaw → 不融合
               true,  false, false,   # vx ✓
               false, false, true,    # vyaw ✓
               false, false, false]

imu0_config:  [false, false, false,   # 姿态 → 不融合
               false, false, false,
               false, false, false,
               false, false, true,    # vyaw ✓
               false, false, false]
```

位置完全由速度积分得到（`位置 = ∫速度 dt`）。卡尔曼滤波的预测步每一轮都会在位置协方差上叠加过程噪声：

```
P_pos(t+1) = P_pos(t) + Δt² × P_vel + Q
```

没有绝对位置观测来压缩协方差，所以运行越久，协方差越大。当时 EKF 已运行 20+ 分钟。

### 为什么这不影响 AMCL

Nav2 的 AMCL 使用 `odom→base_link` 的**帧间增量**（前后两帧的差值）来驱动机器人运动模型，而不是绝对位置。只要速度估计准确，增量就准确，AMCL 就能正常工作。

真正导致问题的是 **TF 冲突**——AMCL 在不同来源的 `odom→base_link` 之间交替读取，看到了虚假的"瞬移"。

### 如果想彻底消除协方差增长

可以在 EKF 中融合 IMU 姿态（如果磁力计可靠）或周期性注入绝对位置参考。但对当前系统而言**不是必需的**——TF 冲突修复后，定位已经稳定。

---

## 受影响文件

| 文件 | 变更 |
|------|------|
| `~/start_canopen.sh` | **新增** - canopen_ros2 启动包装脚本 |
| `~/.pm2/dump.pm2` | **已保存** - PM2 进程列表 |
| `ekf.yaml` | 未修改（协方差增长是预期行为） |
| `nav2_params.yaml` | 未修改 |

---

## 关键经验

1. **TF 冲突不会报错**：tf2 允许多个发布者发布同一 `parent→child`，不警告不报错
2. **AMCL 对里程计跳变极其敏感**：瞬间的位姿跳变会导致粒子滤波发散
3. **EKF 的协方差爆炸是配置决定的**：只融合速度不融合位置，协方差必然单调增长
4. **查看 TF 发布者**：`ros2 topic info /tf -v` 可以看到所有发布者

---

# 续：rviz2 CPU 占用 375% 问题（GPU 渲染修复）

**日期**: 2026-07-01
**关联**: 上述 TF 冲突修复后，定位已稳定，但发现 rviz2 自身 CPU 异常高

---

## 问题现象

TF 冲突修复后，启动 nav.zsh → set_initial_pose.zsh → rviz2，`top` 显示 rviz2 CPU **375~389%**，系统负载 11+。

**与上次问题的区别**：上次是 TF 冲突导致定位漂移，这次是 rviz2 渲染本身消耗 CPU，定位本身正常。

---

## 诊断过程

### 1. 排除 TF 冲突复发

确认 `canopen_ros2` 仍以 `--publish-tf false` 运行，`/tf` 只有 3 个发布者，`odom→base_link` 无冲突。

### 2. 确认渲染后端

rviz2 日志：`OpenGl version: 4.5 (GLSL 4.5)`

**对比验证**：

| 命令 | 渲染器 | 加速 |
|------|--------|------|
| `DISPLAY=:1 glxinfo -B` | **llvmpipe** (LLVM) | ❌ no |
| `EGL_PLATFORM=x11 eglinfo` | **Mali-G610 (Panfrost)** | ✅ yes |

GLX 路径走 llvmpipe CPU 软渲染，EGL 路径能到 GPU。rviz2 硬编码 GLX，无法使用 EGL。

### 3. 确认根因链

```
rviz2 → GLX → XWayland → DRI3 转发不通 → llvmpipe (CPU) → 375%
                          存在但未使用:
rviz2 → EGL → panfrost_dri.so → panthor → Mali G610 (GPU) ✅
```

- rviz2 代码 `render_system.hpp` 硬编码 `#include <GL/glx.h>`，Linux 下只用 GLX
- ROS2 Jazzy 预编译 OGRE 模块未开启 EGL 支持
- GLX 经 XWayland 转发后，DRI3 与 panfrost 驱动不兼容，回落软渲染
- 原生 X11 下 GLX→DRI3→panfrost 链路正常

---

## 修复

### 将 GNOME 桌面从 Wayland 切换到原生 X11

```bash
# 编辑 /etc/gdm3/custom.conf，追加:
WaylandEnable=false

# 重启
sudo reboot
```

### 量产脚本

脚本已部署至 `~/enable-x11-gpu.sh`：

```bash
sudo bash ~/enable-x11-gpu.sh         # 仅修改配置
sudo bash ~/enable-x11-gpu.sh --reboot  # 修改并重启
```

幂等安全，可重复执行。

---

## 修复后验证

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| OpenGL 渲染器 | llvmpipe (CPU) | **Mali-G610 (Panfrost)** |
| 硬件加速 | ❌ | ✅ **yes** |
| rviz2 CPU | 375% | **12.3%** |
| rviz2 内存 | 264 MB | 189 MB |
| 系统负载 | 11.79 | 正常 |

传感器节点、导航栈（AMCL / EKF / 路径规划）不受影响，CPU 释放后可更好服务导航算法。

---

## 受影响文件

| 文件 | 变更 |
|------|------|
| `/etc/gdm3/custom.conf` | **新增** `WaylandEnable=false` |
| `~/enable-x11-gpu.sh` | **新增** - 量产用配置脚本 |
