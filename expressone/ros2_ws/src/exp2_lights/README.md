# EXP2 小车灯光控制 v2.0

## 变更说明 (v1.0 → v2.0)

### 移除
- 移除 `MAX_LIGHTS_ON=4` 的同时点亮数量限制

### 新增
- **方位互斥**: 每个方位(左前/右前/左后/右后)同时只能点亮一种颜色
- `light_on()` 会自动关断同方位的其他颜色，保证硬件级互斥

### 设计原理
4 个方位各有 3 个 GPIO 对应红/绿/黄三种颜色。
同一盏物理灯不可能同时显示两种颜色，因此通过软件互斥来保证:
- 左前红(17)亮 → `light_on(27)` 左前绿 → 红自动灭，绿亮

## 文件结构

```
exp2_lights_new/
├── lights.h              # 头文件
├── lights.c              # GPIO 驱动 (含方位互斥)
├── test_lights.c         # 测试程序 (含互斥验证)
├── Makefile              # 独立编译
├── CMakeLists.txt        # ROS2 编译
└── package.xml           # ROS2 包信息

exp2_lights_bridge_new/
├── src/lights_bridge.cpp     # ROS2 桥接节点
├── include/lights_client.hpp # C++ 客户端头文件
├── lights_client.py          # Python 客户端
├── CMakeLists.txt            # ROS2 编译
└── package.xml               # ROS2 包信息
```

## 编译 & 测试

### 独立编译 (不依赖 ROS2)
```bash
cd exp2_lights_new
make
sudo ./test_lights
```

### ROS2 编译
```bash
# 先编译 lights 库
cd exp2_lights_new
colcon build --packages-select exp2_lights

# 再编译 bridge
cd ../exp2_lights_bridge_new
colcon build --packages-select exp2_lights_bridge
```
