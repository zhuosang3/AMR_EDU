#!/bin/bash
# 干净地停止 Nav2 mapping 所有进程
source /opt/ros/jazzy/setup.bash

echo "=== 通过 lifecycle manager 正常 shutdown ==="
NAMES=(
  "lifecycle_manager_slam"
  "lifecycle_manager_navigation"
)

for mgr in "${NAMES[@]}"; do
  if ros2 node list 2>/dev/null | grep -q "$mgr"; then
    echo "→ Shutdown $mgr ..."
    ros2 lifecycle set /$mgr shutdown 2>/dev/null
    sleep 0.5
  fi
done

echo "=== 强制清理残留进程 ==="
PNAMES=(
  async_slam_toolbox_node sync_slam_toolbox_node slam_toolbox
  map_saver_server map_saver
  controller_server planner_server smoother_server
  behavior_server bt_navigator waypoint_follower
  velocity_smoother collision_monitor lifecycle_manager
  route_server docking_server rviz2
)

for p in "${PNAMES[@]}"; do
  pkill -9 -f "$p" 2>/dev/null && echo "  killed: $p"
done

echo "=== 清理 ROS2 daemon（清空 DDS 缓存） ==="
ros2 daemon stop 2>/dev/null
sleep 1
echo "  ros2 daemon stopped"

echo "=== 检查残留 ==="
sleep 2
REMAIN=$(ros2 node list 2>/dev/null | grep -v -E "canopen_ros2|keli_lidar|imu_bridge|robot_state_publisher|teleop")
if [ -n "$REMAIN" ]; then
  echo "以下节点可能还有残留:"
  echo "$REMAIN"
else
  echo "  OK 已清理干净，仅保留驱动层节点"
fi

echo ""
echo "现在可以重新启动（daemon 会自动重启）:"
echo "  ros2 launch exp2_nav nav.launch.py slam:=True"
