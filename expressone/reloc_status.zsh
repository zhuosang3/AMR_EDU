#!/usr/bin/env zsh
source /opt/ros/jazzy/setup.zsh
echo "重定位状态:"
ros2 topic echo --once /reloc/status 2>/dev/null || echo "无数据（reloc_manager 未运行？）"
