#!/bin/bash
# robot_localization EKF — PM2 启动包装
# 融合 /odom (vx, vyaw) + /imu (yaw 角速度) → /odometry/filtered + TF odom→base_link
# 前提: canopen_ros2 以 --publish-tf false 运行 (TF 由本节点独家广播)
source /opt/ros/jazzy/setup.bash
exec /opt/ros/jazzy/lib/robot_localization/ekf_node \
  --ros-args \
  -r __node:=ekf_filter_node \
  --params-file /home/expressone/ros2_ws/src/exp2_nav/config/ekf.yaml
