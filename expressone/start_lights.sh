#!/bin/bash
# EXP2 灯光桥接启动脚本
# GPIO 已通过 chmod 666 开放，无需 sudo

source /opt/ros/jazzy/setup.bash
source /home/expressone/ros2_ws/install/setup.bash 2>/dev/null

exec /home/expressone/ros2_ws/install/exp2_lights_bridge/lib/exp2_lights_bridge/lights_bridge
