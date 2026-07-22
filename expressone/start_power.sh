#!/bin/bash
# EXP2 电池监控启动脚本 (power_monitor)
source /opt/ros/jazzy/setup.bash
source /home/expressone/ros2_ws/install/setup.bash
exec /home/expressone/ros2_ws/install/power_monitor/lib/power_monitor/power_monitor_node \
    --ros-args \
    -p can_interface:=can0 \
    -p mqtt_host:=localhost \
    -p mqtt_port:=1883 \
    -p mqtt_topic:=/car/power \
    -p publish_interval_s:=5
