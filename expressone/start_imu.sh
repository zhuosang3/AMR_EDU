#!/bin/bash
# EXP2 IMU 驱动启动脚本 (hi12_imu)

source /opt/ros/jazzy/setup.bash
source /home/expressone/ros2_ws/install/setup.bash

exec /home/expressone/ros2_ws/install/hi12_imu/lib/hi12_imu/hi12_imu_node \
    --ros-args -p device:=/dev/hi12_imu -p baudrate:=115200 -p frame_id:=imu_link -p publish_hz:=100
