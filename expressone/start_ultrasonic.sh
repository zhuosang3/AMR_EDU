#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/expressone/ros2_ws/install/setup.bash
exec ros2 run dyp_a21_ultrasonic_cpp ultrasonic_node --ros-args --params-file /home/expressone/ros2_ws/install/dyp_a21_ultrasonic_cpp/share/dyp_a21_ultrasonic_cpp/config/params.yaml
