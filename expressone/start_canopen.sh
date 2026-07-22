#!/bin/bash
source /opt/ros/jazzy/setup.bash
exec /home/expressone/ros2_ws/src/canopen_ros2/target/release/canopen_ros2 --publish-tf false
