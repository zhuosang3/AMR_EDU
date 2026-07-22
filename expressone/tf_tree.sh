#!/bin/bash
cd /opt/ros/jazzy && source setup.bash
cd ~/ros2_ws && source install/setup.bash
cd ~
rqt -s rqt_tf_tree
