#!/usr/bin/env zsh

# 设置 AMCL 初始位姿

# 用法: ./set_initial_pose.zsh


source /opt/ros/jazzy/setup.zsh


ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '

{

  header: {frame_id: "map"},

  pose: {

    pose: {

      position: {x: 1.7675333107856783, y: 3.187751023496645, z: 0.0},

      orientation: {x: 0.0, y: 0.0, z: 0.7382094598366429, w: 0.6745715628513346}

    },

    covariance: [

      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,

      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,

      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467

    ]

  }

}'
