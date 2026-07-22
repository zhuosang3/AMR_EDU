ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose:
  header:
    frame_id: map
  pose:
    position:
      x: 0.4401707193604403
      y: 22.21975313396997
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7557842626792065
      w: 0.6548206993417724"
