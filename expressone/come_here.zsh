#!/usr/bin/env zsh
# 输出一条 ros2 action send_goal 命令，把当前位置+朝向作为导航目标
# 用法：eval "$(./come_here.zsh)"

source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

out=$(timeout 8 python3 -c "
import rclpy, sys
rclpy.init()
node = rclpy.create_node('come_here')
from tf2_ros import Buffer, TransformListener
buf = Buffer(node=node)
tl = TransformListener(buf, node)

for _ in range(160):
  try:
    t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
    print(f'{t.transform.translation.x} {t.transform.translation.y} {t.transform.rotation.z} {t.transform.rotation.w}')
    sys.exit(0)
  except Exception:
    rclpy.spin_once(node, timeout_sec=0.05)
print('NO_TF')
" 2>/dev/null)

read -r x y qz qw <<< "$out"

if [[ "$out" = "NO_TF" || -z "$x" ]]; then
  echo "echo '⚠️  AMCL 未定位——请先在 RViz 设 2D Pose Estimate，或确认 navigation.launch.py 在运行'" >&2
  exit 1
fi

cat <<CMD
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose:
  header:
    frame_id: map
  pose:
    position:
      x: $x
      y: $y
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: $qz
      w: $qw"
CMD
