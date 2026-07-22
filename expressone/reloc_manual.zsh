#!/usr/bin/env zsh
source /opt/ros/jazzy/setup.zsh
X=${1:-0.0}
Y=${2:-0.0}
YAW_DEG=${3:-0}
YAW=$(python3 -c "import math; print(math.radians($YAW_DEG))")

echo "手动设点: ($X, $Y, ${YAW_DEG}deg)"
ros2 topic pub --once /reloc/set_manual_pose geometry_msgs/msg/PoseWithCovarianceStamped "
{
  header: {frame_id: \"map\"},
  pose: {
    pose: {
      position: {x: $X, y: $Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $(python3 -c "import math; print(math.sin($YAW/2))"), w: $(python3 -c "import math; print(math.cos($YAW/2))")}
    },
    covariance: [0.15,0,0,0,0,0, 0,0.15,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.05]
  }
}"
echo "已发送手动位姿"
