#!/usr/bin/env zsh
# ═══════════════════════════════════════════════════════════════
# checktf.zsh — 检查 TF 树完整性，通过后启动 navigation.launch.py
#
# 预检链路（启动前必须存在，由底层驱动提供）：
#   odom → base_link          ← canopen_ros2（里程计）
#   base_link → laser         ← robot_state_publisher + URDF
#   base_link → imu_link      ← robot_state_publisher + URDF
#
# 清理：自动 kill 旧的 Nav2 进程（防止节点名冲突）
#
# 启动后自动验证（由 AMCL 补充）：
#   map → odom
# ═══════════════════════════════════════════════════════════════

set -e

SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

# ── Colors ──────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ── Source ROS ──────────────────────────────────────────────────
# ROS setup.bash 在 zsh 里用 cd 绕开 BASH_SOURCE 相对路径问题
cd /opt/ros/jazzy && source setup.bash && cd "$SCRIPT_DIR"
cd ~/ros2_ws && source install/setup.bash 2>/dev/null && cd "$SCRIPT_DIR"

echo -e "${CYAN}══════════════════════════════════════════════${NC}"
echo -e "${CYAN}  checktf — TF 树完整性检查${NC}"
echo -e "${CYAN}══════════════════════════════════════════════${NC}"
echo ""

# ── 1. 检查关键进程 ────────────────────────────────────────────
echo -e "${YELLOW}[1/4] 检查底层进程...${NC}"
all_ok=true

critical_procs=(
  "hls_canopen: hls_canopen (CAN 驱动)"
  "canopen_ros2: canopen_ros2 (里程计/odom)"
  "keli_lidar: keli_lidar (雷达/scan)"
  "robot_state_publisher: robot_state_publisher (URDF)"
)

for entry in "${critical_procs[@]}"; do
  proc="${entry%%:*}"
  desc="${entry##*: }"

  if pgrep -x "$proc" >/dev/null 2>&1; then
    echo -e "  ${GREEN}✅${NC} $desc"
  elif pgrep -f "$proc" >/dev/null 2>&1; then
    echo -e "  ${YELLOW}⚠${NC} $desc（模糊匹配到）"
  else
    echo -e "  ${RED}❌${NC} $desc — 未运行"
    all_ok=false
  fi
done
echo ""

# ── 2. 检查 TF 树（Python tf2_ros，更可靠）─────────────────────
echo -e "${YELLOW}[2/4] 检查 TF 树（预检链路）...${NC}"

TF_RESULT=$(python3 << 'PYEOF'
import rclpy, time, sys

rclpy.init()
node = rclpy.create_node('tf_checker')
from tf2_ros import Buffer, TransformListener
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
# 手动 spin 2 秒填充 buffer
end = time.time() + 2.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)

checks = [
    ('odom',      'base_link', '里程计 — canopen_ros2'),
    ('base_link', 'laser',     '雷达 — robot_state_publisher'),
    ('base_link', 'imu_link',  'IMU — robot_state_publisher'),
]

lines = []
all_ok = True
for parent, child, label in checks:
    try:
        t = buffer.lookup_transform(parent, child, rclpy.time.Time())
        tr = t.transform.translation
        lines.append(f"PASS:{parent}→{child}:{label}:{tr.x:.3f},{tr.y:.3f},{tr.z:.3f}")
    except Exception as e:
        lines.append(f"FAIL:{parent}→{child}:{label}:{str(e)[:100]}")
        all_ok = False

rclpy.shutdown()
sys.exit(0 if all_ok else 1)
PYEOF
)
TF_EXIT=$?

# 解析并显示结果
while IFS= read -r line; do
  if [[ "$line" == PASS:* ]]; then
    rest="${line#PASS:}"
    tf="${rest%%:*}"
    rest="${rest#*:}"
    label="${rest%%:*}"
    xyz="${rest#*:}"
    echo -e "  ${GREEN}✅${NC} $tf"
    echo "    └─ $label"
    echo "    └─ Translation: [$xyz]"
  elif [[ "$line" == FAIL:* ]]; then
    rest="${line#FAIL:}"
    tf="${rest%%:*}"
    rest="${rest#*:}"
    label="${rest%%:*}"
    err="${rest#*:}"
    echo -e "  ${RED}❌${NC} $tf"
    echo "    └─ $label"
    echo "    └─ 原因: $err"
  fi
done <<< "$TF_RESULT"
echo ""

if [[ "$TF_EXIT" != 0 ]]; then
  echo -e "${RED}❌ TF 树不完整，请排查后重试。${NC}"
  echo "  可能的原因："
  echo "    - canopen_ros2 未正常发布 odom→base_link"
  echo "    - robot_state_publisher 未加载 URDF"
  echo "    - URDF 中缺少 laser/imu_link 帧"
  exit 1
fi

echo -e "${GREEN}✅ 预检链路全部正常！${NC}"
echo ""

# ── 3. 清理旧的 Nav2 残留进程 ──────────────────────────────────
echo -e "${YELLOW}[3/5] 清理旧 Nav2 进程...${NC}"
NAV2_DEAD=$(pkill -f "nav2_" 2>&1 || true)
NAV2_DEAD2=$(pkill -f "opennav_docking" 2>&1 || true)
sleep 1
# 验证清理结果
NAV2_LEFT=$(pgrep -f "nav2_|opennav_docking" 2>/dev/null | wc -l)
if [[ "$NAV2_LEFT" -gt 0 ]]; then
  echo -e "  ${YELLOW}⚠${NC} 仍有 $NAV2_LEFT 个残留进程，尝试强制清理..."
  pkill -9 -f "nav2_" 2>/dev/null || true
  pkill -9 -f "opennav_docking" 2>/dev/null || true
  sleep 1
fi
echo -e "  ${GREEN}✅${NC} Nav2 旧进程已清理"
echo ""

