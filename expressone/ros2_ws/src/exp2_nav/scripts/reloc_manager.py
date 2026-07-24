#!/usr/bin/env python3
"""
reloc_manager - 360 deg rotation + large covariance (odometry-based)
"""
import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from std_msgs.msg import String

try:
    import os as _os, sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    from scan_matcher import ScanMatcher
    _SCAN_MATCHER_OK = True
    _SCAN_MATCHER_ERR = ''
except Exception as _e:          # cv2/numpy 缺失时降级到 rotate
    _SCAN_MATCHER_OK = False
    _SCAN_MATCHER_ERR = str(_e)


def _quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RelocManager(Node):
    DEBOUNCE_SEC = 0.5
    DECEL_DURATION = 0.5      # 500ms deceleration ramp
    DECEL_HOLD = 0.3           # 300ms hold at zero after ramp

    def __init__(self):
        super().__init__('reloc_manager')
        self.declare_parameter('dock.x', 1.768)
        self.declare_parameter('dock.y', 3.188)
        self.declare_parameter('dock.yaw', 2.361)
        self.declare_parameter('manual_cov_xx', 1.0)
        self.declare_parameter('manual_cov_yy', 1.0)
        self.declare_parameter('manual_cov_yyaw', 0.5)
        self.declare_parameter('converged_xy_cov', 0.1)
        self.declare_parameter('localize_timeout_sec', 60.0)
        self.declare_parameter('swing_enabled', True)
        self.declare_parameter('swing_speed', 0.2)
        self.declare_parameter('swing_angle_deg', 360.0)
        self.declare_parameter('swing_tolerance_deg', 4.0)
        # ── 重定位模式 ──
        self.declare_parameter('reloc_mode', 'mh')             # 'mh'(多假设,默认) | 'rotate'(v1兜底)
        self.declare_parameter('scan_map_yaml', '/home/expressone/maps/my_map.yaml')  # mh 诊断 matcher 用
        self.declare_parameter('scan_cap_m', 8.0)              # 扫描渲染半径上限(m)
        # ── mh(多假设)模式: 单帧匹配多解→发协方差让 AMCL 收敛 ──
        self.declare_parameter('mh_cov_xy', 0.5)       # xy 协方差(σ², σ≈0.7m; 收紧信任点击排除假吸引子)
        self.declare_parameter('mh_cov_yyaw', 0.3)     # yaw 协方差(rad², σ≈31°; 收紧排除反向假模态)
        self.declare_parameter('mh_topk', 5)           # top-N 多解诊断候选数
        # mh 诊断 matcher 粗搜参数(仅日志展示多解, 不决定位姿)
        self.declare_parameter('mh_window_m', 2.0)
        self.declare_parameter('mh_coarse_step_m', 1.0)
        self.declare_parameter('mh_coarse_yaw_step_deg', 10)
        self.declare_parameter('mh_trim_frac', 0.85)
        self._load_params()

        self.state = 'UNLOCALIZED'
        self.start_time = None
        self._last_initial_pose_time = 0.0
        self._converged = False
        self.last_odom_yaw = 0.0

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initial_pose_cb, 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, depth=10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_cb, qos)
        self.manual_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/reloc/set_manual_pose',
            self._manual_pose_cb, 10)
        self.status_pub = self.create_publisher(String, '/reloc/status', 10)
        self.dock_srv = self.create_service(
            Trigger, '/reloc/dock_calib', self._dock_calib_cb)
        # 发到 collision_monitor 的输入(/cmd_vel_smoothed)而非其输出(/cmd_vel),
        # 否则 collision_monitor 空闲时发的刹车 0 会盖掉旋转指令。原地旋转线速度=0, 不会被拦。
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)
        # mh 模式: 缓存最新 /scan 供 top-N 诊断
        self._latest_scan = None
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)

        self.rotating = False
        self.decelerating = False
        self.rotation_start_time = 0.0
        self.rotation_start_yaw = 0.0
        self.total_rotated = 0.0
        self.prev_odom_yaw = 0.0
        self.rotation_timer = None

        self.timer = self.create_timer(1.0, self._timer_cb)
        # mh 动态调 AMCL 运动门限: 收敛期临时设≈0(静止也能收敛), 收敛/超时后恢复原值。
        # 全局保持 0 会有 CPU(每帧update) + 粒子过度收敛→recovery注入突刺; 故仅收敛期降低。
        self._amcl_param_client = AsyncParameterClient(self, 'amcl')
        self._amcl_gate_normal = (0.25, 0.1)   # AMCL 正常运动门限(=config), 收敛后恢复用
        self._mh_gate_lowered = False
        if self.reloc_mode == 'rotate':
            tail = '(360deg rotation)'
        else:   # mh
            tail = '(宽协方差+AMCL收敛' + ('+matcher诊断' if self.matcher else ',无matcher诊断') + ')'
        self.get_logger().info(f"reloc_manager started: mode='{self.reloc_mode}' {tail}")

    def _load_params(self):
        self.dock_x = self.get_parameter('dock.x').value
        self.dock_y = self.get_parameter('dock.y').value
        self.dock_yaw = self.get_parameter('dock.yaw').value
        self.manual_cov_xx = self.get_parameter('manual_cov_xx').value
        self.manual_cov_yy = self.get_parameter('manual_cov_yy').value
        self.manual_cov_yyaw = self.get_parameter('manual_cov_yyaw').value
        self.converged_xy_cov = self.get_parameter('converged_xy_cov').value
        self.timeout = self.get_parameter('localize_timeout_sec').value
        self.swing_enabled = self.get_parameter('swing_enabled').value
        self.swing_speed = float(self.get_parameter('swing_speed').value)
        swing_angle_deg = float(self.get_parameter('swing_angle_deg').value)
        self.swing_angle = math.radians(swing_angle_deg)
        # safety timeout: 150% of expected duration
        self.rotation_timeout = self.swing_angle / self.swing_speed * 1.5 + 1.0
        # ── mh 模式参数 ──
        self.reloc_mode = str(self.get_parameter('reloc_mode').value)
        self.scan_map_yaml = str(self.get_parameter('scan_map_yaml').value)
        self.scan_cap_m = float(self.get_parameter('scan_cap_m').value)
        self.mh_cov_xy = float(self.get_parameter('mh_cov_xy').value)
        self.mh_cov_yyaw = float(self.get_parameter('mh_cov_yyaw').value)
        self.mh_topk = int(self.get_parameter('mh_topk').value)
        self.mh_window_m = float(self.get_parameter('mh_window_m').value)
        self.mh_coarse_step_m = float(self.get_parameter('mh_coarse_step_m').value)
        self.mh_coarse_yaw_step_deg = int(self.get_parameter('mh_coarse_yaw_step_deg').value)
        self.mh_trim_frac = float(self.get_parameter('mh_trim_frac').value)
        # mh 诊断 matcher(可选; 失败不影响主流程, 仅跳过 top-N 诊断)
        self.matcher = None
        if self.reloc_mode == 'mh' and _SCAN_MATCHER_OK:
            try:
                self.matcher = ScanMatcher(self.scan_map_yaml, self.scan_cap_m)
            except Exception as e:
                self.get_logger().warn(f'ScanMatcher 初始化失败, mh 跳过 top-N 诊断: {e}')
        elif self.reloc_mode == 'mh':
            self.get_logger().warn(f'scan_matcher 导入失败, mh 跳过 top-N 诊断: {_SCAN_MATCHER_ERR}')

    def publish_initial_pose(self, x, y, yaw,
                             cov_xx, cov_yy, cov_yyaw, frame='map'):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        cov = [0.0] * 36
        cov[0] = float(cov_xx)
        cov[7] = float(cov_yy)
        cov[35] = float(cov_yyaw)
        msg.pose.covariance = cov
        self._last_initial_pose_time = time.time()
        self.initial_pose_pub.publish(msg)

    def _enter_localizing(self, do_rotation=None):
        if do_rotation is None:
            do_rotation = self.swing_enabled
        self.state = 'LOCALIZING'
        self.start_time = time.time()
        self._converged = False
        self._publish_status()
        if do_rotation:
            self._start_rotation()
        else:
            self.get_logger().info('localizing without rotation (AMCL 静止精修)')

    # ────────────────────── odometry ──────────────────────
    def _odom_cb(self, msg: Odometry):
        self.last_odom_yaw = _quat_to_yaw(msg.pose.pose.orientation)

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg

    # ────────────────────── 360deg rotation ───────────────
    def _start_rotation(self):
        if self.rotation_timer is not None:
            self.rotation_timer.cancel()
        self.rotating = True
        self.decelerating = False
        self.rotation_start_time = time.time()
        self.rotation_start_yaw = self.last_odom_yaw
        self.total_rotated = 0.0
        self.prev_odom_yaw = self.last_odom_yaw
        self.rotation_timer = self.create_timer(0.05, self._rotation_step)
        self.get_logger().info(
            f'Starting {math.degrees(self.swing_angle):.0f}deg rotation '
            f'at {self.swing_speed:.2f} rad/s '
            f'(cov={self.manual_cov_xx:.1f},{self.manual_cov_yy:.1f})')

    def _rotation_step(self):
        # Phase 2: deceleration ramp (runs even when not rotating)
        if self.decelerating:
            elapsed = time.time() - self.decel_start_time
            if elapsed >= self.DECEL_DURATION + self.DECEL_HOLD:
                self.decelerating = False
                if self.rotation_timer is not None:
                    self.rotation_timer.cancel()
                    self.rotation_timer = None
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('Deceleration complete, robot stopped')
                return

            if elapsed < self.DECEL_DURATION:
                frac = elapsed / self.DECEL_DURATION
                speed = self.decel_start_speed * (1.0 - frac)
            else:
                speed = 0.0

            twist = Twist()
            twist.angular.z = float(speed)
            self.cmd_vel_pub.publish(twist)
            return

        # Phase 1: normal rotation
        if not self.rotating or \
           self.state not in ('LOCALIZING', 'CONVERGED'):
            return

        # accumulate odometry-based rotation
        delta = _norm_angle(self.last_odom_yaw - self.prev_odom_yaw)
        self.total_rotated += abs(delta)
        self.prev_odom_yaw = self.last_odom_yaw

        # stop when 360deg reached (odometry-based)
        if self.total_rotated >= self.swing_angle:
            self.get_logger().info(
                f'Rotation complete: {math.degrees(self.total_rotated):.0f}deg '
                f'(target {math.degrees(self.swing_angle):.0f}deg), '
                f'converged={self._converged}, starting deceleration ramp')
            if not self._converged:
                self.get_logger().warn(
                    'Rotation done but NOT converged. Re-click (<1m error).')
                self.state = 'UNLOCALIZED'
            self._stop_rotation()
            return

        # safety timeout fallback
        elapsed = time.time() - self.rotation_start_time
        if elapsed > self.rotation_timeout:
            self.get_logger().warn(
                f'Rotation timeout ({elapsed:.0f}s > {self.rotation_timeout:.0f}s), '
                f'rotated {math.degrees(self.total_rotated):.0f}deg')
            if not self._converged:
                self.state = 'UNLOCALIZED'
            self._stop_rotation()
            return

        twist = Twist()
        twist.angular.z = float(self.swing_speed)
        self.cmd_vel_pub.publish(twist)

    def _stop_rotation(self):
        """Initiate deceleration ramp instead of instant brake."""
        if self.decelerating:
            return
        if not self.rotating:
            return
        self.rotating = False
        self.decelerating = True
        self.decel_start_time = time.time()
        self.decel_start_speed = self.swing_speed
        self.get_logger().info(
            f'Deceleration ramp started: '
            f'{self.decel_start_speed:.2f} rad/s -> 0 over {self.DECEL_DURATION}s '
            f'(+{self.DECEL_HOLD}s hold)')

    # ────────────────────── callbacks ────────────────────
    def _initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        now = time.time()
        if now - self._last_initial_pose_time < self.DEBOUNCE_SEC:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        frame = msg.header.frame_id or 'map'
        if self.reloc_mode == 'mh':
            self._handle_mh_click(x, y, yaw, frame)
        else:
            # rotate 模式(v1 兜底): 发大协方差 + 360°旋转
            self.get_logger().info(
                f'rviz click: ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}deg) '
                f'cov=({self.manual_cov_xx:.1f},{self.manual_cov_yy:.1f}) +360deg')
            self.publish_initial_pose(
                x, y, yaw, self.manual_cov_xx, self.manual_cov_yy,
                self.manual_cov_yyaw, frame)
            self._enter_localizing()

    def _amcl_set_gate(self, d, a):
        try:
            self._amcl_param_client.set_parameters([
                Parameter('update_min_d', value=float(d)),
                Parameter('update_min_a', value=float(a))])
        except Exception as e:
            self.get_logger().warn(f'设 AMCL 运动门限失败(忽略): {e}')

    def _amcl_restore_gate(self):
        if self._mh_gate_lowered:
            self._amcl_set_gate(*self._amcl_gate_normal)
            self._mh_gate_lowered = False

    def _handle_mh_click(self, click_x, click_y, click_yaw, frame):
        """mh(多假设)模式: 实测单帧匹配在此环境多解(真值常非最优), 故不盲信 matcher 单解。
        跑 top-N 仅记日志展示多解 → 发宽协方差 initialpose 覆盖窗口内真值+假吸引子模态
        → 不旋转, 等 AMCL 靠 sensor update 收敛到真值(cov<阈值→CONVERGED)。
        关键: 宽协方差让 AMCL 能逃出错模态 + 动态降运动门限(静止也收敛) + 等 AMCL 真收敛。"""
        # 诊断: top-N 展示多解(matcher/scan 缺失或异常都不影响主流程)
        scan = self._latest_scan
        if scan is not None and self.matcher is not None:
            try:
                t0 = time.time()
                tops = self.matcher.match_topn(
                    list(scan.ranges), scan.angle_min, scan.angle_increment,
                    click_x, click_y, topn=self.mh_topk,
                    window_m=self.mh_window_m, coarse_step_m=self.mh_coarse_step_m,
                    coarse_yaw_step_deg=self.mh_coarse_yaw_step_deg,
                    trim_frac=self.mh_trim_frac)
                dt = time.time() - t0
                if tops:
                    self.get_logger().info(
                        f"mh 多解诊断({dt:.1f}s, top{len(tops)}): " + " | ".join(
                            f"#{i+1}({t['x']:+.2f},{t['y']:+.2f},yaw{t['yaw_deg']:.0f}°,"
                            f"s{t['score_m']:.3f},距点击{math.hypot(t['x']-click_x, t['y']-click_y):.2f}m)"
                            for i, t in enumerate(tops)))
                    if len(tops) >= 2 and tops[0]['score_m'] > 1e-6:
                        gap = tops[1]['score_m'] - tops[0]['score_m']
                        if gap / tops[0]['score_m'] < 0.4:
                            self.get_logger().warn(
                                f"mh: 第1/2解 score 接近(gap {gap:.3f}m)→单帧多解, 依赖 AMCL 移动收敛")
            except Exception as e:
                self.get_logger().warn(f"mh top-N 诊断异常(忽略, 照常发 initialpose): {e}")
        # 主流程: 发宽协方差 initialpose, 让 AMCL 粒子覆盖窗口内多模态, 不旋转等收敛
        self.get_logger().info(
            f"rviz click ({click_x:.2f},{click_y:.2f},yaw{math.degrees(click_yaw):.0f}°) "
            f"[mh] 发宽协方差(cov_xy={self.mh_cov_xy:.2f},cov_yaw={self.mh_cov_yyaw:.2f}) "
            f"initialpose, 不旋转, 等 AMCL 移动收敛")
        # 收敛期临时降低 AMCL 运动门限→静止也跑 sensor update 收敛(否则卡在多模态协方差)
        if not self._mh_gate_lowered:
            self._amcl_set_gate(0.0, 0.0)
            self._mh_gate_lowered = True
        self.publish_initial_pose(
            click_x, click_y, click_yaw,
            self.mh_cov_xy, self.mh_cov_xy, self.mh_cov_yyaw, frame)
        self._enter_localizing(do_rotation=False)

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.state != 'LOCALIZING':
            return
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        if cov_x < self.converged_xy_cov and cov_y < self.converged_xy_cov:
            elapsed = time.time() - self.start_time if self.start_time else 0.0
            tail = 'waiting for rotation to finish' if self.rotating else 'localized'
            self.get_logger().info(
                f'Converged! t={elapsed:.1f}s '
                f'cov=({cov_x:.4f},{cov_y:.4f}) '
                f'pos=({msg.pose.pose.position.x:.2f},'
                f'{msg.pose.pose.position.y:.2f}) {tail}')
            self.state = 'CONVERGED'
            self._converged = True
            self._amcl_restore_gate()   # mh 收敛完成→恢复 AMCL 正常运动门限
            self._publish_status()

    def _manual_pose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(
            f'Manual: ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}deg)')
        self.publish_initial_pose(
            x, y, yaw, self.manual_cov_xx, self.manual_cov_yy,
            self.manual_cov_yyaw, msg.header.frame_id or 'map')
        self._enter_localizing()

    def _dock_calib_cb(self, request, response):
        self._amcl_restore_gate()   # 切充电桩标定→恢复 AMCL 门限(mh 期间被打断时)
        self._stop_rotation()
        self.get_logger().info(
            f'Dock: ({self.dock_x:.2f},{self.dock_y:.2f},'
            f'{math.degrees(self.dock_yaw):.0f}deg)')
        self.publish_initial_pose(
            self.dock_x, self.dock_y, self.dock_yaw, 0.01, 0.01, 0.001)
        self.state = 'CONVERGED'
        self._publish_status()
        response.success = True
        response.message = f'Dock calibrated ({self.dock_x:.2f},{self.dock_y:.2f})'
        return response

    def _timer_cb(self):
        self._publish_status()
        if self.state == 'LOCALIZING' and self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > self.timeout:
                self._stop_rotation()
                self.get_logger().warn(
                    f'Timeout ({self.timeout:.0f}s). Re-click.')
                self.state = 'UNLOCALIZED'
                self.start_time = None
                self._amcl_restore_gate()   # mh 超时→恢复 AMCL 门限

    def _publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = RelocManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
