"""EXP2 导航模式 —— 一键启动（定位 + 路径规划 + 重定位管理）。

依赖（PM2 需先启动）：
  keli_lidar, hls_canopen, canopen_ros2, imu_driver, imu_bridge, rsp

用法：ros2 launch exp2_nav navigation.launch.py

重定位模式:
  自动:  启动后 3s 自动全局定位（大协方差 → 粒子撒满全图），收敛后自动就绪
  充电桩: ros2 service call /reloc/dock_calib std_srvs/srv/Trigger
  手动:   ros2 topic pub /reloc/set_manual_pose geometry_msgs/msg/PoseWithCovarianceStamped "..."
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory("exp2_nav")
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    params_file = os.path.join(pkg_nav, "config", "nav2_params.yaml")
    map_yaml = "/home/expressone/maps/my_map.yaml"

    # ── Localization (AMCL + map_server) ─────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "localization_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": params_file,
            "use_composition": "False",
            "use_respawn": "False",
            "container_name": "nav2_container",
        }.items(),
    )

    # ── Navigation stack ─────────────────────────────────
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": params_file,
            "use_composition": "False",
            "use_respawn": "False",
            "container_name": "nav2_container",
        }.items(),
    )

    # ── 重定位管理器 ─────────────────────────────────────
    reloc_manager = Node(
        package="exp2_nav",
        executable="reloc_manager.py",
        name="reloc_manager",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        GroupAction([localization_launch, navigation_launch, reloc_manager]),
    ])
