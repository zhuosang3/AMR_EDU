"""EXP2 建图模式 —— 一键启动。

依赖（PM2 需先启动）：
  keli_lidar, hls_canopen, canopen_ros2, imu_driver, imu_bridge, rsp

用法：ros2 launch exp2_nav mapping.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_nav = get_package_share_directory("exp2_nav")
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    params_file = os.path.join(pkg_nav, "config", "nav2_params.yaml")
    slam_config = os.path.join(pkg_nav, "config", "mapper_params_online_async.yaml")

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

    # ── SLAM toolbox (online_async) ───────────────────────
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": False},
            {"map_file_name": "/home/expressone/maps/map"},
        ],
    )

    # ── Map saver server (用于 ros2 run nav2_map_server map_saver_cli) ──
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        respawn=False,
        parameters=[{"save_map_timeout": 10.0}],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"autostart": True},
            {"node_names": ["map_saver", "slam_toolbox"]},
        ],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),
        GroupAction([
            navigation_launch,
            slam_toolbox,
            map_saver,
            lifecycle,
        ]),
    ])
