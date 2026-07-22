"""Launch DYP-A21 C++ CAN ultrasonic driver."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("dyp_a21_ultrasonic_cpp")
    default_params = os.path.join(pkg_dir, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to YAML parameter file",
        ),
        Node(
            package="dyp_a21_ultrasonic_cpp",
            executable="ultrasonic_node",
            name="dyp_a21_ultrasonic",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
