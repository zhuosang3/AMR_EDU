"""
Launch file for HI12 IMU node.

Usage:
    ros2 launch hi12_imu hi12_imu.launch.py

    # Custom parameters:
    ros2 launch hi12_imu hi12_imu.launch.py \
        device:=/dev/ttyS6 \
        baudrate:=460800 \
        frame_id:=imu_link \
        publish_hz:=500

    # With parameter file:
    ros2 launch hi12_imu hi12_imu.launch.py \
        params_file:=/path/to/hi12_params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('hi12_imu')
    default_params = os.path.join(pkg_dir, 'config', 'hi12_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyS4',
            description='Serial port for HI12 IMU (UART4 on Orange Pi 5 Plus)'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baud rate. Use 460800 or 921600 for >200Hz output'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='TF frame ID for published messages'
        ),
        DeclareLaunchArgument(
            'publish_hz',
            default_value='200',
            description='Max publish rate in Hz (0 = unlimited)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to ROS2 parameter YAML file'
        ),

        Node(
            package='hi12_imu',
            executable='hi12_imu_node',
            name='hi12_imu',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_hz': LaunchConfiguration('publish_hz'),
            }],
            # Restart on serial errors (e.g. USB unplug)
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
