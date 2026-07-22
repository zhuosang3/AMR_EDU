import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='keli_lidar',
            executable='keli_lidar',
            name='keli_lidar',
            output='screen',
            parameters=[{
                'hostname': '192.168.8.11',
                'port': 2112,
                'frame_id': 'laser',
                'range_min': 0.05,
                'range_max': 10.0,
                'time_increment': 0.000040,
                'time_offset': 0.0,
                'min_ang': -3.14159,
                'max_ang': 3.14159,
                'intensity': True,
                'timeout': 5,
                'skip': 0,
            }],
        )
    ])
