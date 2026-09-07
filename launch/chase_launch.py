from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('chase'),  
        'config',
        'chase.yaml'
    )

    return LaunchDescription([
        Node(
            package='chase',
            executable='calculate_attack_pose',
            name='chase',
            output='screen',
            parameters=[params],
            prefix=['taskset -c 8']
        ),
    ])
