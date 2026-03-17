from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('minibot_scan_fusion')
    params = os.path.join(pkg, 'params', 'minibot_scan_fusion.yaml')

    return LaunchDescription([
        Node(
            package='minibot_scan_fusion',
            executable='minibot_scan_fusion_node',
            name='minibot_scan_fusion_node',
            output='screen',
            parameters=[params],
        )
    ])