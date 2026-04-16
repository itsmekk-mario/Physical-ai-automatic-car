import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('jetcar_base')
    vehicle_hw_config = os.path.join(package_share, 'config', 'vehicle_hw.yaml')
    web_config = os.path.join(package_share, 'config', 'manual_web.yaml')

    return LaunchDescription([
        Node(
            package='jetcar_base',
            executable='vehicle_hw_node',
            name='vehicle_hw_node',
            output='screen',
            parameters=[vehicle_hw_config],
        ),
        Node(
            package='jetcar_base',
            executable='web_control_node',
            name='web_control_node',
            output='screen',
            parameters=[web_config],
        ),
    ])
