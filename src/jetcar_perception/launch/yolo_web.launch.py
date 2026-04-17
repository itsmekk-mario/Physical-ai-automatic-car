import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('jetcar_perception')
    config = os.path.join(package_share, 'config', 'yolo_web.yaml')

    return LaunchDescription([
        Node(
            package='jetcar_perception',
            executable='yolo_web_node',
            name='yolo_web_node',
            output='screen',
            parameters=[config],
        ),
    ])
