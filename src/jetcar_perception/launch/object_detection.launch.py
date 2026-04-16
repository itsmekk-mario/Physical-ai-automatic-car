import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('jetcar_perception')
    stereo_camera_config = os.path.join(package_share, 'config', 'stereo_camera.yaml')
    object_detection_config = os.path.join(package_share, 'config', 'object_detection.yaml')

    return LaunchDescription([
        Node(
            package='jetcar_perception',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[stereo_camera_config],
        ),
        Node(
            package='jetcar_perception',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[object_detection_config],
        ),
    ])
