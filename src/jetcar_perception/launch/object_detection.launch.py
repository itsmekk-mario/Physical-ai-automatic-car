import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = get_package_share_directory('jetcar_perception')
    base_share = get_package_share_directory('jetcar_base')
    control_share = get_package_share_directory('jetcar_control')
    stereo_camera_config = os.path.join(perception_share, 'config', 'stereo_camera.yaml')
    object_detection_config = os.path.join(perception_share, 'config', 'object_detection.yaml')
    yolo_config = os.path.join(perception_share, 'config', 'yolo_web_stereo.yaml')
    vehicle_hw_config = os.path.join(base_share, 'config', 'vehicle_hw.yaml')
    control_config = os.path.join(control_share, 'config', 'control.yaml')

    return LaunchDescription([
        Node(
            package='jetcar_base',
            executable='vehicle_hw_node',
            name='vehicle_hw_node',
            output='screen',
            parameters=[vehicle_hw_config],
        ),
        Node(
            package='jetcar_control',
            executable='drive_mode_manager_node',
            name='drive_mode_manager_node',
            output='screen',
            parameters=[control_config],
        ),
        Node(
            package='jetcar_control',
            executable='control_mux_node',
            name='control_mux_node',
            output='screen',
            parameters=[control_config],
        ),
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
        Node(
            package='jetcar_perception',
            executable='yolo_web_node',
            name='yolo_web_node',
            output='screen',
            parameters=[yolo_config],
        ),
    ])
