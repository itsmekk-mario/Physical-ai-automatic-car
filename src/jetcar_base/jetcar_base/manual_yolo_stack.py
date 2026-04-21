import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def build_launch_description():
    base_share = get_package_share_directory('jetcar_base')
    control_share = get_package_share_directory('jetcar_control')
    perception_share = get_package_share_directory('jetcar_perception')
    decision_share = get_package_share_directory('jetcar_decision')

    vehicle_hw_config = os.path.join(base_share, 'config', 'vehicle_hw.yaml')
    control_config = os.path.join(control_share, 'config', 'control.yaml')
    yolo_config = os.path.join(perception_share, 'config', 'yolo_web.yaml')
    stereo_camera_config = os.path.join(perception_share, 'config', 'stereo_camera.yaml')
    stereo_rectification_config = os.path.join(perception_share, 'config', 'stereo_rectification.yaml')
    stereo_depth_config = os.path.join(perception_share, 'config', 'stereo_depth.yaml')
    safety_config = os.path.join(decision_share, 'config', 'safety_supervisor.yaml')

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
            executable='stereo_rectification_node',
            name='stereo_rectification_node',
            output='screen',
            parameters=[stereo_rectification_config],
        ),
        Node(
            package='jetcar_perception',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
            parameters=[stereo_depth_config],
        ),
        Node(
            package='jetcar_decision',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[safety_config],
        ),
        Node(
            package='jetcar_perception',
            executable='yolo_web_node',
            name='yolo_web_node',
            output='screen',
            parameters=[yolo_config, {'image_topic': '/sensors/stereo/left/image_raw'}],
        ),
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(build_launch_description())
    raise SystemExit(launch_service.run())


if __name__ == '__main__':
    main()
