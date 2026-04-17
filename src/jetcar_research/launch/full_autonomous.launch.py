import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('jetcar_base')
    control_share = get_package_share_directory('jetcar_control')
    perception_share = get_package_share_directory('jetcar_perception')
    decision_share = get_package_share_directory('jetcar_decision')
    research_share = get_package_share_directory('jetcar_research')

    return LaunchDescription([
        Node(
            package='jetcar_base',
            executable='vehicle_hw_node',
            name='vehicle_hw_node',
            output='screen',
            parameters=[os.path.join(base_share, 'config', 'vehicle_hw.yaml')],
        ),
        Node(
            package='jetcar_control',
            executable='drive_mode_manager_node',
            name='drive_mode_manager_node',
            output='screen',
            parameters=[
                os.path.join(control_share, 'config', 'control.yaml'),
                {'default_mode': 'MANUAL'},
            ],
        ),
        Node(
            package='jetcar_control',
            executable='control_mux_node',
            name='control_mux_node',
            output='screen',
            parameters=[
                os.path.join(control_share, 'config', 'control.yaml'),
                {'default_mode': 'MANUAL'},
            ],
        ),
        Node(
            package='jetcar_perception',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            parameters=[os.path.join(perception_share, 'config', 'stereo_camera.yaml')],
        ),
        Node(
            package='jetcar_perception',
            executable='stereo_rectification_node',
            name='stereo_rectification_node',
            output='screen',
            parameters=[os.path.join(perception_share, 'config', 'stereo_rectification.yaml')],
        ),
        Node(
            package='jetcar_perception',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
            parameters=[os.path.join(perception_share, 'config', 'stereo_depth.yaml')],
        ),
        Node(
            package='jetcar_perception',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[os.path.join(perception_share, 'config', 'object_detection.yaml')],
        ),
        Node(
            package='jetcar_perception',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[os.path.join(perception_share, 'config', 'lane_detection.yaml')],
        ),
        Node(
            package='jetcar_decision',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[os.path.join(decision_share, 'config', 'safety_supervisor.yaml')],
        ),
        Node(
            package='jetcar_decision',
            executable='autonomous_driver_node',
            name='autonomous_driver_node',
            output='screen',
            parameters=[os.path.join(decision_share, 'config', 'autonomous_driver.yaml')],
        ),
        Node(
            package='jetcar_research',
            executable='experiment_profile_node',
            name='experiment_profile_node',
            output='screen',
            parameters=[
                os.path.join(research_share, 'config', 'research_profiles.yaml'),
                {'profile_name': 'camera_only'},
            ],
        ),
    ])
