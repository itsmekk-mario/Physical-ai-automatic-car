import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def build_launch_description():
    base_share = get_package_share_directory('jetcar_base')
    control_share = get_package_share_directory('jetcar_control')

    vehicle_hw_config = os.path.join(base_share, 'config', 'vehicle_hw.yaml')
    web_config = os.path.join(base_share, 'config', 'manual_web.yaml')
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
            package='jetcar_base',
            executable='web_control_node',
            name='web_control_node',
            output='screen',
            parameters=[web_config],
        ),
    ])


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(build_launch_description())
    raise SystemExit(launch_service.run())


if __name__ == '__main__':
    main()
