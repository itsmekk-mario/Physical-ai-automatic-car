import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('jetcar_base')
    control_share = get_package_share_directory('jetcar_control')
    vehicle_hw_config = os.path.join(package_share, 'config', 'vehicle_hw.yaml')
    keyboard_config = os.path.join(package_share, 'config', 'manual_keyboard.yaml')
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
            executable='keyboard_control_node',
            name='keyboard_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[keyboard_config],
        ),
    ])
