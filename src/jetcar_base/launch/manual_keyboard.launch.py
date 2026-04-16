from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetcar_base',
            executable='vehicle_hw_node',
            name='vehicle_hw_node',
            output='screen',
        ),
        Node(
            package='jetcar_base',
            executable='keyboard_control_node',
            name='keyboard_control_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
