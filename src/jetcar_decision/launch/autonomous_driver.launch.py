import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('jetcar_decision')
    config = os.path.join(package_share, 'config', 'autonomous_driver.yaml')

    return LaunchDescription([
        Node(
            package='jetcar_decision',
            executable='autonomous_driver_node',
            name='autonomous_driver_node',
            output='screen',
            parameters=[config],
        ),
    ])
