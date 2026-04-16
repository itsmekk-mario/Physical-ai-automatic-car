import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('jetcar_research')
    config = os.path.join(package_share, 'config', 'research_profiles.yaml')

    return LaunchDescription([
        Node(
            package='jetcar_research',
            executable='experiment_profile_node',
            name='experiment_profile_node',
            output='screen',
            parameters=[config],
        ),
    ])
