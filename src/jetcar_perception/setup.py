from glob import glob
from setuptools import setup

package_name = 'jetcar_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kown',
    maintainer_email='kown@example.com',
    description='Perception stack for stereo, depth, detection, and lane processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_camera_node = jetcar_perception.stereo_camera_node:main',
            'stereo_rectification_node = jetcar_perception.stereo_rectification_node:main',
            'stereo_depth_node = jetcar_perception.stereo_depth_node:main',
        ],
    },
)
