from glob import glob
from setuptools import setup

package_name = 'jetcar_control'

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
    description='Drive-mode and command arbitration for JetCar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_mux_node = jetcar_control.control_mux_node:main',
            'drive_mode_manager_node = jetcar_control.drive_mode_manager_node:main',
        ],
    },
)
