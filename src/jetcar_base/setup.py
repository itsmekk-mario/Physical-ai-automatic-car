from setuptools import setup
from glob import glob

package_name = 'jetcar_base'

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
    install_requires=['setuptools', 'smbus2', 'flask'],
    zip_safe=True,
    maintainer='kown',
    maintainer_email='kown@example.com',
    description='Jetson vehicle base control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_hw_node = jetcar_base.vehicle_hw_node:main',
            'servo_test_node = jetcar_base.servo_test_node:main',
            'servo_boot_test_node = jetcar_base.servo_boot_test_node:main',
            'motor_test_node = jetcar_base.motor_test_node:main',
            'keyboard_control_node = jetcar_base.keyboard_control_node:main',
            'web_control_node = jetcar_base.web_control_node:main',
            'manual_web_stack = jetcar_base.manual_web_stack:main',
            'manual_yolo_stack = jetcar_base.manual_yolo_stack:main',
        ],
    },
)
