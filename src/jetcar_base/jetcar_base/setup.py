from setuptools import setup

package_name = 'jetcar_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Jetson Orin Nano vehicle base hardware interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_hw_node = jetcar_base.vehicle_hw_node:main',
            'servo_test_node = jetcar_base.servo_test_node:main',
            'motor_test_node = jetcar_base.motor_test_node:main',
            'keyboard_control_node = jetcar_base.keyboard_control_node:main',
            'servo_boot_test_node = jetcar_base.servo_boot_test_node:main',
        ],
    }
)