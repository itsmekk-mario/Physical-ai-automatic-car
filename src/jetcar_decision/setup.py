from glob import glob
from setuptools import setup

package_name = 'jetcar_decision'

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
    description='Decision and safety supervisor nodes for JetCar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor_node = jetcar_decision.safety_supervisor_node:main',
        ],
    },
)
