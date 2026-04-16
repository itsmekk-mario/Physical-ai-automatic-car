from glob import glob
from setuptools import setup

package_name = 'jetcar_research'

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
    description='Research launch and experiment profiles for JetCar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_profile_node = jetcar_research.experiment_profile_node:main',
        ],
    },
)
