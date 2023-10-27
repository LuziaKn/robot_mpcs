import os
from glob import glob
from setuptools import setup

package_name = 'robotmpcs_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luzia Knoedler',
    maintainer_email='l.knoedler@tudelft.nl',
    description='ros2 bridge for robotmpcs repo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_planner_node = robotmpcs_ros2.mpc_planner_node:main'
        ],
    },
)
