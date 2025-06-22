from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elrs_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kendall',
    maintainer_email='kendall@digitalmindrobotics.com',
    description='ROS2 node for ELRS transmitter control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elrs_teleop_node = elrs_teleop.elrs_teleop_node:main',
        ],
    },
)
