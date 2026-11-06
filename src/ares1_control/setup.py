import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ares1_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares1_team',
    maintainer_email='maintainer@example.com',
    description='Swivel-drive inverse kinematics and ros2_control command bridge for ARES-1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swivel_controller = ares1_control.swivel_controller:main',
            'wheel_odometry = ares1_control.wheel_odometry:main',
        ],
    },
)
