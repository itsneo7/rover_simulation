import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ares1_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares1_team',
    maintainer_email='maintainer@example.com',
    description='Top-level bringup, RViz config, status monitor and safety watchdog for ARES-1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_monitor = ares1_bringup.status_monitor:main',
            'safety_watchdog = ares1_bringup.safety_watchdog:main',
        ],
    },
)
