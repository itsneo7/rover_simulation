import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ares1_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares1_team',
    maintainer_email='maintainer@example.com',
    description='Depth processing, detection, and obstacle fusion for ARES-1.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_processor = ares1_perception.depth_processor:main',
            'object_detector = ares1_perception.object_detector:main',
            'obstacle_fusion = ares1_perception.obstacle_fusion:main',
            'tfmini_bridge = ares1_perception.tfmini_bridge:main',
        ],
    },
)
