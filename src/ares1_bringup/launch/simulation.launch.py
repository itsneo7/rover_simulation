"""simulation.launch.py — THE single master command for the full ARES-1
simulation:

    ros2 launch ares1_bringup simulation.launch.py

This is a thin wrapper around full_system.launch.py with use_sim:=true and
use_sim_time:=true pinned (simulation always uses Gazebo + sim time), while
still exposing use_rviz and detector_backend for convenience.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ares1_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    detector_backend = LaunchConfiguration('detector_backend')

    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'full_system.launch.py')),
        launch_arguments={
            'use_sim': 'true',
            'use_sim_time': 'true',
            'use_rviz': use_rviz,
            'detector_backend': detector_backend,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('detector_backend', default_value='simulated'),
        full_system_launch,
    ])
