"""Start ARES-1's localization stack:

  wheel_odometry (ares1_control)  -> /odom
  robot_localization ekf_node     -> odom -> base_link TF
  slam_toolbox async_slam_toolbox_node -> map -> odom TF, /map

    ros2 launch ares1_localization localization.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('ares1_localization')

    ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_localization, 'config', 'slam.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    wheel_odometry_node = Node(
        package='ares1_control',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'wheelbase': 0.55,
            'track_width': 0.45,
            'wheel_radius': 0.11,
            'publish_rate_hz': 50.0,
            'use_sim_time': use_sim_time,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        wheel_odometry_node,
        ekf_node,
        slam_node,
    ])
