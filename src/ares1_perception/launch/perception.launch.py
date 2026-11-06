"""Start ARES-1's perception pipeline: depth processing, object detection
(simulation backend by default), obstacle fusion, and the TFmini bridge.

    ros2 launch ares1_perception perception.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    detector_backend = LaunchConfiguration('detector_backend')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('detector_backend', default_value='simulated',
                               description='"simulated" (default) or "yolo" '
                                            '(requires RealYoloDetectorBackend to be implemented)'),

        Node(
            package='ares1_perception',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='ares1_perception',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'detector_backend': detector_backend}],
        ),
        Node(
            package='ares1_perception',
            executable='obstacle_fusion',
            name='obstacle_fusion',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='ares1_perception',
            executable='tfmini_bridge',
            name='tfmini_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
