"""sensors.launch.py

ARES-1's sensors (RPLIDAR S3, RealSense D456, BNO085 IMU, TFmini) are
defined directly in the robot's URDF/Xacro (ares1_description/urdf/sensors.xacro)
as Gazebo-native sensor plugins, and come online automatically the moment
the robot is spawned by ares1_gazebo/launch/gazebo.launch.py — there is no
separate sensor driver process to start in simulation.

This launch file exists as an explicit, named pipeline stage (as required
by the top-level launch architecture) and starts the TFmini raw-scan ->
Range conversion bridge, which IS a genuine separate ROS node (see
ares1_perception/ares1_perception/tfmini_bridge.py). All other sensor data
flows directly from the Gazebo <-> ROS bridge started in gazebo.launch.py.

    ros2 launch ares1_bringup sensors.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ares1_perception',
            executable='tfmini_bridge',
            name='tfmini_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
