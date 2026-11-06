"""Spawn ARES-1's ros2_control controllers and start the swivel_controller
cmd_vel -> steering/wheel bridge node.

Assumes gz_ros2_control's GazeboSimROS2ControlPlugin (loaded from the URDF)
has already started a `controller_manager` node inside the Gazebo process
using ares1_control/config/ros2_controllers.yaml.

    ros2 launch ares1_control control.launch.py
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    wheel_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    swivel_controller_node = Node(
        package='ares1_control',
        executable='swivel_controller',
        name='swivel_controller',
        output='screen',
        parameters=[{
            'wheelbase': 0.55,
            'track_width': 0.45,
            'wheel_radius': 0.11,
            'max_linear_speed': 1.5,
            'max_angular_speed': 1.5,
            'control_rate_hz': 50.0,
            'cmd_timeout_sec': 0.5,
        }],
    )

    # Spawners must run after Gazebo + gz_ros2_control has brought up the
    # controller_manager; a short stagger avoids race conditions on startup.
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        TimerAction(period=2.0, actions=[steering_controller_spawner]),
        TimerAction(period=2.5, actions=[wheel_velocity_controller_spawner]),
        TimerAction(period=3.0, actions=[swivel_controller_node]),
    ])
