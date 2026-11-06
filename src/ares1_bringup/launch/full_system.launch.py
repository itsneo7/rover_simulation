"""full_system.launch.py — brings up EVERY ARES-1 subsystem in a sensible
dependency order:

  Gazebo + robot spawn + gz<->ROS bridge
      -> ros2_control controllers + swivel_controller
          -> localization (wheel_odometry, EKF, SLAM)
              -> perception (depth processing, detection, fusion, TFmini)
                  -> navigation (Nav2: A* planner, MPPI/Omni controller)
                      -> RViz2, status_monitor, safety_watchdog

`use_sim` (default true) controls whether Gazebo is started. Setting it to
false is the extension point for running this same launch tree against
real ARES-1 hardware in the future (a real hardware ros2_control plugin
would be substituted for gz_ros2_control in the URDF at that point — no
real hardware driver is implemented in this simulation-only project).

    ros2 launch ares1_bringup full_system.launch.py
    ros2 launch ares1_bringup full_system.launch.py use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('ares1_gazebo')
    pkg_control = get_package_share_directory('ares1_control')
    pkg_localization = get_package_share_directory('ares1_localization')
    pkg_perception = get_package_share_directory('ares1_perception')
    pkg_navigation = get_package_share_directory('ares1_navigation')
    pkg_bringup = get_package_share_directory('ares1_bringup')

    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    detector_backend = LaunchConfiguration('detector_backend')

    rviz_config = os.path.join(pkg_bringup, 'rviz', 'ares1.rviz')

    declare_args = [
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('detector_backend', default_value='simulated'),
    ]

    # --- Stage 1: Gazebo + robot spawn + gz<->ROS bridge -----------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_sim),
    )

    # --- Stage 2: ros2_control controllers + swivel bridge ---------------
    control_launch = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_control, 'launch', 'control.launch.py')),
        )],
    )

    # --- Stage 3: localization (wheel_odometry, EKF, SLAM) ----------------
    localization_launch = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_localization, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    # --- Stage 4: perception ----------------------------------------------
    perception_launch = TimerAction(
        period=7.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_perception, 'launch', 'perception.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'detector_backend': detector_backend,
            }.items(),
        )],
    )

    # --- Stage 5: navigation (Nav2: A* + MPPI Omni) -----------------------
    navigation_launch = TimerAction(
        period=9.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    )

    # --- Stage 6: RViz2, monitoring, safety --------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    status_monitor_node = Node(
        package='ares1_bringup',
        executable='status_monitor',
        name='status_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    safety_watchdog_node = Node(
        package='ares1_bringup',
        executable='safety_watchdog',
        name='safety_watchdog',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        declare_args + [
            gazebo_launch,
            control_launch,
            localization_launch,
            perception_launch,
            navigation_launch,
            rviz_node,
            status_monitor_node,
            safety_watchdog_node,
        ]
    )
