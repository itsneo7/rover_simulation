"""Start Gazebo Harmonic with the IGVC arena world, spawn ARES-1, and bridge
sensor/clock topics from gz sim into ROS 2.

    ros2 launch ares1_gazebo gazebo.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('ares1_gazebo')
    pkg_description = get_package_share_directory('ares1_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(pkg_description, 'urdf', 'ares1.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']),
        value_type=str
    )

    bridge_config = os.path.join(pkg_gazebo, 'config', 'gazebo.yaml')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'igvc_arena.sdf'),
        description='Full path to the Gazebo world SDF file'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.30')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world_file, ' -r -v 3']}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ares1',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ares1',
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}],
    )


    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'ares1/base_footprint/rplidar_s3'],
    )

    return LaunchDescription([
        lidar_frame_bridge,
        declare_world,
        declare_x, declare_y, declare_z,
        declare_use_sim_time,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
