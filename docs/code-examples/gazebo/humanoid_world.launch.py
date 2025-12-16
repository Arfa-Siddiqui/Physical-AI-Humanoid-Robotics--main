#!/usr/bin/env python3
"""
Gazebo Launch File for Humanoid Robot Simulation
Launches Gazebo with a simple world and spawns a humanoid robot

Usage:
    ros2 launch <package_name> humanoid_world.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='empty.world')

    # Path to Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # Gazebo server (physics simulation)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Gazebo client (visualization)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        )
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': '$(find humanoid_description)/urdf/humanoid.urdf.xacro'
        }]
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen'
    )

    # Joint state publisher (for manual control during testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('world_file', default_value='empty.world',
                            description='Gazebo world file'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        joint_state_publisher_gui
    ])
