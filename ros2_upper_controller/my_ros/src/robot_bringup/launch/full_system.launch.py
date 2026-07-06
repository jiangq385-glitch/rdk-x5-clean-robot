#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('robot_bringup'), 'launch', 'arm_only.launch.py'])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('robot_vision'), 'launch', 'robot_vision.launch.py'])
            )
        ),
        Node(
            package='robot_skills',
            executable='pick_object_server',
            name='pick_object_server',
            output='screen',
        ),
        Node(
            package='robot_skills',
            executable='place_object_server',
            name='place_object_server',
            output='screen',
        ),
        Node(
            package='robot_skills',
            executable='navigate_to_object_server',
            name='navigate_to_object_server',
            output='screen',
        ),
        Node(
            package='robot_skills',
            executable='clean_area_server',
            name='clean_area_server',
            output='screen',
        ),
        Node(
            package='robot_skills',
            executable='return_home_server',
            name='return_home_server',
            output='screen',
        ),
        Node(
            package='robot_brain',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
        ),
        Node(
            package='robot_brain',
            executable='robot_state_aggregator',
            name='robot_state_aggregator',
            output='screen',
        ),
    ])
