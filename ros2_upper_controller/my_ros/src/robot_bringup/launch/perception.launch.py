#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([
        get_package_share_directory('robot_vision'),
        'config',
        'vision_pipeline.yaml',
    ])
    return LaunchDescription([
        Node(package='robot_vision', executable='mipi_yolo_node', name='mipi_yolo_node', output='screen'),
        Node(package='robot_vision', executable='laser_range_node', name='laser_range_node', output='screen'),
        Node(
            package='robot_vision',
            executable='object_on_plane_node',
            name='object_on_plane_node',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='robot_vision',
            executable='plane_calibration_node',
            name='plane_calibration_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
