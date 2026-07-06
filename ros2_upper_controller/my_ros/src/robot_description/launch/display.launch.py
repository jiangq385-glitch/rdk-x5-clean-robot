#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')

    return LaunchDescription([
        DeclareLaunchArgument('use_joint_state_gui', default_value='false'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.10', '0.0', '0.30', '0', '0', '0', 'base_link', 'camera_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.15', '0.10', '0', '0', '0', 'base_link', 'left_arm_base_link'],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_joint_state_gui),
        ),
    ])
