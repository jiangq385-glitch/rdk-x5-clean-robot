#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([
        get_package_share_directory('my_arm_control'),
        'config',
        'arm_controller.yaml',
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            'arm_config',
            default_value=config_file,
            description='YAML parameters for the arm controller, joint state node',
        ),
        DeclareLaunchArgument(
            'enable_vision_pick',
            default_value='false',
            description='Enable legacy direct vision-to-arm pick node for debugging',
        ),
    ]


    nodes = [
        Node(
            package='my_arm_control',
            executable='arm_controller_node',
            name='arm_controller',
            output='screen',
            parameters=[LaunchConfiguration('arm_config')],
        ),
        Node(
            package='my_arm_control',
            executable='joint_state_node',
            name='joint_state_node',
            output='screen',
            parameters=[LaunchConfiguration('arm_config')],
        ),
        Node(
            package='my_arm_control',
            executable='vision_pick_node',
            name='vision_pick_node',
            output='screen',
            parameters=[LaunchConfiguration('arm_config')],
            condition=IfCondition(LaunchConfiguration('enable_vision_pick')),
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)
