#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

"""
cd /home/sunrise/my_ros
source install/setup.bash
ros2 launch robot_bringup slam.launch.py

# 手动启动键盘控制（另开一个终端）
cd /home/sunrise/my_ros
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

#保存地图
ros2 run nav2_map_server map_saver_cli -f ~/my_ros/src/my_mapping_pkg/maps/my_map
"""
def generate_launch_description() -> LaunchDescription:
    base_port = LaunchConfiguration('base_port')
    base_baud = LaunchConfiguration('base_baud')
    laser_port = LaunchConfiguration('laser_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    model = LaunchConfiguration('model')

    default_model_path = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'urdf', 'cleaner_robot.urdf']
    )
    default_slam_params_file = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'config', 'slam.yaml']
    )
    default_lslidar_params = PathJoinSubstitution(
        [FindPackageShare('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10.yaml']
    )
    ekf_params_file = os.path.join(
        get_package_share_directory('gd32_base_driver'), 'config', 'ekf.yaml'
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/wheeltec_controller',
            description='Base/chassis UART device path, e.g. /dev/wheeltec_controller',
        ),
        DeclareLaunchArgument(
            'base_baud',
            default_value='115200',
            description='Base/chassis UART baudrate, e.g. 115200',
        ),
        DeclareLaunchArgument(
            'laser_port',
            default_value='/dev/wheeltec_lidar',
            description='Lidar UART device path, e.g. /dev/wheeltec_lidar',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params_file,
            description='Full path to slam_toolbox params yaml',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file',
        ),
    ]

    robot_description = {
        'robot_description': ParameterValue(Command(['cat ', model]), value_type=str)
    }

    nodes = [
        # 底盘驱动
        Node(
            package='gd32_base_driver',
            executable='chassis_driver',
            name='chassis_driver',
            output='screen',
            parameters=[{
                'port': ParameterValue(base_port, value_type=str),
                'baud': ParameterValue(base_baud, value_type=int),
            }],
        ),

        # 激光重采样：把抖动的原始 /scan 统一成固定长度 /scan_fixed
        Node(
            package='robot_bringup',
            executable='scan_resampler',
            name='scan_resampler',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_fixed',
                'target_count': 692,
            }],
        ),

        # 激光雷达驱动（lslidar_driver）
        Node(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[
                default_lslidar_params,
                {
                    'serial_port_': ParameterValue(laser_port, value_type=str),
                    'frame_id': 'laser_link',
                },
            ],
        ),

        

        # 机器人 TF 发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),

        # 里程计融合，供 slam_toolbox 计算 odom 位姿
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        ),

        # SLAM 建图（include slam_toolbox 官方 online_async_launch.py）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items(),
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)