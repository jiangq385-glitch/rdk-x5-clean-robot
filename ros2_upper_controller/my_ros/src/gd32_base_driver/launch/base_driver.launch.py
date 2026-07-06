#!/usr/bin/env python3
"""启动文件"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
#ros2 launch gd32_base_driver base_driver.launch.py port:=/dev/wheeltec_controller baud:=115200 debug_tx:=true debug_rx:=true

def generate_launch_description():
    pkg = 'gd32_base_driver'
    pkg_path = get_package_share_directory(pkg)

    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    debug_tx = LaunchConfiguration('debug_tx')
    debug_rx = LaunchConfiguration('debug_rx')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/wheeltec_controller',
            description='UART device path, e.g. /dev/wheeltec_controller'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='UART baudrate, e.g. 115200'
        ),
        DeclareLaunchArgument(
            'debug_tx',
            default_value='false',
            description='Log TX frames in hex (true/false)'
        ),
        DeclareLaunchArgument(
            'debug_rx',
            default_value='false',
            description='Log RX packets in hex (true/false)'
        ),
        # 底盘驱动
        Node(package=pkg, executable='chassis_driver',
             name='chassis_driver', output='screen',
             parameters=[{
                    'port': ParameterValue(port, value_type=str),
                 'baud': ParameterValue(baud, value_type=int),
                 'debug_tx': ParameterValue(debug_tx, value_type=bool),
                 'debug_rx': ParameterValue(debug_rx, value_type=bool),
             }]),

        # EKF 融合
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter', output='screen',
             parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml')],
             remappings=[('odom0', '/odom'),
                         ('imu0', '/imu')]),
    ])