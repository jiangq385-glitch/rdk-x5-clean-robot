#!/usr/bin/python3

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
先 source：cd /home/sunrise/my_ros && source install/setup.bash
启动导航：ros2 launch robot_nav2 nav2_launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_nav2_dir = get_package_share_directory('robot_nav2')
    my_mapping_dir = get_package_share_directory('my_mapping_pkg')
    robot_description_dir = get_package_share_directory('robot_description')
    lslidar_driver_dir = get_package_share_directory('lslidar_driver')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    nav2_param_path = LaunchConfiguration('params_file')
    laser_port = LaunchConfiguration('laser_port')
    model = LaunchConfiguration('model')
    use_initial_pose = LaunchConfiguration('use_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    fallback_global_localization = LaunchConfiguration('fallback_global_localization')

    default_map_yaml = os.path.join(my_mapping_dir, 'maps', 'my_map_6.20.yaml')
    default_nav2_params = os.path.join(robot_nav2_dir, 'config', 'nav2.yaml')
    default_model_path = os.path.join(robot_description_dir, 'urdf', 'cleaner_robot.urdf')
    default_lslidar_params = os.path.join(lslidar_driver_dir, 'params', 'lidar_uart_ros2', 'lsn10.yaml')

    robot_description = {
        'robot_description': ParameterValue(Command(['cat ', model]), value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map file to load',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to param file to load',
        ),
        DeclareLaunchArgument(
            'laser_port',
            default_value='/dev/wheeltec_lidar',
            description='Lidar UART device path, e.g. /dev/wheeltec_lidar',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file',
        ),
        DeclareLaunchArgument(
            'use_initial_pose',
            default_value='false',
            description='Publish /initialpose before falling back to global localization',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial robot x in map frame, meters',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial robot y in map frame, meters',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial robot yaw in map frame, radians',
        ),
        DeclareLaunchArgument(
            'fallback_global_localization',
            default_value='true',
            description='Use AMCL global localization if initial pose does not converge',
        ),


        # 激光雷达驱动：发布 /scan
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

        # TF：发布 base_link/base_footprint/... 等
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),

        Node(
            package='robot_nav2',
            executable='auto_relocalize',
            name='auto_relocalize',
            output='screen',
            parameters=[{
                'use_initial_pose': ParameterValue(use_initial_pose, value_type=bool),
                'initial_pose_x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose_y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose_yaw': ParameterValue(initial_pose_yaw, value_type=float),
                'fallback_global_localization': ParameterValue(fallback_global_localization, value_type=bool),
                'spin_speed': 0.35,
                'drive_speed': 0.06,
                'motion_duration': 60.0,
                'min_motion_before_converged': 18.0,
                'converged_required_count': 3,
                'max_retries': 3,
                'retry_pause': 2.0,
                'max_wait_after_motion': 15.0,
                'covariance_xy_threshold': 0.50,
                'covariance_yaw_threshold': 2.4,
            }],
        ),
    ])

"""

按下面步骤做，一般就能恢复：

1）先把残留进程清干净（关键）
在一个终端执行（会把残留的 nav2/launch/container 一次性清掉）：

pkill -f "ros2 launch robot_nav2 nav2_launch.py" || true
pkill -f "component_container_isolated.*__node:=nav2_container" || true
如果你之前是直接关了终端/VS Code，有时驱动节点也会变成“孤儿进程”，可以再补一刀（可选）：

pkill -f lslidar_driver_node || true
pkill -f robot_state_publisher || true
2）重新启动导航



启动后确认只有一个容器：

ros2 node list | grep nav2_container（应该只出现一次）
"""
