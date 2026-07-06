from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vision_launch = os.path.join(
        get_package_share_directory('robot_vision'),
        'launch',
        'robot_vision.launch.py',
    )
    nav2_launch = os.path.join(
        get_package_share_directory('robot_nav2'),
        'launch',
        'nav2_launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_nav2', default_value='false'),
        DeclareLaunchArgument('start_vision', default_value='false'),
        DeclareLaunchArgument('skip_vision', default_value='true'),
        DeclareLaunchArgument('auto_start_area', default_value=''),
        DeclareLaunchArgument('yolo_wait_sec', default_value='3.0'),
        DeclareLaunchArgument('nav_server_wait_sec', default_value='180.0'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('detections_topic', default_value='/vision/detections_json'),
        DeclareLaunchArgument('use_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            launch_arguments={
                'use_initial_pose': LaunchConfiguration('use_initial_pose'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            condition=IfCondition(LaunchConfiguration('start_vision')),
        ),
        Node(
            package='robot_test',
            executable='clean_table_test_node',
            name='clean_table_test_node',
            output='screen',
            parameters=[{
                'auto_start_area': LaunchConfiguration('auto_start_area'),
                'yolo_wait_sec': LaunchConfiguration('yolo_wait_sec'),
                'nav_server_wait_sec': LaunchConfiguration('nav_server_wait_sec'),
                'skip_vision': LaunchConfiguration('skip_vision'),
                'image_topic': LaunchConfiguration('image_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
            }],
        ),
    ])
