from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_model_path = os.path.join(
        get_package_share_directory('robot_vision'),
        'models',
        'object_x5_rdk_bayese_640x640_nv12.bin'
    )

    config_file = os.path.join(
        get_package_share_directory('robot_vision'),
        'config',
        'vision_pipeline.yaml'
    )

    camera_node = Node(
        package='robot_vision',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[config_file]
    )

    camera_info_node = Node(
        package='robot_vision',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        parameters=[{
            'camera_frame_id': 'camera_link',
            'camera_info_topic': '/camera/camera_info',
            'width': 640,
            'height': 480,
            'fx': 525.0,
            'fy': 525.0,
            'cx': 320.0,
            'cy': 240.0,
            'fps': 30.0,
        }]
    )

    detector_node = Node(
        package='robot_vision',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[
            config_file,
            {'model_path': LaunchConfiguration('model_path')},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to the YOLO model file used by robot_vision'
        ),
        camera_node,
        camera_info_node,
        detector_node,
    ])
