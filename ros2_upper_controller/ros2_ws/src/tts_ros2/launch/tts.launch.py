from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tts_ros2',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[
                {'output_device': 'default'},
                {'tmp_dir': '/tmp/edge_tts_audio'},
                {'proxy': ''},
            ]
        )
    ])