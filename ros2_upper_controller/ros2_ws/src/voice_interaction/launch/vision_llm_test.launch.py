from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/home/sunrise'),
        DeclareLaunchArgument('prompt', default_value='请用中文简短描述这张图片里的主要内容，并判断画面是否正常可识别。'),
        Node(
            package='voice_interaction',
            executable='vision_llm_test',
            name='vision_llm_test',
            output='screen',
            arguments=[
                '--output-dir', LaunchConfiguration('output_dir'),
                '--prompt', LaunchConfiguration('prompt'),
            ],
        ),
    ])
