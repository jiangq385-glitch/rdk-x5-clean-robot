from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sample_rate', default_value='16000'),

        Node(
            package='sherpa_onnx_ros2',
            executable='asr_node',
            name='asr_node',
            output='screen',
            parameters=[{
                'input_device': 'default',
                'sample_rate': LaunchConfiguration('sample_rate'),
                'channels': 1,
                'samples_per_chunk': 1600,
                'topic_final': '/asr/final_text',
                'topic_partial': '/asr/partial_text',
                # 云端 ASR 配置
                'provider': 'openai_compatible',
                'api_base_url': 'https://api.siliconflow.cn/v1',
                'api_key': 'sk-nfvumctiqgdpxvxhkbvcvukiuoamtraeufmwlnrwmphysgak',  # 替换成你的 API Key
                'api_model': 'FunAudioLLM/SenseVoiceSmall',
                'language': 'zh',
                'silence_threshold': 1000,
                'min_speech_ms': 400,
                'trailing_silence_ms': 1200,
                'max_utterance_ms': 10000,
                'session_timeout_sec': 15.0,
            }],
        )
    ])
