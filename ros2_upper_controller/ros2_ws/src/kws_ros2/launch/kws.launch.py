from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from kws_ros2.config import (
    DEFAULT_COOLDOWN_SEC,
    DEFAULT_KWS_MODEL_PATH,
    DEFAULT_RESUME_GRACE_SEC,
    DEFAULT_SCORE_THRESHOLD,
    DEFAULT_TRIGGER_COUNT,
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sample_rate', default_value='16000'),
        DeclareLaunchArgument('engine', default_value='bpu_kws'),
        DeclareLaunchArgument('score_threshold', default_value=str(DEFAULT_SCORE_THRESHOLD)),
        DeclareLaunchArgument('trigger_count', default_value=str(DEFAULT_TRIGGER_COUNT)),
        DeclareLaunchArgument('window_ms', default_value='1000'),
        DeclareLaunchArgument('hop_ms', default_value='200'),
        DeclareLaunchArgument('feature_dim', default_value='40'),
        DeclareLaunchArgument('fbank_frame_length_ms', default_value='25.0'),
        DeclareLaunchArgument('fbank_frame_shift_ms', default_value='10.0'),
        DeclareLaunchArgument('fbank_dither', default_value='0.0'),
        DeclareLaunchArgument('kws_model_path', default_value=DEFAULT_KWS_MODEL_PATH),
        DeclareLaunchArgument('cooldown_sec', default_value=str(DEFAULT_COOLDOWN_SEC)),
        DeclareLaunchArgument('resume_grace_sec', default_value=str(DEFAULT_RESUME_GRACE_SEC)),
        DeclareLaunchArgument('takeover_topic', default_value='/voice/takeover'),
        DeclareLaunchArgument('takeover_window_sec', default_value='5.0'),
        DeclareLaunchArgument('voice_idle_timeout_sec', default_value='30.0'),
        Node(
            package='kws_ros2',
            executable='kws_node',
            name='kws_node',
            output='screen',
            parameters=[{
                'input_device': 'default',
                'sample_rate': LaunchConfiguration('sample_rate'),
                'channels': 1,
                'samples_per_chunk': 1600,
                'wakeup_topic': '/voice/wakeup',
                'voice_state_topic': '/voice/session_state',
                'engine': LaunchConfiguration('engine'),
                'wake_words': ['你好番薯', '番薯番薯'],
                'cooldown_sec': LaunchConfiguration('cooldown_sec'),
                'resume_grace_sec': LaunchConfiguration('resume_grace_sec'),
                'score_threshold': LaunchConfiguration('score_threshold'),
                'trigger_count': LaunchConfiguration('trigger_count'),
                'window_ms': LaunchConfiguration('window_ms'),
                'hop_ms': LaunchConfiguration('hop_ms'),
                'feature_dim': LaunchConfiguration('feature_dim'),
                'fbank_frame_length_ms': LaunchConfiguration('fbank_frame_length_ms'),
                'fbank_frame_shift_ms': LaunchConfiguration('fbank_frame_shift_ms'),
                'fbank_dither': LaunchConfiguration('fbank_dither'),
                'kws_model_path': LaunchConfiguration('kws_model_path'),
            }],
        ),
        Node(
            package='kws_ros2',
            executable='wake_manager',
            name='wake_manager',
            output='screen',
            parameters=[{
                'wakeup_topic': '/voice/wakeup',
                'voice_state_topic': '/voice/session_state',
                'takeover_topic': LaunchConfiguration('takeover_topic'),
                'takeover_window_sec': LaunchConfiguration('takeover_window_sec'),
                'idle_timeout_sec': LaunchConfiguration('voice_idle_timeout_sec'),
            }],
        ),
    ])
