#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('robot_mqtt_bridge')
    default_config = PathJoinSubstitution([package_share, 'config', 'mqtt_bridge.yaml'])

    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML parameters for robot_mqtt_bridge',
        ),
        DeclareLaunchArgument(
            'mqtt_host',
            default_value=EnvironmentVariable('MQTT_HOST', default_value='127.0.0.1'),
            description='MQTT broker host used by both ROS2 and the mini program',
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value=EnvironmentVariable('MQTT_PORT', default_value='1883'),
            description='MQTT broker port, usually 1883 or 8883 with TLS',
        ),
        DeclareLaunchArgument(
            'mqtt_username',
            default_value=EnvironmentVariable('MQTT_USERNAME', default_value=''),
            description='MQTT username',
        ),
        DeclareLaunchArgument(
            'mqtt_password',
            default_value=EnvironmentVariable('MQTT_PASSWORD', default_value=''),
            description='MQTT password',
        ),
        DeclareLaunchArgument(
            'mqtt_use_tls',
            default_value=EnvironmentVariable('MQTT_USE_TLS', default_value='false'),
            description='Whether ROS2 should connect with TLS',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value=EnvironmentVariable('ROBOT_ID', default_value='digua-x5-001'),
            description='Robot/device id used in MQTT topics',
        ),
        DeclareLaunchArgument(
            'topic_prefix',
            default_value=EnvironmentVariable('MQTT_TOPIC_PREFIX', default_value='robot/digua-x5-001'),
            description='MQTT topic prefix, e.g. robot/digua-x5-001',
        ),
    ]

    node = Node(
        package='robot_mqtt_bridge',
        executable='mqtt_bridge_node',
        name='robot_mqtt_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'mqtt_host': LaunchConfiguration('mqtt_host'),
                'mqtt_port': ParameterValue(LaunchConfiguration('mqtt_port'), value_type=int),
                'mqtt_username': LaunchConfiguration('mqtt_username'),
                'mqtt_password': LaunchConfiguration('mqtt_password'),
                'mqtt_use_tls': ParameterValue(LaunchConfiguration('mqtt_use_tls'), value_type=bool),
                'robot_id': LaunchConfiguration('robot_id'),
                'topic_prefix': LaunchConfiguration('topic_prefix'),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [node])
