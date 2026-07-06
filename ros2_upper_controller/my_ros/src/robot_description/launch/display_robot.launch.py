
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
# source /opt/ros/humble/setup.bash
# source install/setup.bash
# ros2 launch robot_description display_robot.launch.py

def generate_launch_description() -> LaunchDescription:
	model = LaunchConfiguration('model')
	gui = LaunchConfiguration('gui')
	rviz = LaunchConfiguration('rviz')
	use_sim_time = LaunchConfiguration('use_sim_time')

	default_model_path = PathJoinSubstitution(
		[FindPackageShare('robot_description'), 'urdf', 'full_robot.urdf']
	)

	declared_arguments = [
		DeclareLaunchArgument(
			'model',
			default_value=default_model_path,
			description='Absolute path to robot URDF file',
		),
		DeclareLaunchArgument(
			'gui',
			default_value='true',
			description='Start joint_state_publisher_gui if true, else joint_state_publisher',
		),
		DeclareLaunchArgument(
			'rviz',
			default_value='true',
			description='Start rviz2',
		),
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use simulation time',
		),
	]

	robot_description = {
		'robot_description': ParameterValue(Command(['cat ', model]), value_type=str)
	}

	nodes = [
		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			parameters=[robot_description, {'use_sim_time': use_sim_time}],
		),
		Node(
			package='joint_state_publisher_gui',
			executable='joint_state_publisher_gui',
			name='joint_state_publisher_gui',
			output='screen',
			condition=IfCondition(gui),
			parameters=[{'use_sim_time': use_sim_time}],
			arguments=[model],
		),
		Node(
			package='joint_state_publisher',
			executable='joint_state_publisher',
			name='joint_state_publisher',
			output='screen',
			condition=UnlessCondition(gui),
			parameters=[{'use_sim_time': use_sim_time}],
			arguments=[model],
		),
		Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			condition=IfCondition(rviz),
		),
	]

	return LaunchDescription(declared_arguments + nodes)