import os
import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext


def get_latest_rosbag(base_dir):
	# st_ctime is not the creation time but the time of the last metadata change
	first = True
	for folder in os.scandir(base_dir):
		if folder.is_dir():
			if first:
				latest = folder
				first = False
			else:
				if folder.stat().st_ctime > latest.stat().st_ctime:
					latest = folder
	return latest.path

def launch_setup(context, *args, **kwargs):
	base_dir = LaunchConfiguration('folder').perform(context)
	file_name = LaunchConfiguration('file').perform(context)
	if file_name == 'none':
		path = get_latest_rosbag(base_dir)
	else:
		path = os.path.join(base_dir, file_name)
	print(f"\nPlaying {path}\n")
	rosbag_play = ExecuteProcess(
		cmd=['ros2', 'bag', 'play', path],
		output='screen'
		)
	return [rosbag_play]


def generate_launch_description():
	odom_tf_pub_dir = get_package_share_directory('odom_tf_pub')
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', os.path.join(get_package_share_directory('ros2_bag'), 'config/rosbag_conf.rviz')],
		output='screen')
	#tf2_node = Node(
	#	package='tf2_ros',
	#	executable = 'static_transform_publisher',
	#	arguments = ["0", "0", "0", "0", "0", "0", "odom_frame", "map"]
	#	)
	odom_tf_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([odom_tf_pub_dir, '/odom_tf_pub.launch.py']),
		launch_arguments={'use_t265': 'False', 'use_zed2': 'True'}.items(),
	)
	return LaunchDescription([
		rviz_node,
		odom_tf_launch,
		DeclareLaunchArgument('folder', default_value='/root/rosbags'),
		DeclareLaunchArgument('file', default_value='none'),
		OpaqueFunction(function = launch_setup)
		])
         
