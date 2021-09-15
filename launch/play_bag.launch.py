import os
import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext


def get_latest_rosbag(base_dir):
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
	base_dir = "/root/rosbags"
	file = LaunchConfiguration('file').perform(context)
	if file == 'none':
		path = get_latest_rosbag(base_dir)
	else:
		path = os.path.join(base_dir, file)
	print(f"\nPlaying {path}\n")
	rosbag_play = ExecuteProcess(
		cmd=['ros2', 'bag', 'play', path],
		output='screen'
		)
	return [rosbag_play]


def generate_launch_description():
	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d', os.path.join(get_package_share_directory('ros2_bag'), 'config/rosbag_conf.rviz')],
		output='screen')
	tf2_node = Node(
		package='tf2_ros',
		executable = 'static_transform_publisher',
		arguments = ["0", "0", "0", "0", "0", "0", "odom_frame", "map"]
		)
	return LaunchDescription([
		rviz_node,
		#tf2_node,
		DeclareLaunchArgument('file', default_value='none'),
		OpaqueFunction(function = launch_setup)
		])
         
