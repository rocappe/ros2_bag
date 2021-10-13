import os
import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext

# base usage: ros2 launch ros2_bag record_bag.launch.py
# it records a bag called test, in the folder /root/rosbags that is created if not present
# complete usage: ros2 launch ros2_bag record_bag.launch.py folder:=/root/rosbags file:=test overwrite:=false
def launch_setup(context, *args, **kwargs):
	qos_override_path = os.path.join(get_package_share_directory('ros2_bag'), 'config/qos_override.yaml')
	base_dir = LaunchConfiguration('folder').perform(context)
	file_name = LaunchConfiguration('file').perform(context)
	ow = LaunchConfiguration('overwrite').perform(context)
	if not os.path.isdir(base_dir):
		os.mkdir(base_dir)
	rosbag_path = os.path.join(base_dir, file_name)
	if os.path.isdir(rosbag_path):
		# if the file has to be overriden do it, otherwise create one with the same name
		# plus an increasing integer from 1 to 100 at the end
		if ow == 'true':
			shutil.rmtree(rosbag_path)
			new_rosbag_path= rosbag_path
		else:
			for i in range(100):
				new_rosbag_path = rosbag_path + str(i+1)
				if not os.path.isdir(new_rosbag_path):
					break
	else:
		new_rosbag_path = rosbag_path
	print(f"Saving robag in {new_rosbag_path}")
	rosbag_record = ExecuteProcess(
		cmd=['ros2', 'bag', 'record',
		'/tf', '/tf_static', '/camera/depth/color/points', '/scan', '/odom', '--qos-profile-overrides-path', qos_override_path,
		'-o', new_rosbag_path],
		output='screen'
		)
	rosbag_record_split = ExecuteProcess(
		cmd=['ros2', 'bag', 'record',
		'/local_costmap/costmap', '/global_costmap/costmap', '/behavior_tree_log',
		'/camera/pose',	'/tf', '/local_plan', '/received_global_plan',
		 '/transformed_global_plan', '/map', '/tf_static', '/scan',
		'/goal_update', '/goal_pose', '-b', '500000000', '--qos-profile-overrides-path', qos_override_path,
		'-o', new_rosbag_path],
		output='screen'
		)
	return [rosbag_record]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument('folder', default_value='/root/rosbags'),
		DeclareLaunchArgument('file', default_value='test'),
		DeclareLaunchArgument('overwrite', default_value='false'),
		OpaqueFunction(function = launch_setup)
		])
         
