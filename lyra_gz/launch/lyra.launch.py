import launch
import launch_ros.actions

import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
#from ament_index_python.packages import get_package_share_directory

def generate_launch_description():#{

	#Launch Gazebo:
	#TODO For the time being the default gazebo_ros launch file is complete garbage and doesn't offer any of the right functionality, so instead we do it manually.
	#gz_srv = launch.actions.IncludeLaunchDescription(
	#	launch.launch_description_sources.PythonLaunchDescriptionSource(
 # 	get_package_share_directory('gazebo_ros') + '/launch/empty_world.launch.py'
#)
	#)
	gz_srv = launch.actions.ExecuteProcess(
		cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
		output='screen'
	)
	
	#TODO Is there a way to fit this into the GZ server call (or determine that it is impossible) (or get gazebo_ros to do it for me)
	gz_gui = launch.actions.ExecuteProcess(
		cmd=['gzclient'],
		output='screen'
	)
	
	#TODO Add robot_description support (RVIZ, tf, etc) when the Angels of ROS figure out exactly what they want that to *be*.
	#hand_spawner = launch_ros.actions.Node(
	#	package='gazebo_ros',
	#	node_executable='spawn_entity',
	#	output='screen',
	#	arguments='test'
	#)
	
	#TODO Make it a node call
	#TODO Why does the model spawn well-formed if picked from the GUI but cause a crash when spawned in launch?
	hand_spawner = launch.actions.ExecuteProcess(
		cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'hand', '-file', '/home/chrysalis/chrysalis_ws/src/lyra_gz/mdl/mpl_haptix_right_forearm/model.config', '-x', '0', '-y', '0', '-z', '0'],
		output='screen'
	)

	return launch.LaunchDescription([
		gz_srv,
		gz_gui,
		#hand_spawner
	])
#}
