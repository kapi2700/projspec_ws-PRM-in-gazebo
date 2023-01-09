import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
#	pkg_name="PKG_NAME"
#	urdf_file="URDF_FILE"
#	pkg_share=get_package_share_directory(pkg_name)
	
	# Bridge
	bridge = Node(
	    package='ros_ign_bridge',
	    executable='parameter_bridge',
	    arguments=['/model/vehicle_blue1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
		       '/model/vehicle_blue1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
		       '/model/vehicle_blue2/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
		       '/model/vehicle_blue2/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
		       '/world/testwrld/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
		       '/world/testwrld/dynamic_pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
	    output='screen'
	)
	return LaunchDescription([
		bridge,
	])
