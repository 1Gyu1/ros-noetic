#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool

flag = False

def flag_cb(traj_flag):
	global flag
	print("callback")
	print(traj_flag.data)
	flag = traj_flag.data

def publish_point():
	global flag
	rospy.init_node('trajectory_publisher')
	pub = rospy.Publisher('/traj_array', PoseArray, queue_size=1)
	rospy.Subscriber("/traj_flag", Bool, flag_cb)
	rate = rospy.Rate(1)

	n = 10
	start = [0.69, -0.16, 0.69, 1.57, 0.0, 1.57]
	mid = [0.46, 0.63, 0.6, 1.07, 0.0, 2.57]
	end = [0.16, 0.73, 0.5, 1.07, 0.0, 2.57]
	# start = [0.50, -0.16, 0.50, -3.14, 0, 3.14]
	# mid = [0.36, 0.33, 0.6, -3.14, 1.57, 3.14]
	# end = [0.16, 0.53, 0.7, -3.14, 1.57, 5]
	point_name = [start, mid, end]

	pose_end = Pose()
	
	
	pose_end.position.x = end[0]
	pose_end.position.y = end[1]
	pose_end.position.z = end[2]
	tmp = tf.transformations.quaternion_from_euler(
			end[3],end[4], end[5])
	pose_end.orientation.x = tmp[0]
	pose_end.orientation.y = tmp[1]
	pose_end.orientation.z = tmp[2]
	pose_end.orientation.w = tmp[3]

	pose_pointlist = PoseArray()
	for j in range(len(point_name) - 1):
		for i in range(n):
			# put point frame in poses
			pose_point = Pose()
			pose_point.position.x = point_name[j][0] + (
				point_name[j + 1][0] - point_name[j][0])* i / n
			pose_point.position.y = point_name[j][1] + (
				point_name[j + 1][1] - point_name[j][1])* i / n
			pose_point.position.z = point_name[j][2] + (
				point_name[j + 1][2] - point_name[j][2])* i / n
			q = tf.transformations.quaternion_from_euler(
				point_name[j][3] + 
				(point_name[j + 1][3] - point_name[j][3])* i / n, 
				point_name[j][4] + 
				(point_name[j + 1][4] - point_name[j][4])* i / n, 
				point_name[j][5] + 
				(point_name[j + 1][5] - point_name[j][5])* i / n)
			pose_point.orientation.x = q[0]
			pose_point.orientation.y = q[1]
			pose_point.orientation.z = q[2]
			pose_point.orientation.w = q[3]
			pose_pointlist.poses.append(pose_point)

	pose_pointlist.poses.append(pose_end)
	print("publish part")
	print(flag)
	if not flag:
		pub.publish(pose_pointlist)

	rate.sleep()
	

if __name__ == '__main__':
	while not rospy.is_shutdown():
		publish_point()
		if flag:
			break

		
