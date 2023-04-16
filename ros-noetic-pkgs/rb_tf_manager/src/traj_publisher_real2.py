#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool, Float32MultiArray

flag = False

def flag_cb(traj_flag):
	global flag
	print("callback")
	print(traj_flag.data)
	flag = traj_flag.data

def publish_point(object_pose):
	global flag
	# number of segments
	n = 8

	start = [0.690, -0.157, 0.460, 1.57, 0, 1.57]
	mid1 = [0.690, -0.157, 0.676, 1.57, 0, 1.57]
	mid2 = [0.134, 0.696, 0.676, 1.57, 0, 3.14]

	X = object_pose.position.x
	Y = object_pose.position.y
	Z = object_pose.position.z
	e = tf.transformations.euler_from_quaternion(
		[object_pose.orientation.x, object_pose.orientation.y,
		object_pose.orientation.z, object_pose.orientation.w])

	object_frame = [X, Y, Z, e[0], e[1], e[2]]

	# start = [0.50, -0.16, 0.50, -3.14, 0, 3.14]
	# mid = [0.36, 0.33, 0.6, -3.14, 1.57, 3.14]
	# end = [0.16, 0.53, 0.7, -3.14, 1.57, 5]

	point_name = [start, mid1, mid2, object_frame]

	pose_pointlist = PoseArray()
	for j in range(len(point_name) - 1):
		for i in range(n + 1):
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

	print("publish part")
	print(flag)
	if not flag:
		pub.publish(pose_pointlist)
	# rate.sleep()

if __name__ == '__main__':
	rospy.init_node('trajectory_publisher')
	rospy.Subscriber('frame/object', Pose, publish_point)
	rospy.Subscriber("/traj_flag", Bool, flag_cb)
	pub = rospy.Publisher('/traj_array', PoseArray, queue_size=1)
	rospy.spin()
	# if flag:
	# 	break