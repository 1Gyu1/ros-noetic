#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose, PoseArray

def publish_point():
	"""
	Make start / mid / end point and segments between two points, 
	and publish data of these points as PoseArray.
	"""
	rospy.init_node('trajectory_publisher')
	pub = rospy.Publisher('traj', PoseArray, queue_size=1)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Number of segments
		n = 10
		start = [0.69, -0.16, 0.69, 1.57, 0, 1.57]
		mid = [0.46, 0.63, 0.6, 2.07, 0, 2.57]
		end = [0.16, 0.73, 0.5, 2.07, 0, 2.57]
		point_name = [start, mid, end]

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
		pub.publish(pose_pointlist)
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_point()
	except rospy.ROSInterruptException:
		pass