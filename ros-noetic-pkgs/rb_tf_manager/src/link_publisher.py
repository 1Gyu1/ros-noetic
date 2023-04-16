#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose, PoseArray

def publish_link():
	"""
	Receive tf data(it suggests position of link frame relative to previous link frame) from rb robot,
	and publish data of these points as PoseArray.
	"""
	rospy.init_node('link_publisher')
	pub = rospy.Publisher('link', PoseArray, queue_size=1)
	rate = rospy.Rate(10)
	listener = tf.TransformListener()
	trans = [0 for j in range(6)]
	rot = [0 for k in range(6)]

	while not rospy.is_shutdown():
		while True:
			try:
				(trans[0], rot[0]) = listener.lookupTransform('/Axis_0', '/base', rospy.Time(0))
				(trans[1], rot[1]) = listener.lookupTransform('/base', '/shoulder', rospy.Time(0))
				(trans[2], rot[2]) = listener.lookupTransform('/shoulder', '/elbow', rospy.Time(0))
				(trans[3], rot[3]) = listener.lookupTransform('/elbow', '/wrist1', rospy.Time(0))
				(trans[4], rot[4]) = listener.lookupTransform('/wrist1', '/wrist2', rospy.Time(0))
				(trans[5], rot[5]) = listener.lookupTransform('/wrist2', '/wrist3', rospy.Time(0))
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		pose_linklist = PoseArray()
		for i in range(6):
			pose_link = Pose()
			pose_link.position.x = trans[i][0]
			pose_link.position.y = trans[i][1]
			pose_link.position.z = trans[i][2]
			pose_link.orientation.x = rot[i][0]
			pose_link.orientation.y = rot[i][1]
			pose_link.orientation.z = rot[i][2]
			pose_link.orientation.w = rot[i][3]
			pose_linklist.poses.append(pose_link)
		pub.publish(pose_linklist)
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_link()
	except rospy.ROSInterruptException:
		pass