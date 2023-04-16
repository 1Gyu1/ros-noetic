#!/usr/bin/env python3
import rospy, roslib, tf

from PyKDL import *
from math import pi
from geometry_msgs.msg import Pose, PoseArray

def publish_link():
	"""
	Receive tf data(it suggests position of link frame relative to previous link frame) from indy7 robot,
	and publish data of these points as PoseArray.
	"""
	rospy.init_node('link_publisher')
	pub = rospy.Publisher('link', PoseArray, queue_size=1)
	rate = rospy.Rate(10)
	listener = tf.TransformListener()
	trans = [0 for j in range(8)]
	rot = [0 for k in range(8)]

	while not rospy.is_shutdown():

		pose_linklist = PoseArray()

		while True:
			try:
				(trans[0], rot[0]) = listener.lookupTransform('/world', '/link0', rospy.Time(0))
				(trans[1], rot[1]) = listener.lookupTransform('/link0', '/link1', rospy.Time(0))
				(trans[2], rot[2]) = listener.lookupTransform('/link1', '/link2', rospy.Time(0))
				(trans[3], rot[3]) = listener.lookupTransform('/link2', '/link3', rospy.Time(0))
				(trans[4], rot[4]) = listener.lookupTransform('/link3', '/link4', rospy.Time(0))
				(trans[5], rot[5]) = listener.lookupTransform('/link4', '/link5', rospy.Time(0))
				(trans[6], rot[6]) = listener.lookupTransform('/link5', '/link6', rospy.Time(0))
				(trans[7], rot[7]) = listener.lookupTransform('/link6', '/tcp', rospy.Time(0))
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

		for i in range(8):
			pose_link = Pose()

			if i == 0 :
				pose_link.position.x = trans[i][0] + 0.5
			else :
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