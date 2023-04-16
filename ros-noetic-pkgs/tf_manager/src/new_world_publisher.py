#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose

def publish_point():
	"""
	calculate from frame data(translation and rotation) of 'new_world' frame to tf data
	and publish as topic 'frame/new_world'. 
	"""
	rospy.init_node('new_world_publisher')
	pub = rospy.Publisher('frame/new_world', Pose, queue_size=1)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# put object2 frame in poses
		pose_object = Pose()
		pose_object.position.x = 0.0
		pose_object.position.y = 0.5
		pose_object.position.z = 0.1

		qo = tf.transformations.quaternion_from_euler(0, 0, 0)
		pose_object.orientation.x = qo[0]
		pose_object.orientation.y = qo[1]
		pose_object.orientation.z = qo[2]
		pose_object.orientation.w = qo[3]

		pub.publish(pose_object)
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_point()
	except rospy.ROSInterruptException:
		pass