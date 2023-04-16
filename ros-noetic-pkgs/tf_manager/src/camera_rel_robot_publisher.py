#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose

def publish_point():
	"""
	calculate from frame data(translation and rotation) of 'world'(base of indy) frame to tf data 
	relative to 'base_footprint'(base of rainbow) frame and publish as topic 'frame/indy_rel_rb'. 
	"""
	rospy.init_node('camera_relative_robot')
	pub_rb = rospy.Publisher('frame/camera_rel_rb', Pose, queue_size=1)
	pub_indy = rospy.Publisher('frame/camera_rel_indy', Pose, queue_size=1)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# put object2 frame in poses
		pose_rb = Pose()
		pose_rb.position.x = 0.517
		pose_rb.position.y = -0.134
		pose_rb.position.z = 1.627

		pose_rb.orientation.x = 0.639
		pose_rb.orientation.y = 0.623
		pose_rb.orientation.z = -0.309
		pose_rb.orientation.w = 0.330

		pub_rb.publish(pose_rb)

		# put camera frame w.r.t indy
		pose_indy = Pose()
		pose_indy.position.x = 1.392
		pose_indy.position.y = 0.480
		pose_indy.position.z = 0.851

		pose_indy.orientation.x = 0.644
		pose_indy.orientation.y = 0.613
		pose_indy.orientation.z = - 0.310
		pose_indy.orientation.w = - 0.337

		pub_indy.publish(pose_indy)


if __name__ == '__main__':
	try:
		publish_point()
	except rospy.ROSInterruptException:
		pass