#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose

def publish_point():
	"""
	calculate from frame data(translation and rotation) of 'object' frame to tf data
	and publish as topic 'frame/object'. 
	"""
	rospy.init_node('object_publisher')
	pub = rospy.Publisher('frame/object', Pose, queue_size=1)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		# put object frame in poses
		pose_object = Pose()
		pose_object.position.x = 0.0
		pose_object.position.y = 0.696
		pose_object.position.z = 0.260
		q = tf.transformations.quaternion_from_euler(1.57, 0, 3.14)
		pose_object.orientation.x = q[0]
		pose_object.orientation.y = q[1]
		pose_object.orientation.z = q[2]
		pose_object.orientation.w = q[3]
		pub.publish(pose_object)
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_point()
	except rospy.ROSInterruptException:
		pass