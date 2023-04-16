#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose

def publish_point():
	"""
	Make start point and put increment to each parameter(x, y, z, R, P, Y), 
	and publish data of these points as Pose method.
	"""
	rospy.init_node('point1_publisher')
	pub = rospy.Publisher('point/point1', Pose, queue_size=1)
	rate = rospy.Rate(2)
	while pub.get_num_connections() < 1:
		# Number of segments
		n = 5
		for i in range(n):
			# put object1 frame in poses
			pose_object = Pose()
			pose_object.position.x = 0.0 + (0.5 / n) * i
			pose_object.position.y = 1.0 - (0.5 / n) * i
			pose_object.position.z = 0.0 + (0.5 / n) * i
			qo = tf.transformations.quaternion_from_euler(
				0, -pi / 2 + (pi / (2 * n)) * i, pi / 2 - (pi / (2 * n)) * i)
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