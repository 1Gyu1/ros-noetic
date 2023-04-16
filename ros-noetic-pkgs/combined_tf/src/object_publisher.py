#!/usr/bin/env python3
import rospy, roslib, tf
from math import pi
from geometry_msgs.msg import Pose, PoseArray
from im_pickit_msgs.msg import Object, ObjectArray

"""
Subscribe positions of objects, and transform to PoseArray type.
"""

def publish_point(object_poses):
	pose_pointlist = PoseArray()
	for i in range(object_poses.n_valid_objects):
		# put object frame in poses.
		pose_point = Pose()
		pose_point.position.x = object_poses.objects[i].object_tf.transform.translation.x
		pose_point.position.y = object_poses.objects[i].object_tf.transform.translation.y
		pose_point.position.z = object_poses.objects[i].object_tf.transform.translation.z
		pose_point.orientation.x = object_poses.objects[i].object_tf.transform.rotation.x
		pose_point.orientation.y = object_poses.objects[i].object_tf.transform.rotation.y
		pose_point.orientation.z = object_poses.objects[i].object_tf.transform.rotation.z
		pose_point.orientation.w = object_poses.objects[i].object_tf.transform.rotation.w

		pose_pointlist.poses.append(pose_point)
		print (pose_point)

	pub.publish(pose_pointlist)
	print (object_poses.n_valid_objects)


if __name__ == '__main__':
	rospy.init_node('object_publisher')
	pub = rospy.Publisher('frame/objects', PoseArray, queue_size=1)
	rospy.sleep(0.1)
	rospy.Subscriber('/pickit/objects_wrt_robot_frame', ObjectArray, publish_point)
	rospy.spin()