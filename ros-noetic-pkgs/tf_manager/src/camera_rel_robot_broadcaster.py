#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose

def indy_to_tf(indy):
	"""
	calculate from frame data(translation and rotation) of 'world' frame to tf data
	"""
	pose_world = indy
	br = tf.TransformBroadcaster()
	br.sendTransform(
		(pose_world.position.x, pose_world.position.y, 
		pose_world.position.z), 
		(pose_world.orientation.x, pose_world.orientation.y, 
		pose_world.orientation.z, pose_world.orientation.w), 
		rospy.Time.now(), 'camera', 'world')

def rb_to_tf(rb):
	"""
	calculate from frame data(translation and rotation) of 'world' frame to tf data
	"""
	pose_base_footprint = rb
	br = tf.TransformBroadcaster()
	br.sendTransform(
		(pose_base_footprint.position.x, pose_base_footprint.position.y, 
		pose_base_footprint.position.z), 
		(pose_base_footprint.orientation.x, pose_base_footprint.orientation.y, 
		pose_base_footprint.orientation.z, pose_base_footprint.orientation.w), 
		rospy.Time.now(), 'base_footprint', 'camera')

if __name__ == '__main__':
	rospy.init_node('camera_relative_robot_broadcaster')
	rospy.Subscriber('frame/camera_rel_rb', Pose, rb_to_tf)
	rospy.Subscriber('frame/camera_rel_indy', Pose, indy_to_tf)
	rospy.spin()