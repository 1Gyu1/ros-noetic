#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose

roslib.load_manifest('indy7_robotiq_moveit_config')

def new_world_to_tf(new_world):
	"""
	calculate from frame data(translation and rotation) of 'new world' frame to tf data
	and publish as topic 'frame/new_world'. 
	"""
	pose_new_world = new_world
	br = tf.TransformBroadcaster()
	br.sendTransform(
		(pose_new_world.position.x, pose_new_world.position.y, 
		pose_new_world.position.z), 
		(pose_new_world.orientation.x, pose_new_world.orientation.y, 
		pose_new_world.orientation.z, pose_new_world.orientation.w), 
		rospy.Time.now(), 'new_world', 'world')

def object_to_tf(object):
	"""
	calculate from frame data(translation and rotation) of 'object' frame to tf data
	and publish as topic 'frame/object'.
	"""
	pose_object = object
	br = tf.TransformBroadcaster()
	br.sendTransform((pose_object.position.x, pose_object.position.y, 
					pose_object.position.z), 
					(pose_object.orientation.x, pose_object.orientation.y, 
					pose_object.orientation.z, pose_object.orientation.w), 
					rospy.Time.now(), 'object', 'new_world')

if __name__ == '__main__':
	rospy.init_node('tf_manager')
	rospy.Subscriber('frame/new_world', Pose, new_world_to_tf)
	rospy.Subscriber('frame/object', Pose, object_to_tf)
	rospy.spin()