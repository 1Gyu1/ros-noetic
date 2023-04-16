#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose

roslib.load_manifest('indy7_robotiq_moveit_config')

def point1_to_tf(point1):
	"""
	calculate from frame data(translation and rotation) of 'point1' frame to tf data,
	and publish as topic 'point/point1'. 
	"""
	pose_object1 = point1
	br = tf.TransformBroadcaster()
	br.sendTransform((pose_object1.position.x, pose_object1.position.y, 
					pose_object1.position.z), 
					(pose_object1.orientation.x, pose_object1.orientation.y, 
					pose_object1.orientation.z, pose_object1.orientation.w), 
					rospy.Time.now(), 'object1', 'link0')

def point2_to_tf(point2):
	"""
	calculate from frame data(translation and rotation) of 'point2' frame to tf data, 
	and publish as topic 'point/point2'. 
	"""
	pose_object2 = point2
	br = tf.TransformBroadcaster()
	br.sendTransform((pose_object2.position.x, pose_object2.position.y, 
					pose_object2.position.z), 
					(pose_object2.orientation.x, pose_object2.orientation.y, 
					pose_object2.orientation.z, pose_object2.orientation.w), 
					rospy.Time.now(), 'object2', 'link0')

if __name__ == '__main__':
	rospy.init_node('tf_broadcaster')
	rospy.Subscriber('point/point1', Pose, point1_to_tf)
	rospy.Subscriber('point/point2', Pose, point2_to_tf)
	rospy.spin()
	
	
