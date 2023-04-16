#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import PoseArray

roslib.load_manifest('rb_tf_manager')


link_name = [
	'newlink1', 'newlink2', 'newlink3', 'newlink4', 'newlink5', 'newlink6'
]

def link_to_tf(link):
	for i in range(6):
		if i == 0:
			pose_linklist = link.poses[i]
			br = tf.TransformBroadcaster()
			br.sendTransform(
				(pose_linklist.position.x, pose_linklist.position.y, 
					pose_linklist.position.z), 
				(pose_linklist.orientation.x, pose_linklist.orientation.y, 
					pose_linklist.orientation.z, pose_linklist.orientation.w), 
				rospy.Time.now(), link_name[i], '/Axis_0')
		else:
			pose_linklist = link.poses[i]
			br = tf.TransformBroadcaster()
			br.sendTransform(
				(pose_linklist.position.x, pose_linklist.position.y, 
					pose_linklist.position.z), 
				(pose_linklist.orientation.x, pose_linklist.orientation.y, 
					pose_linklist.orientation.z, pose_linklist.orientation.w), 
				rospy.Time.now(), link_name[i], link_name[i - 1])
	

if __name__ == '__main__':
	rospy.init_node('tf_broadcaster_real')
	rospy.Subscriber('link', PoseArray, link_to_tf)
	rospy.spin()