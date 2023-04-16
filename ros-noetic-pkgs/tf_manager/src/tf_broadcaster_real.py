#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import PoseArray

roslib.load_manifest('indy7_robotiq_moveit_config')

link_name = ['new0', 'new1', 'new2', 'new3', 'new4', 'new5', 'new6', 'newTCP']

def link_to_tf(link):
	"""
	calculate from frame data(translation and rotation) of each link to tf data
	and publish as topic 'link'.
	"""
	for i in range(8):
		if i == 0:
			pose_linklist = link.poses[i]
			br = tf.TransformBroadcaster()
			br.sendTransform(
				(pose_linklist.position.x, pose_linklist.position.y, 
				pose_linklist.position.z), 
				(pose_linklist.orientation.x, pose_linklist.orientation.y,
				pose_linklist.orientation.z, pose_linklist.orientation.w), 
				rospy.Time.now(), link_name[i], 'world')
		else:
			pose_linklist = link.poses[i]
			br = tf.TransformBroadcaster()
			br.sendTransform(
				(pose_linklist.position.x, pose_linklist.position.y, 
				pose_linklist.position.z), 
				(pose_linklist.orientation.x, pose_linklist.orientation.y, 
				pose_linklist.orientation.z, pose_linklist.orientation.w), 
				rospy.Time.now(), link_name[i], link_name[i - 1])

if __name__ == "__main__":
    rospy.init_node("tf_broadcaster_real")
    rospy.Subscriber("link", PoseArray, link_to_tf)
    rospy.spin()
