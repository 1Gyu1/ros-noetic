#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import PoseArray

roslib.load_manifest('rb_tf_manager')

def trajectory_to_tf(pointlist):
	"""
	calculate from frame data(translation and rotation) of each subpoint of trajectory to tf data
	"""

	for i in range(len(pointlist.poses)):
		pose_linklist = pointlist.poses[i]
		br = tf.TransformBroadcaster()
		br.sendTransform(
			(pose_linklist.position.x, pose_linklist.position.y, 
				pose_linklist.position.z), 
			(pose_linklist.orientation.x, pose_linklist.orientation.y, 
				pose_linklist.orientation.z, pose_linklist.orientation.w), 
			rospy.Time.now(), str(i), 'Axis_0')

if __name__ == '__main__':
	rospy.init_node('trajectory_broadcaster')
	rospy.Subscriber('traj_array', PoseArray, trajectory_to_tf)
	rospy.spin()
	
	
