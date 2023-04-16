#!/usr/bin/env python
"""
Listening 6 poses and publish TF tree for each link
"""

import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray
# from kdl_chain.msg import PoseArray


link_name = ['newlink1', 'newlink2', 'newlink3', 'newlink4', 'newlink5', 'newlink6']


def link_to_tf(link) :

  # build kdtree and tftree
	br = tf.TransformBroadcaster() 

	for i in range(6):

		pose_linklist = link.poses[i]
		br.sendTransform((0.0, 0.0, 0.0775), 
				         (0.0, 0.0, 0.0, 1.0), 
			             rospy.Time.now(), '/link0', '/world')
		if i == 0 :
			pose_linklist = link.poses[i]
			br.sendTransform((pose_linklist.position.x, 
							  pose_linklist.position.y, 
							  pose_linklist.position.z), 
				             (pose_linklist.orientation.x, 
							  pose_linklist.orientation.y, 
							  pose_linklist.orientation.z, 
							  pose_linklist.orientation.w), 
			                  rospy.Time.now(), link_name[i], '/link0')
		else :
			pose_linklist = link.poses[i]
			br.sendTransform((pose_linklist.position.x, 
							  pose_linklist.position.y, 
							  pose_linklist.position.z), 
				 			 (pose_linklist.orientation.x, 
							  pose_linklist.orientation.y, 
							  pose_linklist.orientation.z, 
							  pose_linklist.orientation.w), 
							  rospy.Time.now(), link_name[i], link_name[i-1])
	

if __name__ == '__main__':
	rospy.init_node('tf_publisher')
	rospy.Subscriber('/robot_link', PoseArray, link_to_tf)
	rospy.spin()


# if __name__ == "__main__":
#   rospy.init_node('tf_publisher')

#   # build kdtree and tftree
#   br = tf.TransformBroadcaster()
#   rate = rospy.Rate(10.0)

#   while not rospy.is_shutdown():    
#     br.sendTransform((0,0,0), 
#                      tf.transformations.quaternion_from_euler(0,0,30), 
#                      rospy.Time.now(), 
#                      "line1", 
#                      "world")