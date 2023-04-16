#!/usr/bin/env python3
import rospy, roslib, tf
from geometry_msgs.msg import Pose, PoseArray

roslib.load_manifest('combined_tf')

def posearray_rb(data):
	print(data.poses)

def posearray_indy(data):
	print(data.poses)

def listener():
	rospy.init_node('object_listener')
	rospy.Subscriber('frame/objects_rb', PoseArray, posearray_rb)
	rospy.Subscriber('frame/objects_indy', PoseArray, posearray_indy)
	rospy.spin()

if __name__ == '__main__':
	listener()