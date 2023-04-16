#!/usr/bin/env python3

import rospy, tf
from math import pi
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool

flag = False

def flag_cb(traj_flag):
	global flag
	print("callback")
	print(traj_flag.data)
	flag = traj_flag.data

def publish_point():
	# global flag
    rospy.init_node('trajectory_publisher_onePoint')
    pub = rospy.Publisher('/traj_array', PoseArray, queue_size=1)
    rospy.Subscriber("/traj_flag", Bool, flag_cb)
    rate = rospy.Rate(1)

    # random point
    # p = [0.5, 0.5, 0.5, 1.57, 0.0, 1.57]
    p = [0.5, 0.5, 0.5, 1.57, 0.7, 1.57]

    pose_p = Pose()
    
    pose_p.position.x = p[0]
    pose_p.position.y = p[1]
    pose_p.position.z = p[2]
    tmp = tf.transformations.quaternion_from_euler(
			p[3],p[4], p[5])
    pose_p.orientation.x = tmp[0]
    pose_p.orientation.y = tmp[1]
    pose_p.orientation.z = tmp[2]
    pose_p.orientation.w = tmp[3]
    pose_pointlist = PoseArray()
    pose_pointlist.poses.append(pose_p)
	# print("publish part")
	# print(flag)
    pub.publish(pose_pointlist)
    rate.sleep()
	

if __name__ == '__main__':
	while not rospy.is_shutdown():
		publish_point()
		if flag:
			break

		
