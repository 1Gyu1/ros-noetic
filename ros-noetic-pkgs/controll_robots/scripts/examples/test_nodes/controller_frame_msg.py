#!/usr/bin/python3

import sys
import os
import math
import rospy
import tf

from PyKDL import *
from time import sleep
from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage


def Frame_talker():
    """
    Publish topics that are teaching points (frame values)

    Args:
        frame_val : saved frame values



    """
    pub = rospy.Publisher("/frame_val", Frame, queue_size=1)
    rospy.init_node("frame_talker", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     cmd_msg = input('enter point(1~16). robot stop(s). : ')
    #     rospy.loginfo(cmd_msg)
    #     pub.publish(cmd_msg)
    #     rate.sleep()


if __name__ == "__main__":
    try:
        Frame_talker()
    except rospy.ROSInterruptException:
        pass
