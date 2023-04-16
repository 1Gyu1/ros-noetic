#!/usr/bin/python3

import sys
import os
import math
import rospy
import tf
import rospy

from PyKDL import *
from time import sleep
from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage

from controll_robots.msg import rb_command

init_str = "jointall "
rb_val = [0.300, 0.100]
jnt_val = [0, 0, 0, 0, 0, 0]
cmd_val = rb_val + jnt_val
cmd = rb_command()

jnt_exec_pub = rospy.Publisher("/rb_command", rb_command, queue_size=1)


def INPUT_cmd():
    """
    rb10 robot input test node


    """

    rospy.init_node("input_test")

    stop_sign = False

    while not stop_sign:
        if not stop_sign:
            for i in range(6):
                cmd_val[i + 2] = input("input joint deg : ")

                if cmd_val[i + 2] == "q":
                    stop_sign = True

                cmd_val[i + 2] = float(cmd_val[i + 2])

        for i in range(len(cmd_val)):
            cmd_val[i] = str(cmd_val[i])

        rb_str = ",".join(cmd_val)
        cmd.cmd = init_str + rb_str

        print(cmd.cmd)

        jnt_exec_pub.publish(cmd)


if __name__ == "__main__":
    INPUT_cmd()
    rospy.spin()
