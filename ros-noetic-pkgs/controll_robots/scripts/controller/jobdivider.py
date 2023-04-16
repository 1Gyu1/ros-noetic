#!/usr/bin/python3
"""
ROBOT job divider
"""

import sys
import os
import math
import rospy
import tf

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
from robot import Robot
from controll_robots.msg import robots_info

STANDBY = 'standby'
GET_OBJ = 'get_object'
WARN = 'warn_area'
SAFE = 'safe_area'


class Job(Robot):
    def __init__(self, robot):
        # is ok robot playing next job
        self.work_flag = False
        self.object_flag  = False
        self.my_status = 'standby'
        self.error = False
        self.r_info = robots_info()
        self.robot = robot

        # Publisher
        self.send_obj = rospy.Publisher("object", Pose, queue_size=1)
        self.send_status_rb = rospy.Publisher("/rb10/control_flag", String, queue_size=1)
        self.send_status_indy = rospy.Publisher("/indy7/control_flag", String, queue_size=1)

        if self.robot == "indy7":
            self.send_status = self.send_status_rb
            rospy.Subscriber("/rb10/control_flag", String, self.get_flag_cb)
        elif self.robot == "rb10":
            self.send_status = self.send_status_indy
            rospy.Subscriber("/indy7/control_flag", String, self.get_flag_cb)

        # Subscriber
        rospy.Subscriber("/r_info", robots_info, self.robot_data_cb)

    def SET_STATUS(self):
        # set my_status state
        # self.my_status -> STANBY, GET_OBJ, WARN, SAFE
        # standby -> stanby position
        # get_obj -> standby position and get object data
        # warn -> robot start moving
        # safe -> next post pick position
        if self.robot == "indy7":
            status = self.data.robot_status_indy7
        elif self.robot == "rb10":
            status = self.data.robot_status_rb10




    def COWORK_STATUS(self, cowork_status):
        """Check robot status to co-work

        Returns:
            robot status(FSM)
        """
        if self.my_status == STANDBY:
            if cowork_status != GET_OBJ:
                self.object_flag = True
        elif self.my_status == GET_OBJ:
            if cowork_status == WARN:
                self.work_flag = False
            elif cowork_status == GET_OBJ:
                print("Status Error!")
            else:
                self.work_flag = True

        elif (self.my_status == WARN) and (cowork_status == WARN):
            # MAKE Robots STOP 
            self.error = True
            print("Status Error! -Warn-")

        else:
            self.object_flag = False
            self.work_flag = False


    def get_flag_cb(self, flag):
        self.COWORK_STATUS(flag.data)
        
    def robot_data_cb(self, data):
        self.r_info = data
        self.SET_STATUS()