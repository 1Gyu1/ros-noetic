#!/usr/bin/python3

import sys
import os
import math
import rospy

from time import sleep
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs


def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class JointControllerApp:
    """Indy Joint Controller"""

    def __init__(self):
        rospy.init_node("joint_controller_indy7")

        self.ready = 0

        # Publishers
        self.stop_pub = rospy.Publisher("/stop_motion", Bool, queue_size=1)

        ##########  namespace -> prefix #####################
        self.execute_joint_state_pub = rospy.Publisher(
            "execute_joint_state", JointState, queue_size=1
        )

        # Subscribers
        self.indy_status_check_sub = rospy.Subscriber(
            "status", GoalStatusArray, self.status_cb
        )
        self.task_joint_sub = rospy.Subscriber(
            "joint_val", Float32MultiArray, self.joint_cb
        )
        self.node_shutdown_sub = rospy.Subscriber(
            "shutdown_msg", Bool, self.shutdown_cb
        )


        # self.execute_joint_state_pub = rospy.Publisher(
        #     "/indy7/execute_joint_state", JointState, queue_size=1
        # )

        # # Subscribers
        # self.indy_status_check_sub = rospy.Subscriber(
        #     "/indy7/status", GoalStatusArray, self.status_cb
        # )
        # self.task_joint_sub = rospy.Subscriber(
        #     "/indy7/joint_val", Float32MultiArray, self.joint_cb
        # )
        # self.node_shutdown_sub = rospy.Subscriber(
        #     "/indy7/shutdown_msg", Bool, self.shutdown_cb
        # )

        self.isstatus = GoalStatusArray()
        self.isshutdown = False

    def status_cb(self, status):
        """robot status update

        Args:
            status: current indy7 status
                idle: 0
                moving: 1
        """
        self.isstatus = status
        # print(self.isstatus.status_list[0].status, end=' : ')
        # print(self.isstatus.status_list[0].text)
        self.ready = self.isstatus.status_list[0].status

    def go(self, q_jnt):
        """publish joint angles to indy7

        Args:
            q_jnt: joint angles
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [
            "joint0",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
        ]
        # joint_state_msg.position = degs2rads(q)
        # print("joint controller data")
        # print(q_jnt.data)
        joint_state_msg.position = q_jnt.data
        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        self.execute_joint_state_pub.publish(joint_state_msg)

    def stop(self):
        """stop indy7"""
        self.stop_pub.publish(True)

    def joint_cb(self, jnt_val):
        """get joint angles from TaskController and pub to robot connector
        """
        # if not self.ready:
        #     self.go(jnt_val)
        #     print(jnt_val)
        # else:
        #     self.stop()
        # print(jnt_val.data)
        self.go(jnt_val)

    def shutdown_cb(self, quit):
        """shutdown JointController node

        Args:
            quit: termination signal from TaskController
        """
        self.isshutdown = quit.data


def main():
    app = JointControllerApp()

    while not rospy.is_shutdown():

        if app.isshutdown:
            # sys.exit()
            break
        # rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
