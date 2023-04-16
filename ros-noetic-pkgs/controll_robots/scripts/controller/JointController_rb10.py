#!/usr/bin/python3

import sys
import os
import math
import rospy

from time import sleep
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray

# custom msg
from controll_robots.msg import rb_command, rb_data


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs


def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads
    
class JointControllerApp:
    """RB10 Joint Controller"""

    def __init__(self):
        rospy.init_node("joint_controller_rb10")

        self.ready = 0
        self.spd = "0.3"
        self.acc = "0.1"
        self.rb_init_val = [self.spd, self.acc]
        self.init_str_joint = "jointall "

        # Publisher
        self.rb_command_pub = rospy.Publisher("/rb_command", rb_command, queue_size=1)
        # self.stop_pub = rospy.Publisher("/stop_motion", Bool, queue_size=1)

        # Subscriber
        self.rb_status_check_sub = rospy.Subscriber("/rb_data", rb_data, self.status_cb)
        self.task_stop_sub = rospy.Subscriber("/stop_rb", Bool, self.stop_cb)

        ##########  namespace -> prefix #####################
        self.task_joint_sub = rospy.Subscriber(
            "joint_val", Float32MultiArray, self.joint_cb
        )
        self.node_shutdown_sub = rospy.Subscriber(
            "shutdown_msg", Bool, self.shutdown_cb
        )
        
        # self.task_joint_sub = rospy.Subscriber(
        #     "/rb10/joint_val", Float32MultiArray, self.joint_cb
        # )
        # self.node_shutdown_sub = rospy.Subscriber(
        #     "/rb10/shutdown_msg", Bool, self.shutdown_cb
        # )

        self.isstatus = GoalStatusArray()
        self.isshutdown = False

    def go(self, q_jnt):
        """publish joint angles to rb10

        Args:
            q_jnt: joint angles
        """
        cmd = rb_command()
        stop = rb_command()
        stop.cmd = "task pause"
        jnt_val = []
        jnt_val = jnt_val + self.rb_init_val

        for i in range(len(q_jnt)):
            jnt_val.append(str(q_jnt[i]))

        cmd_tmp = ",".join(jnt_val)
        cmd.cmd = self.init_str_joint + cmd_tmp

        self.rb_command_pub.publish(cmd)

        # IDLE state
        # if self.ready == 1:
        #     self.rb_command_pub.publish(cmd)
        # Moving
        # elif self.ready == 3:
        #     print("no stop")
        #     self.stop_cb(stop)
        #     rospy.sleep(2)
        #     self.rb_command_pub.publish(cmd)

        # else:
        #     self.stop_cb(stop)

    # Callback functions
    def joint_cb(self, jnt_val):
        """get joint angles from TaskController and pub to robot connector
        """
        self.go(rads2degs(jnt_val.data))
        # if self.ready == 1:
        #     self.go(rads2degs(jnt_val.data))

    def stop_cb(self, stop_cmd):
        """stop rb10

        Args:
            stop_cmd: stop sigal
        """
        stop = rb_command()
        if stop_cmd:
            stop.cmd = "task pause"
            self.rb_command_pub.publish(stop)
            rospy.sleep(2)
            stop.cmd = "task stop"
            self.rb_command_pub.publish(stop)

    def status_cb(self, status):
        """robot status update

        Args:
            status: current indy7 status
                idle: 1
                moving: 3
        """
        self.ready = status.robot_state
        # print(self.ready)

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
            break
    # rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
