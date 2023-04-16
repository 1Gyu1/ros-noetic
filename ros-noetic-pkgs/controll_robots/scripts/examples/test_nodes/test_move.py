#!/usr/bin/python3

import sys
import os
import math
import rospy

# from threading import Thread

from time import sleep
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray

global ready
global READY_STATE, BUSY_STATE, MOVING, ERROR  # , ROBOT
READY_STATE = 101
BUSY_STATE = 202
MOVING = 303
ERROR = 404


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs


def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads


class JointControllerApp:
    def __init__(self):
        rospy.init_node("joint_controller")
        global ready

        self.execute_joint_state_pub = rospy.Publisher(
            "/indy/execute_joint_state", JointState, queue_size=1
        )
        self.stop_pub = rospy.Publisher("/stop_motion", Bool, queue_size=1)
        self.indy_status_check_sub = rospy.Subscriber(
            "/indy/status", GoalStatusArray, self.callback
        )
        self.isstatus = GoalStatusArray()

    def callback(self, data):
        global ready
        self.isstatus = data
        # print(self.isstatus.status_list[0].status, end=' : ')
        # print(self.isstatus.status_list[0].text)
        ready = self.isstatus.status_list[0].status

    def go(self, q):
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
        joint_state_msg.position = degs2rads(q)
        joint_state_msg.velocity = []
        joint_state_msg.effort = []
        self.execute_joint_state_pub.publish(joint_state_msg)

    def stop(self):
        self.stop_pub.publish(True)


q = [0, 0, 0, 0, 0, 0]
home = [0, 0, -90, 0, -90, 0]
way_home = [8, -17, -86, 0, -77, 7]
p1_up = [10, -32, -88, 0, -60, 10]
p1_down = [10, -36, -92, 0, -52, 10]
p2_up = [0, -37, -76, 0, -67, 0]
p2_down = [0, -42, -82, 0, -57, 0]
p3_up = [2, -50, -53, 0, -78, 2]
p3_down = [2, -53, -58, 0, -70, 2]
p4_up = [11, -45, -63, 0, -73, 11]
p4_down = [11, -48, -68, 0, -65, 11]
way_home2 = [11, -24, -82, -1, -74, 11]

way_point = [
    home,
    way_home,
    p1_up,
    p1_down,
    p1_up,
    p2_up,
    p2_down,
    p2_up,
    p3_up,
    p3_down,
    p3_up,
    p4_up,
    p4_down,
    p4_up,
    way_home,
    home,
]


# ROBOT status check and turn
def isReadyState(ROBOT):
    global ready
    global READY_STATE, BUSY_STATE, MOVING, ERROR

    # ROBOT start moving
    if ROBOT == MOVING:

        # wait ROBOT status changing
        if ready == 0:
            return ROBOT
        # ROBOT moving (ROBOT status : busy)
        elif ready == 1:
            ROBOT = BUSY_STATE
            return ROBOT
        # Direct teaching(ready=2)/ Collision Error(ready=3)
        else:
            print("CAN NOT MOVING!")
            return ERROR

    # ROBOT moving...
    elif ROBOT == BUSY_STATE:
        # still moving
        if ready == 1:
            return ROBOT  # BUSY_STATE
        # ROBOT stops(turn to READY_STATE)
        elif ready == 0:
            ROBOT = READY_STATE
            return ROBOT
        # Direct teaching(ready=2)/ Collision Error(ready=3)
        else:
            print("CAN NOT MOVING!")
            return ERROR


def main():
    global READY_STATE, BUSY_STATE, MOVING

    ROBOT = READY_STATE
    app = JointControllerApp()

    q = home
    input_sig = True
    stop_sign = 0
    i = 0

    while not stop_sign:

        try:
            if input_sig:
                go_sign = input("enter point(1~16). robot stop(s). : ")

            if go_sign == "0":
                q = [0, 0, 0, 0, 0, 0]
                app.go(q)

            elif go_sign == "s":
                app.stop()
            elif go_sign == "q":
                stop_sign = 1

            elif go_sign == "r":
                input_sig = False

                # READY_STATE
                if ROBOT == READY_STATE:
                    app.go(way_point[i])
                    ROBOT = MOVING
                    i = i + 1

                # ROBOT status renew
                ROBOT = isReadyState(ROBOT)

                # ERROR occurs
                if ROBOT == ERROR:
                    break

                if i > 15:
                    input_sig = True
                    i = 0

            else:
                _input = int(go_sign) - 1

                if _input > 15:
                    print("input ERROR! please enter num again(1~16)")
                else:
                    q = way_point[_input]
                    app.go(q)

        except KeyboardInterrupt:
            # go_sign = 's'
            sys.exit()


if __name__ == "__main__":
    main()
