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


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs


def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads


class TestControllerApp:
    """Joint Controller

    Args:
      self.isstatus : check robot status
      self.listener : get tf(robot) info /link0 to /link6

      self.chain : define robot link(chain) from tf listener -> to calc kinematics(fk/ik)
      self.eeFrame : save current frame(rotation, vector)
      self.jointAngles : get current joint value of robot

      self.trans : vector value to build chain
      self.rot : ratation value to build chain

      self.q_jnt : save joint value after inverse kinematics

    """

    def __init__(self):
        rospy.init_node("controller_test_indy")

        self.ready = 0
        self.READY_STATE = 101
        self.BUSY_STATE = 202
        self.MOVING = 303
        self.ERROR = 404

        # publisher
        self.execute_joint_state_pub = rospy.Publisher(
            "/indy/execute_joint_state", JointState, queue_size=1
        )
        self.stop_pub = rospy.Publisher("/stop_motion", Bool, queue_size=1)
        self.jnt_pub = rospy.Publisher("/joint_val", Float32MultiArray, queue_size=1)

        # listener
        self.indy_status_check_sub = rospy.Subscriber(
            "/indy/status", GoalStatusArray, self.status_callback
        )
        self.indy_joint_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_callback
        )
        self.cmd_msg_sub = rospy.Subscriber("/cmd_msg", String, self.msg_callback)

        self.isstatus = GoalStatusArray()
        self.robot_tf = TFMessage()
        self.listener = tf.TransformListener()
        self.indy_jnt = JointState()

        self.chain = Chain()
        self.eeFrame = Frame()
        # self.jointAngles = JntArray(chain.getNrOfJoints(chain))
        self.jointAngles = JntArray(6)
        self.q_jnt = JntArray(6)

        self.trans = [0 for i in range(7)]
        self.rot = [0 for j in range(7)]

        self.go_sign = 888

        while True:
            try:
                (self.trans[0], self.rot[0]) = self.listener.lookupTransform(
                    "/link0", "/link1", rospy.Time(0)
                )
                (self.trans[1], self.rot[1]) = self.listener.lookupTransform(
                    "/link1", "/link2", rospy.Time(0)
                )
                (self.trans[2], self.rot[2]) = self.listener.lookupTransform(
                    "/link2", "/link3", rospy.Time(0)
                )
                (self.trans[3], self.rot[3]) = self.listener.lookupTransform(
                    "/link3", "/link4", rospy.Time(0)
                )
                (self.trans[4], self.rot[4]) = self.listener.lookupTransform(
                    "/link4", "/link5", rospy.Time(0)
                )
                (self.trans[5], self.rot[5]) = self.listener.lookupTransform(
                    "/link5", "/link6", rospy.Time(0)
                )
                # test
                self.ttkk = self.listener.lookupTransform(
                    "/world", "/link6", rospy.Time(0)
                )
                # for i in range(6):
                #     print(self.trans[i])
                #     print(self.rot[i])
                break

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                # print("except error")
                continue

        # Build chain link0 to link6
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(
                        self.rot[0][0], self.rot[0][1], self.rot[0][2], self.rot[0][3]
                    )
                )
                * Frame(Vector(self.trans[0][0], self.trans[0][1], self.trans[0][2])),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(
                        self.rot[1][0], self.rot[1][1], self.rot[1][2], self.rot[1][3]
                    )
                )
                * Frame(Vector(self.trans[1][0], self.trans[1][1], self.trans[1][2])),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.TransZ),
                Frame(
                    Rotation.Quaternion(
                        self.rot[2][0], self.rot[2][1], self.rot[2][2], self.rot[2][3]
                    )
                )
                * Frame(Vector(self.trans[2][0], self.trans[2][1], self.trans[2][2])),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(
                        self.rot[3][0], self.rot[3][1], self.rot[3][2], self.rot[3][3]
                    )
                )
                * Frame(Vector(self.trans[3][0], self.trans[3][1], self.trans[3][2])),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(
                        self.rot[4][0], self.rot[4][1], self.rot[4][2], self.rot[4][3]
                    )
                )
                * Frame(Vector(self.trans[4][0], self.trans[4][1], self.trans[4][2])),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(
                        self.rot[5][0], self.rot[5][1], self.rot[5][2], self.rot[5][3]
                    )
                )
                * Frame(Vector(self.trans[5][0], self.trans[5][1], self.trans[5][2])),
            )
        )

        # solver to calc kinematics(fk/ik)
        self.fksolverpos = ChainFkSolverPos_recursive(self.chain)
        self.iksolver1v = ChainIkSolverVel_pinv(self.chain)
        self.iksolverpos = ChainIkSolverPos_NR(
            self.chain, self.fksolverpos, self.iksolver1v
        )

        ############## Solver ref###################################
        # self.jacsolver = ChainJntToJacSolver(self.chain)
        # self.jacdotsolver = ChainJntToJacDotSolver(self.chain)
        # self.fksolverpos = ChainFkSolverPos_recursive(self.chain)
        # self.fksolvervel = ChainFkSolverVel_recursive(self.chain)
        # self.iksolvervel = ChainIkSolverVel_pinv(self.chain)
        # self.iksolvervel_givens = ChainIkSolverVel_pinv_givens(self.chain)
        # self.iksolverpos = ChainIkSolverPos_NR(self.chain, self.fksolverpos, self.iksolvervel)
        # self.iksolverpos_givens = ChainIkSolverPos_NR(self.chain, self.fksolverpos, self.iksolvervel_givens)

    # Subscriber callback funcs
    def status_callback(self, data):
        self.isstatus = data
        # print(self.isstatus.status_list[0].status, end=' : ')
        # print(self.isstatus.status_list[0].text)
        self.ready = self.isstatus.status_list[0].status

    # current joint value(rad)
    def joint_callback(self, jnt):
        for i in range(6):
            self.jointAngles[i] = jnt.position[i]

    def msg_callback(self, cmd_msg):
        self.go_sign = cmd_msg.data

    # Publish funcs to Robot
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

    # Robot status func
    def isReadyState(self, ROBOT):
        """ROBOT status check and convert

        Args:
          self.ready : current robot status (1:ready, 2:busy, 3:direct teaching)
          self.READY_STATE : when robot can get joint value and move
          self.BUSY_STATE : when robot moving
          self.MOVING : when controller give joint value to robot
          self.ERROR : occurs error

        Returns:
          robot state (compared with previous state)

        Raises:
          RobotError: If robot status is error status.
        """

        # ROBOT start moving
        if ROBOT == self.MOVING:
            if self.ready == 0:
                return ROBOT
            elif self.ready == 1:
                ROBOT = self.BUSY_STATE
                return ROBOT
            else:
                print("CAN NOT MOVING!")
                return self.ERROR

        # ROBOT moving...
        elif ROBOT == self.BUSY_STATE:
            if self.ready == 1:
                return ROBOT
            elif self.ready == 0:
                ROBOT = self.READY_STATE
                return ROBOT
            else:
                print("CAN NOT MOVING!")
                return self.ERROR


# examples of teaching points
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


def main():

    app = TestControllerApp()
    ROBOT = app.READY_STATE

    test_frame = Frame()

    q = home
    input_sig = True
    stop_sign = 0
    i = 0

    while not stop_sign:

        app.fksolverpos.JntToCart(app.jointAngles, app.eeFrame)

        """
        keyboard input guide

            w         r
        a       d
            x         f

        """
        while app.go_sign == 888:
            continue

        try:
            # print(app.go_sign)
            # if input_sig:
            #     go_sign = input('enter point(1~16). robot stop(s). : ')

            if app.go_sign == "0":
                q = [0, 0, 0, 0, 0, 0]
                app.go(q)

            # vector x + 0.1
            elif app.go_sign == "w":
                test_frame = Frame(
                    app.eeFrame.M,
                    Vector(app.eeFrame.p[0] + 0.1, app.eeFrame.p[1], app.eeFrame.p[2]),
                )
                app.iksolverpos.CartToJnt(app.jointAngles, test_frame, app.q_jnt)
                app.go(rads2degs(app.q_jnt))

            # vector x - 0.1
            elif app.go_sign == "x":
                test_frame = Frame(
                    app.eeFrame.M,
                    Vector(app.eeFrame.p[0] - 0.1, app.eeFrame.p[1], app.eeFrame.p[2]),
                )
                app.iksolverpos.CartToJnt(app.jointAngles, test_frame, app.q_jnt)
                app.go(rads2degs(app.q_jnt))

            # vector y + 0.1
            elif app.go_sign == "a":
                test_frame = Frame(
                    app.eeFrame.M,
                    Vector(app.eeFrame.p[0], app.eeFrame.p[1] + 0.1, app.eeFrame.p[2]),
                )
                app.iksolverpos.CartToJnt(app.jointAngles, test_frame, app.q_jnt)
                app.go(rads2degs(app.q_jnt))

            # vector y - 0.1
            elif app.go_sign == "d":
                test_frame = Frame(
                    app.eeFrame.M,
                    Vector(app.eeFrame.p[0], app.eeFrame.p[1] - 0.1, app.eeFrame.p[2]),
                )
                app.iksolverpos.CartToJnt(app.jointAngles, test_frame, app.q_jnt)
                app.go(rads2degs(app.q_jnt))

            # vector z + 0.1
            elif app.go_sign == "r":
                test_frame = Frame(
                    app.eeFrame.M,
                    Vector(app.eeFrame.p[0], app.eeFrame.p[1], app.eeFrame.p[2] + 0.1),
                )
                app.iksolverpos.CartToJnt(app.jointAngles, test_frame, app.q_jnt)
                app.go(rads2degs(app.q_jnt))

            # vector z - 0.1
            elif app.go_sign == "f":
                test_frame = Frame(
                    app.eeFrame.M,
                    Vector(app.eeFrame.p[0], app.eeFrame.p[1], app.eeFrame.p[2] - 0.1),
                )
                app.iksolverpos.CartToJnt(app.jointAngles, test_frame, app.q_jnt)
                app.go(rads2degs(app.q_jnt))

            elif app.go_sign == "s":
                app.stop()

            elif app.go_sign == "q":
                stop_sign = 1

            elif app.go_sign == "p":
                # input_sig = False

                if ROBOT == app.READY_STATE:
                    app.go(way_point[i])
                    ROBOT = app.MOVING
                    i = i + 1

                ROBOT = app.isReadyState(ROBOT)

                if ROBOT == app.ERROR:
                    break

                # if i > 15:
                #     input_sig = True
                #     i = 0

            elif app.go_sign != 888:
                _input = int(app.go_sign) - 1

                if _input > 15:
                    print("input ERROR! please enter num again(1~16)")
                else:
                    q = way_point[_input]
                    q = degs2rads(q)

                    for i, k in enumerate(q):
                        app.q_jnt[i] = k

                    app.go(rads2degs(app.q_jnt))

            app.go_sign = 888

        except KeyboardInterrupt:
            sys.exit()

        print("frame")
        print(app.eeFrame)
        print("link0-link1")
        print(app.trans[0])
        print(app.rot[0])
        print("goal joint")
        print(app.q_jnt[0])
        print("world to link6")
        print(app.ttkk[0])
        print(app.ttkk[1])


if __name__ == "__main__":
    main()
