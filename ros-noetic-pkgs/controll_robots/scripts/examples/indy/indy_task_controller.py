#!/usr/bin/python3

import sys
import os
import math
import rospy
import tf

from PyKDL import (  # pylint: disable=no-name-in-module
    Chain,
    Frame,
    Vector,
    Rotation,
    Segment,
    JntArray,
    Joint,
    ChainFkSolverPos_recursive,
    ChainIkSolverVel_pinv,
    ChainIkSolverPos_NR,
)

# from time import sleep
from std_msgs.msg import Bool, String, Float32MultiArray
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage


################ joint test list ####################

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
# chain_test = [0, 90, 0, 0, -90, 0]

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

################ joint test list end ################


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs


def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads


class TaskControllerApp:
    """Joint Controller

    Args:
      self.isstatus : check robot status
      self.listener : get tf(robot) info /link0 to /link6
      self.isshutdown : program termination param

      self.chain : define robot link(chain) from tf listener -> to calc kinematics(fk/ik)
      self.jointAngles : get current joint value of robot

      self.trans : vector value to build chain
      self.rot : ratation value to build chain

    """

    def __init__(self):
        rospy.init_node("task_controller_indy")

        self.ready = 0
        self.READY_STATE = 101
        self.BUSY_STATE = 202
        self.MOVING = 303
        self.ERROR = 404

        # publisher
        self.stop_pub = rospy.Publisher("/stop_motion", Bool, queue_size=1)
        self.jnt_pub = rospy.Publisher("joint_val", Float32MultiArray, queue_size=1)
        self.node_shutdown_pub = rospy.Publisher("/shutdown_msg", Bool, queue_size=1)

        # subscriber
        self.indy_status_check_sub = rospy.Subscriber(
            "/indy/status", GoalStatusArray, self.status_callback
        )
        self.indy_joint_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_callback
        )
        self.cmd_msg_sub = rospy.Subscriber("/cmd_msg", String, self.msg_callback)
        # self.frame_sub = rospy.Subscriber("/way_point_msg", Frame, self.get_point_cb)

        self.isstatus = GoalStatusArray()
        self.robot_tf = TFMessage()
        self.listener = tf.TransformListener()
        self.indy_jnt = JointState()
        self.isshutdown = False

        self.chain = Chain()
        self.jointAngles = JntArray(6)
        self.q_list = Float32MultiArray()

        self.q_list.data = [0 for l in range(6)]
        self.trans = [0 for i in range(6)]
        self.rot = [0 for j in range(6)]

        self.go_sign = "1"

        while True:
            try:
                (self.trans[0], self.rot[0]) = self.listener.lookupTransform(
                    "/link1", "/link2", rospy.Time(0)
                )
                (self.trans[1], self.rot[1]) = self.listener.lookupTransform(
                    "/link2", "/link3", rospy.Time(0)
                )
                (self.trans[2], self.rot[2]) = self.listener.lookupTransform(
                    "/link3", "/link4", rospy.Time(0)
                )
                (self.trans[3], self.rot[3]) = self.listener.lookupTransform(
                    "/link4", "/link5", rospy.Time(0)
                )
                (self.trans[4], self.rot[4]) = self.listener.lookupTransform(
                    "/link5", "/link6", rospy.Time(0)
                )
                (self.trans[5], self.rot[5]) = self.listener.lookupTransform(
                    "/link6", "/tcp", rospy.Time(0)
                )
                break

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

        # Build chain link1 to tcp
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
                    Vector(self.trans[0][0], self.trans[0][1], self.trans[0][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[1][0], self.trans[1][1], self.trans[1][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
                    Vector(self.trans[2][0], self.trans[2][1], self.trans[2][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
                    Vector(self.trans[3][0], self.trans[3][1], self.trans[3][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
                    Vector(self.trans[4][0], self.trans[4][1], self.trans[4][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[5][0], self.trans[5][1], self.trans[5][2]),
                ),
            )
        )

        ######## solver to calc kinematics(fk/ik) ##################
        # self.fksolverpos = ChainFkSolverPos_recursive(self.chain)
        # self.iksolver1v = ChainIkSolverVel_pinv(self.chain)
        # self.iksolverpos = ChainIkSolverPos_NR(self.chain, self.fksolverpos, self.iksolver1v)

        ############## Solver ref###################################
        # self.jacsolver = ChainJntToJacSolver(self.chain)
        # self.jacdotsolver = ChainJntToJacDotSolver(self.chain)
        # self.fksolverpos = ChainFkSolverPos_recursive(self.chain)
        # self.fksolvervel = ChainFkSolverVel_recursive(self.chain)
        # self.iksolvervel = ChainIkSolverVel_pinv(self.chain)
        # self.iksolvervel_givens = ChainIkSolverVel_pinv_givens(self.chain)
        # self.iksolverpos = ChainIkSolverPos_NR(self.chain, self.fksolverpos, self.iksolvervel)
        # self.iksolverpos_givens = ChainIkSolverPos_NR(self.chain, self.fksolverpos, self.iksolvervel_givens)

    def move_seq(self):

        ROBOT = self.READY_STATE
        ROBOT = self.isReadyState(ROBOT)
        q_jnt = JntArray(self.chain.getNrOfJoints())

        job_check = True
        job_count = 0

        while job_check:

            if ROBOT == self.READY_STATE:
                q = way_point[job_count]
                q = degs2rads(q)

                for l, k in enumerate(q):
                    q_jnt[l] = k

                for j in range(6):
                    self.q_list.data[j] = q_jnt[j]

                print(self.q_list)
                self.jnt_pub.publish(self.q_list)

                ROBOT = self.MOVING
                job_count += 1

            ROBOT = self.isReadyState(ROBOT)

            if ROBOT == self.ERROR:
                print("Error!")
                break

            if job_count > 15:
                # input_sig = True
                job_count = 0
                job_check = False

            if self.isshutdown:
                job_check = False

    def move_sig(self, sig):
        """
        keyboard input guide

            w         r
        a       d
            x         f


            Args :
                jointAngles : current joint values
                current_frame : current Frame value calculated fk using current joint angle value
                goal_frame : goal Frame
                q_jnt : goal joint angle value calculated ik using goal frame
        """

        ROBOT = self.READY_STATE
        ROBOT = self.isReadyState(ROBOT)
        goal_frame = Frame()
        current_frame = Frame()
        q_jnt = JntArray(self.chain.getNrOfJoints())

        self.fksolverpos = ChainFkSolverPos_recursive(self.chain)
        self.iksolver1v = ChainIkSolverVel_pinv(self.chain)
        self.iksolverpos = ChainIkSolverPos_NR(
            self.chain, self.fksolverpos, self.iksolver1v
        )

        self.fksolverpos.JntToCart(self.jointAngles, current_frame)

        tmp_ROT = Rotation.GetRPY(current_frame.M)

        ######## print for check #########

        print(" ")
        print("current joint angle")
        print(self.jointAngles)
        print("current frame")
        print(current_frame)
        print("#####################")

        ######## print for check end #####

        if sig == "0":
            q = [0, 0, 0, 0, 0, 0]
            q = degs2rads(q)

            for l, k in enumerate(q):
                q_jnt[l] = k

        ######################## VECTOR #########################
        # 0.1 => 0.1m
        # vector x + 0.1
        elif sig == "w":
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0] + 0.1, current_frame.p[1], current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector x - 0.1
        elif sig == "x":
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0] - 0.1, current_frame.p[1], current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector y + 0.1
        elif sig == "a":
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1] + 0.1, current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector y - 0.1
        elif sig == "d":
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1] - 0.1, current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector z + 0.1
        elif sig == "r":
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1], current_frame.p[2] + 0.1
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector z - 0.1
        elif sig == "f":
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1], current_frame.p[2] - 0.1
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
            tmp_ROT = Rotation.GetRPY(current_frame.M)
        ######################## VECTOR end #####################

        ######################## ROTATION #######################
        # 0.1 => 17 deg
        elif sig == "y":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0] + 0.1, tmp_ROT[1], tmp_ROT[2]), current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "u":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0] - 0.1, tmp_ROT[1], tmp_ROT[2]), current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "g":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1] + 0.1, tmp_ROT[2]), current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "h":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1] - 0.1, tmp_ROT[2]), current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "b":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1], tmp_ROT[2] + 0.1), current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "n":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1], tmp_ROT[2] - 0.1), current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        ######################## ROTATION end ###################

        ########## test #########################################

        elif sig == "j":
            goal_frame = Frame(current_frame.M, Vector(0.3, 0.3, 0.5))
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "k":
            goal_frame = Frame(current_frame.M, Vector(0.3, -0.3, 0.5))
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "l":
            goal_frame = Frame(current_frame.M, Vector(0.13, 0.756, 0.568))
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)

        ########## test end #####################################

        else:
            _input = int(self.go_sign) - 1

            if _input > 15:
                print("input ERROR! please enter num again(1~16)")
            else:
                q = way_point[_input]
                q = degs2rads(q)

                for l, k in enumerate(q):
                    q_jnt[l] = k

        for j in range(6):
            tmp_q = q_jnt[j] // math.pi
            tmp_r = q_jnt[j] % math.pi

            if tmp_q % 2:
                self.q_list.data[j] = tmp_r - math.pi

            else:
                self.q_list.data[j] = tmp_r

        self.jnt_pub.publish(self.q_list)

        # if ROBOT == self.READY_STATE:
        #     print(self.q_list)
        #     self.jnt_pub.publish(self.q_list)

        ######## print for check #########

        print("goal joint angle")
        print(self.q_list.data)
        print("goal frame")
        print(goal_frame)
        print("#####################")

        # print("ROT")
        # # print(current_frame.M.GetQuaternion())
        # print(current_frame.M.GetRPY())
        # print("Vector")
        # print(current_frame.p)

        ######## print for check end ######

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

    # get user command
    def msg_callback(self, cmd_msg):
        self.go_sign = cmd_msg.data

        if self.go_sign == "p":
            self.move_seq()
        elif self.go_sign == "s":
            self.stop_pub.publish(True)
        elif self.go_sign == "q":
            self.isshutdown = True
            self.node_shutdown_pub.publish(True)
        else:
            self.move_sig(self.go_sign)

    # get goal point frame and pub goal joint values
    def get_point_cb(self, fr_data):
        q_jnt = JntArray(self.chain.getNrOfJoints())
        self.iksolverpos.CartToJnt(self.jointAngles, fr_data, q_jnt)
        self.jnt_pub.publish(q_jnt)

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

        else:
            ROBOT = self.READY_STATE
            return ROBOT


def main():

    app = TaskControllerApp()

    while not rospy.is_shutdown():
        if app.isshutdown:
            break

    # rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
