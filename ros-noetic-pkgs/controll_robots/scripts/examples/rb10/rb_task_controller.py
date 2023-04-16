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
from time import sleep
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage
# from robot_model.RB10 import RB10

# custom msg
from controll_robots.msg import rb_command, rb_data


################ joint test list ####################

# examples of teaching points
q = [0, 0, 0, 0, 0, 0]
home = [0, 0, 90, 0, 90, 0]
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

################ joint test list end ################


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs


def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads


class TaskControllerApp:
    """
    Task Controller

    Args:
      self.isstatus : check robot status
      self.listener : get tf(robot) info /Axis_0 to /wrist3
      self.isshutdown : program termination param

      self.chain : define robot link(chain) from tf listener -> to calc kinematics(fk/ik)
      self.eeFrame : save current frame(rotation, vector)
      self.jointAngles : get current joint value of robot
      self.cmd_msg : rb_command msg

      self.trans : vector value to build chain
      self.rot : ratation value to build chain

      self.q_jnt : save joint value after inverse kinematics

    """

    def __init__(self):
        rospy.init_node("task_controller_rb")

        self.ready = 0
        self.READY_STATE = 101
        self.BUSY_STATE = 202
        self.MOVING = 303
        self.ERROR = 404

        # Publisher
        self.rb_command_pub = rospy.Publisher("/rb_command", rb_command, queue_size=1)
        self.stop_pub = rospy.Publisher("/stop_rb", Bool, queue_size=1)
        self.jnt_pub = rospy.Publisher("/joint_val", Float32MultiArray, queue_size=1)
        self.node_shutdown_pub = rospy.Publisher("/shutdown_msg", Bool, queue_size=1)
        # self.chain_data_node = rospy.Publisher("/get_data", Bool, queue_size=1)
        self.to_traj_pub = rospy.Publisher("/traj_flag", Bool, queue_size=1)

        # Subscriber
        self.rb_status_check_sub = rospy.Subscriber("/rb_data", rb_data, self.status_cb)
        self.rb_joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_cb)
        self.cmd_msg_sub = rospy.Subscriber("/cmd_msg", String, self.msg_cb)
        self.trajectory_sub = rospy.Subscriber(
            "/traj_array", PoseArray, self.get_trajectory_cb
        )

        self.cmd_msg = rb_command()
        self.isstatus = GoalStatusArray()
        self.isshutdown = False
        self.flag = False
        self.robot_tf = TFMessage()
        self.listener = tf.TransformListener()
        self.rb_jnt = JointState()
        self.chain = Chain()
        self.jointAngles = JntArray(6)
        self.q_list = Float32MultiArray()
        self.traj_list = PoseArray()

        # self.jointAngles[2] = self.jointAngles[4] = math.pi / 2
        self.count = 0

        self.q_list.data = [0 for l in range(6)]
        self.trans = [0 for i in range(6)]
        self.rot = [0 for j in range(6)]

        while True:
            try:
                (self.trans[0], self.rot[0]) = self.listener.lookupTransform(
                    "/base", "/shoulder", rospy.Time(0)
                )
                (self.trans[1], self.rot[1]) = self.listener.lookupTransform(
                    "/shoulder", "/elbow", rospy.Time(0)
                )
                (self.trans[2], self.rot[2]) = self.listener.lookupTransform(
                    "/elbow", "/wrist1", rospy.Time(0)
                )
                (self.trans[3], self.rot[3]) = self.listener.lookupTransform(
                    "/wrist1", "/wrist2", rospy.Time(0)
                )
                (self.trans[4], self.rot[4]) = self.listener.lookupTransform(
                    "/wrist2", "/wrist3", rospy.Time(0)
                )
                (self.trans[5], self.rot[5]) = self.listener.lookupTransform(
                    "/wrist3", "/Tool_Center_Point", rospy.Time(0)
                )
                break

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

        
        # Build chain Axis to wrist3
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[0][0], self.trans[0][1], self.trans[0][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotY),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[1][0], self.trans[1][1], self.trans[1][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotY),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[2][0], self.trans[2][1], self.trans[2][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotY),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[3][0], self.trans[3][1], self.trans[3][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotZ),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[4][0], self.trans[4][1], self.trans[4][2]),
                ),
            )
        )
        self.chain.addSegment(
            Segment(
                Joint(Joint.RotY),
                Frame(
                    Rotation.Quaternion(0, 0, 0, 1),
                    Vector(self.trans[5][0], self.trans[5][1], self.trans[5][2]),
                ),
            )
        )

        
        ############ solver to calc kinematics(fk/ik) ##############
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

    ########## TEST move ##############

    def move_test(self):

        ROBOT = self.READY_STATE
        ROBOT = self.isReadyState(ROBOT)
        q_jnt = JntArray(6)

        job_count = 0

        while not rospy.is_shutdown():

            if ROBOT == self.READY_STATE:

                if job_count % 2:
                    q = [70, -20, 75, 36, 90, 0]
                else:
                    q = [0, 0, 90, 0, 90, 0]

                q = degs2rads(q)

                for l, k in enumerate(q):
                    q_jnt[l] = k

                for j in range(6):
                    self.q_list.data[j] = q_jnt[j]

                # print(self.q_list)
                self.jnt_pub.publish(self.q_list)

                ROBOT = self.MOVING
                job_count += 1
                print("job count : ", job_count)

            rospy.sleep(1.5)
            ROBOT = self.isReadyState(ROBOT)
            if ROBOT == self.MOVING:
                ROBOT = self.READY_STATE

            elif ROBOT == self.ERROR:
                print("Error!!")
                break

    ########## TEST move end ###########

    def move_seq(self):

        ROBOT = self.READY_STATE
        ROBOT = self.isReadyState(ROBOT)
        q_jnt = JntArray(6)
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

            print(self.isshutdown)

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
        current_frame = Frame()
        goal_frame = Frame()
        q_jnt = JntArray(self.chain.getNrOfJoints())

        # Calc fk to get current frame
        self.fksolverpos.JntToCart(self.jointAngles, current_frame)

        tmp_ROT = Rotation.GetRPY(current_frame.M)

        ######## print for check #########

        print(" ")
        print("current joint angle")
        print(self.jointAngles)
        print("current_frame")
        print(current_frame)
        print("#####################")

        ######## print for check end #####

        if sig == "0":
            print("zero position")
            q = [0, 0, 0, 0, 0, 0]
            q = degs2rads(q)
            for l, k in enumerate(q):
                q_jnt[l] = k

        ######################## VECTOR #########################
        # 0.1 => 0.1m
        # vector x + 0.1
        elif sig == "w":
            print("sig : ", sig)
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0] + 0.1, current_frame.p[1], current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector x - 0.1
        elif sig == "x":
            print("sig : ", sig)
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0] - 0.1, current_frame.p[1], current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector y + 0.1
        elif sig == "a":
            print("sig : ", sig)
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1] + 0.1, current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector y - 0.1
        elif sig == "d":
            print("sig : ", sig)
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1] - 0.1, current_frame.p[2]
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector z + 0.1
        elif sig == "r":
            print("sig : ", sig)
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1], current_frame.p[2] + 0.1
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # vector z - 0.1
        elif sig == "f":
            print("sig : ", sig)
            goal_frame = Frame(
                current_frame.M,
                Vector(
                    current_frame.p[0], current_frame.p[1], current_frame.p[2] - 0.1
                ),
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)

        ######################## VECTOR end #####################

        ######################## ROTATION #######################
        # 0.1 => 17 deg
        # Rx + 0.1 rads
        elif sig == "y":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0] + 0.1, tmp_ROT[1], tmp_ROT[2]),
                current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # Rx - 0.1 rads
        elif sig == "u":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0] - 0.1, tmp_ROT[1], tmp_ROT[2]),
                current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
            # Ry + 0.1 rads
        elif sig == "g":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1] + 0.1, tmp_ROT[2]),
                current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # Ry - 0.1 rads
        elif sig == "h":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1] - 0.1, tmp_ROT[2]),
                current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # Rz + 0.1 rads
        elif sig == "b":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1], tmp_ROT[2] + 0.1),
                current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        # Rz - 0.1 rads
        elif sig == "n":
            goal_frame = Frame(
                Rotation.RPY(tmp_ROT[0], tmp_ROT[1], tmp_ROT[2] - 0.1),
                current_frame.p
            )
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)

        ######################## ROTATION end ###################

        ########## test #########################################
        elif sig == "j":
            print("sig : ", sig)
            goal_frame = Frame(current_frame.M, Vector(0.5, -0.5, 0.8))
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "k":
            goal_frame = Frame(current_frame.M, Vector(0.13, 0.756, 0.568))
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        elif sig == "l":
            goal_frame = Frame(current_frame.M, Vector(0.5, 0.5, 0.8))
            self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)
        ########## test end ######################################

        else:
            _input = int(self.go_sign) - 1

            if _input > 15:
                print("input ERROR! please enter num again(1~16)")
            else:
                q = way_point[_input]
                q = degs2rads(q)

                for l, k in enumerate(q):
                    q_jnt[l] = k

        # nomalize joint radian value
        for j in range(6):
            tmp_q = q_jnt[j] // math.pi
            tmp_r = q_jnt[j] % math.pi

            if tmp_q % 2:
                self.q_list.data[j] = tmp_r - math.pi

            else:
                self.q_list.data[j] = tmp_r

        self.jnt_pub.publish(self.q_list)

        ######## print for check #########

        print("goal joint angle")
        print(self.jointAngles)
        print("goal frame")
        print(goal_frame)
        print("#####################")
        # print(Rotation.GetRPY(goal_frame.M))
        # print(Rotation.GetRPY(current_frame.M))

        ######## print for check end #####

    def move_traj(self, traj, traj_len):
        ROBOT = self.READY_STATE
        ROBOT = self.isReadyState(ROBOT)
        current_frame = Frame()
        q_jnt = JntArray(6)
        idx = 0

        # Calc fk to get current frame
        self.fksolverpos.JntToCart(self.jointAngles, current_frame)
        # tmp_ROT = Rotation.GetRPY(current_frame.M)

        while True:
            if ROBOT == self.READY_STATE:

                print("idx :", idx)

                goal_frame = Frame()
                goal_frame = Frame(
                    current_frame.M,
                    Vector(
                        traj.poses[idx].position.x,
                        traj.poses[idx].position.y,
                        traj.poses[idx].position.z,
                    ),
                )
                # goal_frame.p[0] = traj[i].poses.position.x
                # goal_frame.p[1] = traj[i].poses.position.y
                # goal_frame.p[2] = traj[i].poses.position.z
                # goal_frame.M = current_frame.M
                # goal_frame.Rotation.Quaternion(traj[i].poses.orientation.x,
                #                                traj[i].poses.orientation.y,
                #                                traj[i].poses.orientation.z,
                #                                traj[i].poses.orientation.w)
                self.iksolverpos.CartToJnt(self.jointAngles, goal_frame, q_jnt)

                for j in range(6):
                    self.q_list.data[j] = q_jnt[j]

                print("test traj q_list")
                print(self.q_list)
                idx += 1

                self.jnt_pub.publish(self.q_list)

                ROBOT = self.MOVING
                # rospy.sleep(1.5)
            

            ROBOT = self.isReadyState(ROBOT)
            if self.jointAngles == q_jnt:
                ROBOT = self.READY_STATE
            # if ROBOT == self.MOVING:
            #     ROBOT = self.READY_STATE
            if idx > traj_len:
                break
            if ROBOT == self.ERROR:
                print("Error!")
                break
            if self.isshutdown:
                break

        self.flag = False
        self.to_traj_pub.publish(self.flag)
        print("flag :", self.flag)

    # append trajectory Posearray
    def append_traj(self, tmp_traj, traj):
        tmp_traj.poses.append(traj)

    # Subscriber cb funcs
    def get_trajectory_cb(self, traj):
        self.to_traj_pub.publish(self.flag)
        tmp_traj = PoseArray()
        traj_len = 0

        # while not traj[traj_len] == None :
        #     self.append_traj(tmp_traj, traj.poses[traj_len])
        #     traj_len += 1

        for idx, tmp in enumerate(traj.poses):
            tmp_traj.poses.append(tmp)
            # self.append_traj(tmp_traj, traj.poses[idx])
            traj_len = idx

        # self.traj_list = tmp_traj

        print("current flag :", self.flag)
        # move to a point
        if not self.flag:
            self.move_traj(tmp_traj, traj_len)

        # communicate with trajectory publisher
        self.flag = True
        self.to_traj_pub.publish(self.flag)

    def status_cb(self, data):
        self.isstatus = data
        self.ready = self.isstatus.robot_state
        # print("ready : ", self.ready)
        # RB_DATA.robot_state
        # 0 = None
        # 1 = Idle
        # 2 = Pause
        # 3 = Moving

    # current joint value(rad)
    def joint_cb(self, jnt):
        for i in range(6):
            self.jointAngles[i] = jnt.position[i]

    # get user command
    def msg_cb(self, cmd_msg):
        self.go_sign = cmd_msg.data

        if self.go_sign == "p":
            self.move_seq()
        elif self.go_sign == "m":
            self.move_test()
        elif self.go_sign == "s":
            self.stop_pub.publish(True)
        elif self.go_sign == "q":
            self.isshutdown = True
            self.node_shutdown_pub.publish(True)
        else:
            self.move_sig(self.go_sign)

    # Not USE
    def stop(self):
        """publish stop signal to robot

        msg :
            "task stop" -> quick stop that more stress for robot
            "task pause -> less stress for robot
        """

        cmd = rb_command()
        cmd.cmd = "task pause"
        print(cmd)
        self.rb_command_pub.publish(cmd)
        # cmd.cmd = "task stop"
        # self.rb_command_pub.publish(cmd)

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
            if self.ready == 1:
                return ROBOT
            elif self.ready == 3:
                ROBOT = self.BUSY_STATE
                return ROBOT
            else:
                print("Error!")
                return self.ERROR

        # ROBOT moving...
        elif ROBOT == self.BUSY_STATE:
            if self.ready == 3:
                return ROBOT
            elif self.ready == 1:
                ROBOT = self.READY_STATE
                return ROBOT
            else:
                print("Error!")
                return self.ERROR

        else:
            # ROBOT = self.READY_STATE
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
