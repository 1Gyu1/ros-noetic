#!/usr/bin/python3

import sys
import os
import math
import rospy
import tf
import copy

from PyKDL import (  # pylint: disable=no-name-in-module
    Chain,
    Segment,
    Frame,
    Vector,
    Rotation,
    JntArray,
    Joint,
)
from time import sleep
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose, Vector3
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage
from robot import Robot

# custom msg
from controll_robots.msg import rb_command, rb_data, robots_info

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

def indy7_chain():
    """build chain

    Returns:
        indy7 chain data
    """
    chain = Chain()
    listener = tf.TransformListener()
    trans = [0 for i in range(6)]
    rot = [0 for j in range(6)]

    while True:
        try:
            (trans[0], rot[0]) = listener.lookupTransform(
                "/link1", "/link2", rospy.Time(0)
            )
            (trans[1], rot[1]) = listener.lookupTransform(
                "/link2", "/link3", rospy.Time(0)
            )
            (trans[2], rot[2]) = listener.lookupTransform(
                "/link3", "/link4", rospy.Time(0)
            )
            (trans[3], rot[3]) = listener.lookupTransform(
                "/link4", "/link5", rospy.Time(0)
            )
            (trans[4], rot[4]) = listener.lookupTransform(
                "/link5", "/link6", rospy.Time(0)
            )
            (trans[5], rot[5]) = listener.lookupTransform(
                "/link6", "/tcp", rospy.Time(0)
            )
            break

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

    # Build chain Axis to wrist3
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
                Vector(trans[0][0], trans[0][1], trans[0][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[1][0], trans[1][1], trans[1][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
                Vector(trans[2][0], trans[2][1], trans[2][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
                Vector(trans[3][0], trans[3][1], trans[3][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
                Vector(trans[4][0], trans[4][1], trans[4][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[5][0], trans[5][1], trans[5][2]),
            ),
        )
    )
    return chain

def rb10_chain():
    """build chain

    Returns:
        rb10 chain data
    """
    chain = Chain()
    listener = tf.TransformListener()
    trans = [0 for i in range(6)]
    rot = [0 for j in range(6)]

    while True:
        try:
            (trans[0], rot[0]) = listener.lookupTransform(
                "/base", "/shoulder", rospy.Time(0)
            )
            (trans[1], rot[1]) = listener.lookupTransform(
                "/shoulder", "/elbow", rospy.Time(0)
            )
            (trans[2], rot[2]) = listener.lookupTransform(
                "/elbow", "/wrist1", rospy.Time(0)
            )
            (trans[3], rot[3]) = listener.lookupTransform(
                "/wrist1", "/wrist2", rospy.Time(0)
            )
            (trans[4], rot[4]) = listener.lookupTransform(
                "/wrist2", "/wrist3", rospy.Time(0)
            )
            (trans[5], rot[5]) = listener.lookupTransform(
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
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[0][0], trans[0][1], trans[0][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotY),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[1][0], trans[1][1], trans[1][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotY),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[2][0], trans[2][1], trans[2][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotY),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[3][0], trans[3][1], trans[3][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotZ),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[4][0], trans[4][1], trans[4][2]),
            ),
        )
    )
    chain.addSegment(
        Segment(
            Joint(Joint.RotY),
            Frame(
                Rotation.Quaternion(0, 0, 0, 1),
                Vector(trans[5][0], trans[5][1], trans[5][2]),
            ),
        )
    )
    return chain

class TaskController_RB(Robot):
    """Task Controller rb

    Args:
        chain : robot chain data
    """
    def __init__(self, robot_chain):
        super().__init__(robot_chain)
        self.ready = 0
        self.READY_STATE = 101
        self.BUSY_STATE = 202
        self.MOVING = 303
        self.ERROR = 404
        self.ROBOT = self.READY_STATE

        # Publisher
        self.rb_command_pub = rospy.Publisher("/rb_command", rb_command, queue_size=1)
        self.stop_pub = rospy.Publisher("/stop_rb", Bool, queue_size=1)
        self.jnt_pub = rospy.Publisher("/rb10/joint_val", Float32MultiArray, queue_size=1)
        self.node_shutdown_pub = rospy.Publisher("/rb10/shutdown_msg", Bool, queue_size=1)
        self.to_manager_pub = rospy.Publisher("/rb10/object_flag", Bool, queue_size=1)
        self.control_flag_pub = rospy.Publisher("/rb10/control_flag", String, queue_size=1)
        # self.robot_info_pub = rospy.Publisher("/r_info", robots_info, queue_size=100)

        # Subscriber
        self.rb_status_check_sub = rospy.Subscriber("/rb_data", rb_data, self.status_cb)
        self.rb_joint_sub = rospy.Subscriber("/rb10/joint_states", JointState, self.joint_cb)
        self.cmd_msg_sub = rospy.Subscriber("/rb10/cmd_msg", String, self.msg_cb)
        self.trajectory_sub = rospy.Subscriber(
            "/rb10/object_pose", Pose, self.get_object_cb
        )
        self.tcp_offset_sub = rospy.Subscriber("/rb10/tcp_offset", Vector3, self.get_tcp_offset)
        # self.control_flag_sub = rospy.Subscriber("/indy7/control_flag", String, self.control_flag_cb)

        self.ctl_flag = String()
        self.go_sign = String()
        # self.isstatus = GoalStatusArray()
        self.isshutdown = False
        self.flag = False
        self.isstop = False
        self.robot_tf = TFMessage()
        self.jointAngles = JntArray(6)
        self.traj_list = PoseArray()
        self.rb10_data = robots_info()
        self.waypoint = [0 for l in range(13)]
        self.pick_frame = Frame()
        self.offset = Vector3(0.0, 0.0, 0.0)
        self.tcp = Frame()

        # self.jointAngles[2] = self.jointAngles[4] = math.pi / 2
        self.count = 0

    def broadcast_tf(self):
        tcp_trans = self.tcp.p
        tcp_rot = self.tcp.M.GetQuaternion()
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (tcp_trans[0], tcp_trans[1], tcp_trans[2]), 
            (tcp_rot[0], tcp_rot[1], tcp_rot[2], tcp_rot[3]), 
            rospy.Time.now(), "<R_TCP>", 'Axis_0') 

        for idx in range(len(self.waypoint)):
            if self.waypoint[idx] != 0:
                pose = Pose()
                trans = self.waypoint[idx][0].p
                rot = Rotation.GetQuaternion(self.waypoint[idx][0].M)
                pose.position.x = trans[0]
                pose.position.y = trans[1]
                pose.position.z = trans[2]
                pose.orientation.x = rot[0]
                pose.orientation.y = rot[1]
                pose.orientation.z = rot[2]
                pose.orientation.w = rot[3]

                br = tf.TransformBroadcaster()
                br.sendTransform(
                    (pose.position.x, pose.position.y, 
                        pose.position.z), 
                    (pose.orientation.x, pose.orientation.y, 
                        pose.orientation.z, pose.orientation.w), 
                    rospy.Time.now(), "R_" + str(idx), 'Axis_0')                

    ############################# task move test###################################################################
    def move_sig(self, sig):
        """move robot a single way

        Args:
            sig : user input

        keyboard input(sig) guide:
            w         r
        a       d
            x         f
        """
        current_frame = Frame()
        goal_frame = Frame()
        error = False

        print("rb sig: ", sig)


        # Calc fk to get current frame
        # super().update_joint(self.jointAngles)
        # current_frame = super().get_frame()

        current_frame = self.convert_tcp()

        # ######## print for check #########
        # print(" ")
        # print("current joint angle")
        # print(self.jointAngles)
        # print("current_frame")
        # print(current_frame)
        # print("#####################")
        # ######## print for check end #####
       
        ######################## VECTOR #########################
        # 0.1 => 0.1m
        # vector x + 0.1
        if sig == "w":
            goal_frame = super().get_frame_XYZ(current_frame, _x=0.1)
        # vector x - 0.1
        elif sig == "x":
            goal_frame = super().get_frame_XYZ(current_frame, _x=-0.1)
        # vector y + 0.1
        elif sig == "a":
            goal_frame = super().get_frame_XYZ(current_frame, _y=0.1)
        # vector y - 0.1
        elif sig == "d":
            goal_frame = super().get_frame_XYZ(current_frame, _y=-0.1)
        # vector z + 0.1
        elif sig == "r":
            goal_frame = super().get_frame_XYZ(current_frame, _z=0.1)
        # vector z - 0.1
        elif sig == "f":
            goal_frame = super().get_frame_XYZ(current_frame, _z=-0.1)
        ######################## VECTOR end #####################

        ######################## ROTATION #######################
        # 0.1 => 17 deg
        # Rx + 0.1 rads
        elif sig == "y":
            goal_frame = super().get_frame_Rxyz(current_frame, rx=0.1)
        # Rx - 0.1 rads
        elif sig == "u":
            goal_frame = super().get_frame_Rxyz(current_frame, rx=-0.1)
        # Ry + 0.1 rads
        elif sig == "g":
            goal_frame = super().get_frame_Rxyz(current_frame, ry=0.1)
        # Ry - 0.1 rads
        elif sig == "h":
            goal_frame = super().get_frame_Rxyz(current_frame, ry=-0.1)
        # Rz + 0.1 rads
        elif sig == "b":
            goal_frame = super().get_frame_Rxyz(current_frame, rz=0.1)
        # Rz - 0.1 rads
        elif sig == "n":
            goal_frame = super().get_frame_Rxyz(current_frame, rz=-0.1)
        ######################## ROTATION end ###################

        # ########## test #########################################
        # elif sig == "j":
        #     print("sig : ", sig)
        #     goal_frame = Frame(current_frame.M, Vector(0.5, -0.5, 0.8))
        # elif sig == "k":
        #     goal_frame = Frame(current_frame.M, Vector(0.13, 0.756, 0.568))
        # elif sig == "l":
        #     goal_frame = Frame(current_frame.M, Vector(0.5, 0.5, 0.8))
        # ########## test end ######################################

        else:
            error = True
            print("Input Error!")
        # ######## print for check #########
        # print(" ")
        # print("goal_frame")
        # print(goal_frame)
        # print("#####################")
        # ######## print for check end #####joint_cb

        # goal_frame = self.reverse_tcp(goal_frame)

        if not error:
            print("go")
            self.moveJ(goal_frame)
    ############################# task move test end###############################################################

    def _move_traj(self, traj, traj_len):
        """move robot along trajectory

        Args:
            traj: PoseArray data that tcp will follow
            traj_len: no. of trajectory points
        """
        self.flag = True
        current_frame = Frame()
        goal_frame = []
        q_jnt = JntArray(6)
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        idx = 0

        for idx in range(traj_len):
            goal_frame.append(
                super().pose_to_frame(traj[idx])
            )

        while not self.isstop:
            if self.ROBOT == self.READY_STATE:
                print("idx :", idx)

                super().update_frame(goal_frame[idx])
                q_jnt = super().get_joint()

                for j in range(6):
                    q_list.data[j] = q_jnt[j]

                print("test traj q_list")
                print(q_list)
                idx += 1

                # Calc fk to get current frame
                super().update_joint(self.jointAngles)
                current_frame = super().get_frame()
                if goal_frame[idx] != current_frame:
                    self.jnt_pub.publish(q_list)
                    self.ROBOT = self.MOVING

            self.ROBOT = self.isReadyState(self.ROBOT)
            
            if idx >= traj_len:
                break
            if self.ROBOT == self.ERROR:
                print("Error!")
                break
            if self.isshutdown:
                break

        self.flag = False
        self.isstop = False
        print("flag :", self.flag)
    
    def traj_move(self, traj, traj_len):
        """move robot along trajectory 

        Args:
            traj: list of Frame data that tcp will follow
            traj_len: no. of trajectory points
        """
        current_frame = Frame()
        goal_frame = Frame()
        q_jnt = JntArray(6)
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        idx = 0

        # compare first traj point with current 
        # print("tarj", traj)
        super().update_joint(self.jointAngles)
        current_frame = super().get_frame()
        # super().update_frame(traj[idx])
        # q_jnt = super().get_joint()
        # if JntArray.Equal(self.jointAngles, q_jnt):
        #     idx = 1
        # else:
        #     idx = 0 
        if current_frame == traj[idx]:
            idx = 1

        while not self.isstop:
            if (idx >= traj_len) and (self.ROBOT == self.READY_STATE):
                print("Done!")
                break
            if self.ROBOT == self.READY_STATE:
                # print("idx :", idx)  
                # goal_frame = traj[idx]
                goal_frame = self.reverse_tcp(traj[idx])

                super().update_frame(goal_frame)
                q_jnt = super().get_joint()

                for j in range(6):
                    q_list.data[j] = q_jnt[j]

                # print("traj q_list")
                # print(q_list)
                idx += 1

                # nomalize joint radian value
                for j in range(6):
                    tmp_q = q_jnt[j] // math.pi
                    tmp_r = q_jnt[j] % math.pi
                    if tmp_q % 2:
                        q_list.data[j] = tmp_r - math.pi
                    else:
                        q_list.data[j] = tmp_r

                self.jnt_pub.publish(q_list)
                self.ROBOT = self.MOVING

            self.ROBOT = self.isReadyState(self.ROBOT)

            if self.ROBOT == self.ERROR:
                print("Error! - traj move")
                break
            if self.isshutdown:
                print("Program shutdown!")
                break

    def moveJ(self, point):
        """movej

        Args:
            point: goal point Frame data
        """
        tmp = []
        tmp.append(point)
        print("tmp frame:", tmp)
        self.traj_move(tmp, 1)

    def moveL(self, point, unit=0.01):
        """moveL

        Args:
            point: goal point Frame data
            unit: unit distance (m)
        """
        super().update_joint(self.jointAngles)
        current_frame = super().get_frame()
        
        # generate trajectory every 1cm(unit)
        (traj, traj_len) = self.generate_traj(current_frame, point, unit)
        self.traj_move(traj, traj_len)
    
    def generate_traj(self, v1, v2, unit):
        """generate trajectory current tcp frame to point

        Args:
            v1: frame 1 (start point)
            v2: frame 2 (end point)
            unit: unit distance (m)

        Returns:
            trajectory generated with PoseArray data,
            number of trajectory point
        """
        traj = []
        dist_x = v2.p[0] - v1.p[0]
        dist_y = v2.p[1] - v1.p[1]
        dist_z = v2.p[2] - v1.p[2]
        dist = math.sqrt(
            math.pow(dist_x, 2) +
            math.pow(dist_y, 2) +
            math.pow(dist_z, 2)
        )
        # dist = self.get_distance(v1.p, v2.p)

        # trajectory number of points
        n = int(dist // unit)
        print("dist n:", n)
        if n == 0:
            n = 1
        _x = dist_x / n
        _y = dist_y / n
        _z = dist_z / n

        for idx in range(1, n+1):
            _traj = Frame()
            _traj.p[0] = v1.p[0] + _x * idx
            _traj.p[1] = v1.p[1] + _y * idx
            _traj.p[2] = v1.p[2] + _z * idx
            _traj.M = v2.M
            traj.append(_traj)

        return traj, n

    def save_point(self, idx, _type='j'):
        """save waypoint

        Args:
            idx: waypoints index number(0~11)
            type: move type
        """
        # super().update_joint(self.jointAngles)
        # save_point = super().get_frame()
        save_point = self.convert_tcp()
        self.waypoint[idx] = [save_point, _type]
        print("saved point: ",idx)
        print(self.waypoint)
      
    def job_play(self):
        idx = 0
        for idx in range(12):
            print("job_move idx: ", idx)
            if self.waypoint[idx] == 0:
                pass
            elif self.waypoint[idx][1] == 'j':
                print("moveJ")
                self.moveJ(self.waypoint[idx][0])
            elif self.waypoint[idx][1] == 'l':
                print("moveL")
                self.moveL(self.waypoint[idx][0])
        self.moveJ(self.waypoint[0][0])
    
    def go_loop(self):
        print("Loop play")
        while self.ROBOT != self.ERROR:
            idx = 0
            for idx in range(12):
                if self.isstop or self.isshutdown:
                    break
                print("job_move idx: ", idx)
                if self.waypoint[idx] == 0:
                    pass
                elif self.waypoint[idx][1] == 'j':
                    print("moveJ")
                    self.moveJ(self.waypoint[idx][0])
                elif self.waypoint[idx][1] == 'l':
                    print("moveL")
                    self.moveL(self.waypoint[idx][0])
            # self.moveJ(self.waypoint[0][0])

            if self.isstop or self.isshutdown:
                    break
        

    def zero(self):
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        print("Zero Position")
        q = [0, 0, 0, 0, 0, 0]
        q = degs2rads(q)
        for l, k in enumerate(q):
            q_list.data[l] = k
        self.jnt_pub.publish(q_list)

    def home(self):
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        print("Home Position")
        q = [0, 0, 90, 0, 90, 0]
        q = degs2rads(q)
        for l, k in enumerate(q):
            q_list.data[l] = k
        self.jnt_pub.publish(q_list)

    def pick_and_place_demo(self):
        # super().update_joint(self.jointAngles)
        # fr = super().get_frame()
        fr = self.convert_tcp()
        fr = super().get_frame_XYZ(fr, _x=-0.2, _y=-0.1)
        self.waypoint[0] = [fr, 'j']    # standby
        
        ############## pick ##################
        fr = super().get_frame_XYZ(fr, _x=0.0, _y=-0.3, _z=0.2)
        self.waypoint[1] = [fr, 'j']            # above bin box 
        fr = super().get_frame_XYZ(fr, _z=-0.2)
        self.waypoint[2] = [fr, 'j']            # prepick point
        fr = super().get_frame_XYZ(fr, _z=-0.03)
        self.waypoint[3] = [fr, 'j']    # L        # pick point
        fr = super().get_frame_XYZ(fr, _z=0.03)
        self.waypoint[4] = [fr, 'j']    # L        # postpick point
        fr = super().get_frame_XYZ(fr, _z=0.2)
        self.waypoint[5] = [fr, 'j']

        fr = super().get_frame_XYZ(fr, _x=-0.1, _y=0.3)
        self.waypoint[12] = self.waypoint[6] = [fr, 'j']     # mid point

        ################ place ################
        fr = super().get_frame_XYZ(fr, _x=0.1, _y=0.5, _z=-0.2)
        self.waypoint[7] = [fr, 'j']    
        fr = super().get_frame_XYZ(fr, _z=-0.1)
        self.waypoint[8] = [fr, 'j'] 
        fr = super().get_frame_XYZ(fr, _z=-0.03)
        self.waypoint[9] = [fr, 'j']        # L
        fr = super().get_frame_XYZ(fr, _z=0.03)
        self.waypoint[10] = [fr, 'j']       # L
        fr = super().get_frame_XYZ(fr, _z=0.1)
        self.waypoint[11] = [fr, 'j']    


        # change pick pose to given data
        # self.convert_pick_pose(obj)

    def convert_pick_pose(self, pick_frame):
        """adjust pick pose to real object
        """
        # pre_pick -> object z-axis offset
        # post_pick -> world z-axis offset
        
        pick_frame = pick_frame * Frame(Rotation.RPY(1.5709, 0, 0), Vector(0, 0, 0.01))
        pre_pick = pick_frame * Frame(Vector(0, 0.05, 0))
        post_pick = copy.deepcopy(pick_frame)
        post_pick.p = Vector(
                post_pick.p[0],
                post_pick.p[1],
                post_pick.p[2] + 0.05
            )

        self.waypoint[3][0] = pick_frame
        self.waypoint[2][0] = pre_pick
        self.waypoint[4][0] = post_pick

    def convert_tcp(self):
        """add tool and get tcp frame

        Returns:
            tcp: real tcp frame
        """
        super().update_joint(self.jointAngles)
        tool_flange = super().get_frame()
        self.tcp = tool_flange * Frame(Vector(self.offset.x, self.offset.y, self.offset.z))

        return self.tcp

        # waypoint[5] = tcp

    def reverse_tcp(self, _frame):
        """get tool flange frame

        Returns:
            flange: flange frame before attach tool
        """
        tcp = _frame * Frame(
            Vector(
                -1.0 * self.offset.x,
                -1.0 * self.offset.y,
                -1.0 * self.offset.z))

        return tcp


    ###############################################################
    ################# Subscriber cb funcs #########################
    ###############################################################
    def get_tcp_offset(self, data):
        self.offset.z = data.x
        self.offset.x = data.y
        self.offset.y = -1 * data.z
        
    def get_object_cb(self, object):
        self.pick_frame = super().pose_to_frame(object)
        print("get pick frame")
        print(self.pick_frame)
        # self.cowork_status = 'get_object'
        self.convert_pick_pose(self.pick_frame)

        self.flag = True
        self.to_manager_pub.publish(self.flag) 
        print("current flag :", self.flag)
        self.flag = False

        # execute pick and place about given object data
        # PLAY pick and place  

    def status_cb(self, status):
        # RB_DATA.robot_state
        # 0 = None
        # 1 = Idle
        # 2 = Pause
        # 3 = Moving
        self.rb10_data.robot_status_rb10 = self.ready = status.robot_state
        # self.robot_info_pub.publish(self.rb10_data)
        # print("ready : ", self.ready)

    # current joint value(rad)
    def joint_cb(self, jnt):
        fr = self.convert_tcp()
        for i in range(6):
            self.rb10_data.joint_rb10[i] = self.jointAngles[i] = jnt.position[i]

        # super().update_joint(self.jointAngles)
        # fr = super().get_frame()
        rot = Rotation.GetRPY(fr.M)
        trans = fr.p
        for idx, val in enumerate(trans):
            self.rb10_data.tcp_rb10[idx] = val
        for idx, val in enumerate(rot, start=3):
            self.rb10_data.tcp_rb10[idx] = math.degrees(val)

    # get user command
    def msg_cb(self, cmd_msg):
        self.go_sign.data = cmd_msg.data
        if self.go_sign.data == "s":
            self.isstop = True
            self.stop_pub.publish(True)
            # self.isstop = False
        elif self.go_sign.data == "q":
            self.isshutdown = True
            self.node_shutdown_pub.publish(True)
        elif self.go_sign.data == "0":
            self.zero()
        elif self.go_sign.data == "1":
            self.home()

        ########## pick and place ######################################
        elif "way" in self.go_sign.data:
            self.save_point(int(self.go_sign.data[3:6]), self.go_sign.data[6])
        elif "del" in self.go_sign.data:
            self.waypoint[int(self.go_sign.data[3:6])] = 0
        elif self.go_sign.data == "goloop":
            self.go_loop()
        elif self.go_sign.data == "go":
            self.job_play()
        elif "go" in self.go_sign.data:
            if self.waypoint[int(self.go_sign.data[2:])] != 0:
                self.moveJ(self.waypoint[int(self.go_sign.data[2])][0])
            else:
                print("there is no ", self.go_sign.data[2:5], "waypoint")
        elif self.go_sign.data == "pick":
            self.home()
            self.pick_and_place_demo()
        ########## pick and place end ##################################

        else:
            self.move_sig(self.go_sign.data)

    def control_flag_cb(self, ctl_flag_msg):
        """Set flag to control 2 robots concurrently
        """
        self.ctl_flag.data = ctl_flag_msg.data
    ###############################################################
    ################# Subscriber cb funcs end #####################
    ###############################################################


    # Not USE
    def stop(self):
        """publish stop signal to robot"""
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
          ROBOT: previous robot status

        Returns:
          current robot state

        Raises:
          RobotError: If robot status is not idle or moving
        """
        # ROBOT start moving
        if ROBOT == self.MOVING:
            if self.ready == 1: pass
            elif self.ready == 3:
                ROBOT = self.BUSY_STATE
            else:
                print("Error!")
                ROBOT =  self.ERROR
        # ROBOT moving...
        elif ROBOT == self.BUSY_STATE:
            if self.ready == 3: pass
            elif self.ready == 1:
                ROBOT = self.READY_STATE
            else:
                print("Error!")
                ROBOT = self.ERROR
        
        # print("Robot state:", ROBOT)
        return ROBOT


class TaskController_INDY(Robot):
    """Task Controller indy

    Args:
        chain : robot chain data
    """
    def __init__(self, robot_chain):
        # rospy.init_node("task_controller_indy7")
        super().__init__(robot_chain)
        self.ready = 0
        self.READY_STATE = 101
        self.BUSY_STATE = 202
        self.MOVING = 303
        self.ERROR = 404
        self.ROBOT = self.READY_STATE

        # Publisher
        self.stop_pub = rospy.Publisher("/stop_motion", Bool, queue_size=1)
        self.jnt_pub = rospy.Publisher("/indy7/joint_val", Float32MultiArray, queue_size=1)
        self.node_shutdown_pub = rospy.Publisher("/indy7/shutdown_msg", Bool, queue_size=1)
        self.to_manager_pub = rospy.Publisher("/indy7/object_flag", Bool, queue_size=1)
        self.control_flag_pub = rospy.Publisher("/indy7/control_flag", String, queue_size=1)
        # self.robot_info_pub = rospy.Publisher("/r_info", robots_info, queue_size=100)

        # Subscriber
        self.indy_status_check_sub = rospy.Subscriber("/indy7/status", GoalStatusArray, self.status_cb)
        self.indy_joint_sub = rospy.Subscriber("/indy7/joint_states", JointState, self.joint_cb)
        self.cmd_msg_sub = rospy.Subscriber("/indy7/cmd_msg", String, self.msg_cb)
        self.trajectory_sub = rospy.Subscriber(
            "/indy7/object_pose", Pose, self.get_object_cb
        )
        self.tcp_offset_sub = rospy.Subscriber("/indy7/tcp_offset", Vector3, self.get_tcp_offset)
        # self.control_flag_sub = rospy.Subscriber("/rb10/control_flag", String, self.control_flag_cb)

        self.ctl_flag = String()
        self.go_sign = String()
        self.isstatus = GoalStatusArray()
        self.isshutdown = False
        self.flag = False
        self.isstop = False
        self.robot_tf = TFMessage()
        self.jointAngles = JntArray(6)
        self.traj_list = PoseArray()
        self.indy7_data = robots_info()
        self.waypoint = [0 for l in range(13)]
        self.pick_frame = Frame()
        self.offset = Vector3(0.0, 0.0, 0.0)
        self.tcp = Frame()

        # self.jointAngles[2] = self.jointAngles[4] = math.pi / 2
        self.count = 0

    def broadcast_tf(self):
        tcp_trans = self.tcp.p
        tcp_rot = self.tcp.M.GetQuaternion()
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (tcp_trans[0], tcp_trans[1], tcp_trans[2] + 0.177), 
            (tcp_rot[0], tcp_rot[1], tcp_rot[2], tcp_rot[3]), 
            rospy.Time.now(), "<I_TCP>", 'world') 
        
        for idx in range(len(self.waypoint)):
            if self.waypoint[idx] != 0:
                pose = Pose()
                trans = self.waypoint[idx][0].p
                rot = Rotation.GetQuaternion(self.waypoint[idx][0].M)
                pose.position.x = trans[0]
                pose.position.y = trans[1]
                pose.position.z = trans[2]
                pose.orientation.x = rot[0]
                pose.orientation.y = rot[1]
                pose.orientation.z = rot[2]
                pose.orientation.w = rot[3]

                br = tf.TransformBroadcaster()
                br.sendTransform(
                    (pose.position.x, pose.position.y, 
                        pose.position.z + 0.177), 
                    (pose.orientation.x, pose.orientation.y, 
                        pose.orientation.z, pose.orientation.w), 
                    rospy.Time.now(), "I_" + str(idx), 'world') 

    ############################# task move test###################################################################
    def move_sig(self, sig):
        """move robot a single way

        Args:
            sig : user input

        keyboard input(sig) guide:
            w         r
        a       d
            x         f
        """
        current_frame = Frame()
        goal_frame = Frame()
        error = False

        print("indy sig: ", sig)

        # Calc fk to get current frame
        # super().update_joint(self.jointAngles)
        # current_frame = super().get_frame()

        current_frame = self.convert_tcp()

        # ######## print for check #########
        # print(" ")
        # print("current joint angle")
        # print(self.jointAngles)
        # print("current_frame")
        # print(current_frame)
        # print("#####################")
        # ######## print for check end #####
       
        ######################## VECTOR #########################
        # 0.1 => 0.1m
        # vector x + 0.1
        if sig == "w":
            goal_frame = super().get_frame_XYZ(current_frame, _x=0.01)
        # vector x - 0.1
        elif sig == "x":
            goal_frame = super().get_frame_XYZ(current_frame, _x=-0.01)
        # vector y + 0.1
        elif sig == "a":
            goal_frame = super().get_frame_XYZ(current_frame, _y=0.01)
        # vector y - 0.1
        elif sig == "d":
            goal_frame = super().get_frame_XYZ(current_frame, _y=-0.01)
        # vector z + 0.1
        elif sig == "r":
            goal_frame = super().get_frame_XYZ(current_frame, _z=0.01)
        # vector z - 0.1
        elif sig == "f":
            goal_frame = super().get_frame_XYZ(current_frame, _z=-0.01)
        ######################## VECTOR end #####################

        ######################## ROTATION #######################
        # 0.1 => 17 deg
        # Rx + 0.1 rads
        elif sig == "y":
            goal_frame = super().get_frame_Rxyz(current_frame, rx=0.1)
        # Rx - 0.1 rads
        elif sig == "u":
            goal_frame = super().get_frame_Rxyz(current_frame, rx=-0.1)
        # Ry + 0.1 rads
        elif sig == "g":
            goal_frame = super().get_frame_Rxyz(current_frame, ry=0.1)
        # Ry - 0.1 rads
        elif sig == "h":
            goal_frame = super().get_frame_Rxyz(current_frame, ry=-0.1)
        # Rz + 0.1 rads
        elif sig == "b":
            goal_frame = super().get_frame_Rxyz(current_frame, rz=0.1)
        # Rz - 0.1 rads
        elif sig == "n":
            goal_frame = super().get_frame_Rxyz(current_frame, rz=-0.1)
        ######################## ROTATION end ###################

        ########## test #########################################
        # elif sig == "j":
        #     print("sig : ", sig)
        #     goal_frame = Frame(current_frame.M, Vector(0.5, -0.5, 0.8))
        # elif sig == "k":
        #     goal_frame = Frame(current_frame.M, Vector(0.13, 0.756, 0.568))
        # elif sig == "l":
        #     goal_frame = Frame(current_frame.M, Vector(0.5, 0.5, 0.8))
        ########## test end ######################################

        else:
            error = True
            print("Input Error!")
        # ######## print for check #########
        # print(" ")
        # print("goal_frame")
        # print(goal_frame)
        # print("#####################")
        # ######## print for check end #####

        # goal_frame = self.reverse_tcp(goal_frame)

        if not error:
            self.moveJ(goal_frame)
    ############################# task move test end###############################################################

    def _move_traj(self, traj, traj_len):
        """move robot along trajectory

        Args:
            traj: PoseArray data that tcp will follow
            traj_len: no. of trajectory points
        """
        self.flag = True
        current_frame = Frame()
        goal_frame = []
        q_jnt = JntArray(6)
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        idx = 0

        for idx in range(traj_len):
            goal_frame.append(
                self.pose_to_frame(traj[idx])
            )

        while not self.isstop:
            if self.ROBOT == self.READY_STATE:
                print("idx :", idx)

                super().update_frame(goal_frame[idx])
                q_jnt = super().get_joint()

                for j in range(6):
                    q_list.data[j] = q_jnt[j]

                print("test traj q_list")
                print(q_list)
                idx += 1

                # Calc fk to get current frame
                super().update_joint(self.jointAngles)
                current_frame = super().get_frame()
                if goal_frame[idx] != current_frame:
                    self.jnt_pub.publish(q_list)
                    self.ROBOT = self.MOVING

            self.ROBOT = self.isReadyState(self.ROBOT)
            
            if idx >= traj_len:
                break
            if self.ROBOT == self.ERROR:
                print("Error!")
                break
            if self.isshutdown:
                break

        self.flag = False
        self.isstop = False
        print("flag :", self.flag)
    
    def traj_move(self, traj, traj_len):
        """move robot along trajectory 

        Args:
            traj: list of Frame data that tcp will follow
            traj_len: no. of trajectory points
        """
        current_frame = Frame()
        goal_frame = Frame()
        q_jnt = JntArray(6)
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        idx = 0

        # compare first traj point with current 
        # print("tarj", traj)
        super().update_joint(self.jointAngles)
        current_frame = super().get_frame()
        # super().update_frame(traj[idx])
        # q_jnt = super().get_joint()
        # if JntArray.Equal(self.jointAngles, q_jnt):
        #     idx = 1
        # else:
        #     idx = 0 
        if current_frame == traj[idx]:
            idx = 1

        # self.ROBOT == self.READY_STATE

        while not self.isstop:
            if (idx >= traj_len) and (self.ROBOT == self.READY_STATE):
            # if (idx >= traj_len):
                print("Done!")
                break
            if self.ROBOT == self.READY_STATE:
                # print("idx :", idx)  
                # goal_frame = traj[idx]
                goal_frame = self.reverse_tcp(traj[idx])

                super().update_frame(goal_frame)
                q_jnt = super().get_joint()

                for j in range(6):
                    q_list.data[j] = q_jnt[j]

                print("traj q_list")
                print(q_list)
                idx += 1

                # nomalize joint radian value
                for j in range(6):
                    tmp_q = q_jnt[j] // math.pi
                    tmp_r = q_jnt[j] % math.pi
                    if tmp_q % 2:
                        q_list.data[j] = tmp_r - math.pi
                    else:
                        q_list.data[j] = tmp_r

                self.jnt_pub.publish(q_list)
                self.ROBOT = self.MOVING
            self.ROBOT = self.isReadyState(self.ROBOT)

            if self.ROBOT == self.ERROR:
                print("Error! - traj move")
                break
            if self.isshutdown:
                print("Program shutdown!")
                break

    def moveJ(self, point):
        """movej

        Args:
            point: goal point Frame data
        """
        tmp = []
        tmp.append(point)
        print("tmp frame:", tmp)
        self.traj_move(tmp, 1)

    def moveL(self, point, unit=0.01):
        """moveL

        Args:
            point: goal point Frame data
            unit: unit distance (m)
        """
        super().update_joint(self.jointAngles)
        current_frame = super().get_frame()
        
        # generate trajectory every 1cm(unit)
        (traj, traj_len) = self.generate_traj(current_frame, point, unit)
        self.traj_move(traj, traj_len)
    
    def generate_traj(self, v1, v2, unit):
        """generate trajectory current tcp frame to point

        Args:
            v1: frame 1 (start point)
            v2: frame 2 (end point)
            unit: unit distance (m)

        Returns:
            trajectory generated with PoseArray data,
            number of trajectory point
        """
        traj = []
        dist_x = v2.p[0] - v1.p[0]
        dist_y = v2.p[1] - v1.p[1]
        dist_z = v2.p[2] - v1.p[2]
        dist = math.sqrt(
            math.pow(dist_x, 2) +
            math.pow(dist_y, 2) +
            math.pow(dist_z, 2)
        )
        # dist = self.get_distance(v1.p, v2.p)

        # trajectory number of points
        n = int(dist // unit)
        print("dist n:", n)
        if n == 0:
            n = 1
        _x = dist_x / n
        _y = dist_y / n
        _z = dist_z / n

        for idx in range(1, n+1):
            _traj = Frame()
            _traj.p[0] = v1.p[0] + _x * idx
            _traj.p[1] = v1.p[1] + _y * idx
            _traj.p[2] = v1.p[2] + _z * idx
            _traj.M = v2.M
            traj.append(_traj)

        return traj, n

    def save_point(self, idx, _type='j'):
        """save waypoint

        Args:
            idx: waypoints index number(0~11)
            type: move type
        """
        # super().update_joint(self.jointAngles)
        # save_point = super().get_frame()
        save_point = self.convert_tcp()
        self.waypoint[idx] = [save_point, _type]
        print("saved point: ",idx)
        print(self.waypoint)

    def job_play(self):
        idx = 0
        for idx in range(12):
            print("job_move idx: ", idx)
            if self.waypoint[idx] == 0:
                pass
            elif self.waypoint[idx][1] == 'j':
                print("moveJ")
                self.moveJ(self.waypoint[idx][0])
            elif self.waypoint[idx][1] == 'l':
                print("moveL")
                self.moveL(self.waypoint[idx][0])
        self.moveJ(self.waypoint[0][0])

    def go_loop(self):
        print("Loop play")
        while self.ROBOT != self.ERROR:
            idx = 0
            for idx in range(12):
                if self.isstop or self.isshutdown:
                    break
                print("job_move idx: ", idx)
                if self.waypoint[idx] == 0:
                    pass
                elif self.waypoint[idx][1] == 'j':
                    print("moveJ")
                    self.moveJ(self.waypoint[idx][0])
                elif self.waypoint[idx][1] == 'l':
                    print("moveL")
                    self.moveL(self.waypoint[idx][0])
            # self.moveJ(self.waypoint[0][0])

            if self.isstop or self.isshutdown:
                    break

    def zero(self):
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        print("Zero Position")
        q = [0, 0, 0, 0, 0, 0]
        q = degs2rads(q)
        for l, k in enumerate(q):
            q_list.data[l] = k
        self.jnt_pub.publish(q_list)

    def home(self):
        q_list = Float32MultiArray()
        q_list.data = [0 for l in range(6)]
        print("Home Position")
        q = [0, 0, -90, 0, -90, 0]
        q = degs2rads(q)
        for l, k in enumerate(q):
            q_list.data[l] = k
        self.jnt_pub.publish(q_list)

    def pick_and_place_demo(self):
        # super().update_joint(self.jointAngles)
        # fr = super().get_frame()
        fr = self.convert_tcp()
        fr = super().get_frame_XYZ(fr, _x=0.1, _y=0.2)
        self.waypoint[0] = [fr, 'j']    # stanby
        
        ############## pick ##################
        fr = super().get_frame_XYZ(fr, _x=0.1, _y=0.2, _z=0.1)
        self.waypoint[1] = [fr, 'j']            # above bin box
        fr = super().get_frame_XYZ(fr, _z=-0.2)
        self.waypoint[2] = [fr, 'j']            # prepick point
        fr = super().get_frame_XYZ(fr, _z=-0.03)
        self.waypoint[3] = [fr, 'j']   # L         # pick point
        fr = super().get_frame_XYZ(fr, _z=0.03)
        self.waypoint[4] = [fr, 'j']   # L         # postpick point
        fr = super().get_frame_XYZ(fr, _z=0.2)
        self.waypoint[5] = [fr, 'j']

        fr = super().get_frame_XYZ(fr, -0.1, -0.3, 0.1)
        self.waypoint[12] = self.waypoint[6] = [fr, 'j']     # mid point

        ################ place ################
        fr = super().get_frame_XYZ(fr, _x=-0.1, _y=-0.3, _z=-0.2)
        self.waypoint[7] = [fr, 'j']    
        fr = super().get_frame_XYZ(fr, _z=-0.2)
        self.waypoint[8] = [fr, 'j'] 
        fr = super().get_frame_XYZ(fr, _z=-0.03)
        self.waypoint[9] = [fr, 'j']        # L
        fr = super().get_frame_XYZ(fr, _z=0.03)
        self.waypoint[10] = [fr, 'j']       # L
        fr = super().get_frame_XYZ(fr, _z=0.2)
        self.waypoint[11] = [fr, 'j'] 

        # change pick pose to given data
        # self.convert_pick_pose(obj)

    def convert_pick_pose(self, pick_frame):
        """adjust pick pose to real object
        """
        # pre_pick -> object z-axis offset
        # post_pick -> world z-axis offset
        
        pick_frame = pick_frame * Frame(Rotation.RPY(3.141592, 0, 1.5709), Vector(0, 0, 0.01))
        pre_pick = pick_frame * Frame(Vector(0, 0, -0.05))
        post_pick = copy.deepcopy(pick_frame)
        post_pick.p = Vector(
                post_pick.p[0],
                post_pick.p[1],
                post_pick.p[2] + 0.05
            )

        self.waypoint[3][0] = pick_frame
        self.waypoint[2][0] = pre_pick
        self.waypoint[4][0] = post_pick

    def convert_tcp(self):
        """add tool and get tcp frame

        Returns:
            tcp: real tcp frame
        """
        super().update_joint(self.jointAngles)                                                                                                                                                                                                                                                                                                                                                                                                      
        tool_flange = super().get_frame()
        self.tcp = tool_flange * Frame(Vector(self.offset.x, self.offset.y, self.offset.z))

        return self.tcp

    def reverse_tcp(self, _frame):
        """get tool flange frame

        Returns:
            flange: flange frame before attach tool
        """
        flange = _frame * Frame(
            Vector(
                -1.0 * self.offset.x,
                -1.0 * self.offset.y,
                -1.0 * self.offset.z))

        return flange

        # waypoint[5] = tcp
    
    ###############################################################
    ################# Subscriber cb funcs #########################
    ###############################################################
    def get_tcp_offset(self, data):
        self.offset.x = -1.0 * data.x
        self.offset.y = data.y
        self.offset.z = data.z

    def get_object_cb(self, object):
        self.pick_frame = super().pose_to_frame(object)
        print("get pick frame")
        print(self.pick_frame)
        # self.cowork_status = 'get_object'
        self.convert_pick_pose(self.pick_frame)

        self.flag = True
        self.to_manager_pub.publish(self.flag) 
        print("current flag :", self.flag)
        self.flag = False

        # execute pick and place about given object data
        # PLAY pick and place  

    def status_cb(self, status):
        self.indy7_data.robot_status_indy7 = self.ready = status.status_list[0].status
        # self.robot_info_pub.publish(self.indy7_data)
        # print("ready : ", self.ready)
        # RB_DATA.robot_state
        # 0 = Idle
        # 1 = Moving
        # 2 = Direct Teaching

    # current joint value(rad)
    def joint_cb(self, jnt):
        fr = self.convert_tcp()
        for i in range(6):
            self.indy7_data.joint_indy7[i] = self.jointAngles[i] = jnt.position[i]

        # super().update_joint(self.jointAngles)
        # fr = super().get_frame()
        rot = Rotation.GetRPY(fr.M)
        trans = fr.p
        for idx, val in enumerate(trans):
            self.indy7_data.tcp_indy7[idx] = val
        for idx, val in enumerate(rot, start=3):
            self.indy7_data.tcp_indy7[idx] = math.degrees(val)

    # get user command
    def msg_cb(self, cmd_msg):
        self.go_sign.data = cmd_msg.data
        if self.go_sign.data == "s":
            self.isstop = True
            self.stop_pub.publish(True)
        elif self.go_sign.data == "q":
            self.isshutdown = True
            self.node_shutdown_pub.publish(True)
        elif self.go_sign.data == "0":
            self.zero()
        elif self.go_sign.data == "1":
            self.home()

        ########## pick and place ######################################
        elif "way" in self.go_sign.data:
            self.save_point(int(self.go_sign.data[3:6]), self.go_sign.data[6])
        elif "del" in self.go_sign.data:
            self.waypoint[int(self.go_sign.data[3:6])] = 0
        elif self.go_sign.data == "goloop":
            self.go_loop()
        elif self.go_sign.data == "go":
            self.job_play()
        elif "go" in self.go_sign.data:
            if self.waypoint[int(self.go_sign.data[2:])] != 0:
                self.moveJ(self.waypoint[int(self.go_sign.data[2])][0])
            else:
                print("there is no ", self.go_sign.data[2:5], "waypoint")
        elif self.go_sign.data == "pick":
            self.home()
            self.pick_and_place_demo()
        ########## pick and place end ##################################
        else:
            self.move_sig(self.go_sign.data)

    def control_flag_cb(self, ctl_flag_msg):
        """Set flag to control 2 robots concurrently
        """
        self.ctl_flag.data = ctl_flag_msg.data
    ###############################################################
    ################# Subscriber cb funcs end #####################
    ###############################################################

    # Not USE
    def stop(self):
        """publish stop signal to robot"""
        self.stop_pub.publish(True)

    # Robot status func
    def isReadyState(self, ROBOT):
        """ROBOT status check and convert

        Args:
          ROBOT: previous robot status

        Returns:
          current robot state

        Raises:
          RobotError: If robot status is not idle or moving
        """
        # ROBOT start moving
        if ROBOT == self.MOVING:
            if self.ready == 0: pass
            elif self.ready == 1:
                ROBOT = self.BUSY_STATE
            else:
                print("Error!")
                ROBOT =  self.ERROR
        # ROBOT moving...
        elif ROBOT == self.BUSY_STATE:
            if self.ready == 1: pass
            elif self.ready == 0:
                ROBOT = self.READY_STATE
            else:
                print("Error!")
                ROBOT = self.ERROR
        
        # print("Robot state:", ROBOT)
        return ROBOT



def main():
    rospy.init_node("task_controller")
    # rb10_chain = Chain()
    # indy7_chain = Chain()
    _rb10_chain = rb10_chain()
    _indy7_chain = indy7_chain()
    rb10 = TaskController_RB(_rb10_chain)
    indy7 = TaskController_INDY(_indy7_chain)
    robot_data = robots_info()
    r_data_pub = rospy.Publisher("/r_info", robots_info, queue_size=10)

    while not rospy.is_shutdown():
        rb10.broadcast_tf()
        indy7.broadcast_tf()

        # data update
        robot_data.joint_indy7 = indy7.indy7_data.joint_indy7
        robot_data.tcp_indy7 = indy7.indy7_data.tcp_indy7
        robot_data.robot_status_indy7 = indy7.indy7_data.robot_status_indy7

        robot_data.joint_rb10 = rb10.rb10_data.joint_rb10
        robot_data.tcp_rb10 = rb10.rb10_data.tcp_rb10
        robot_data.robot_status_rb10 = rb10.rb10_data.robot_status_rb10

        r_data_pub.publish(robot_data)

        if rb10.isshutdown or indy7.isshutdown:
            break
    # rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass