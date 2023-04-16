#!/usr/bin/env python3

import copy
import math
from turtle import pos
import rospy
import tf

from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from PyKDL import (  # pylint: disable=no-name-in-module
    Chain,
    Segment,
    Joint,
    Frame,
    Vector,
    ChainFkSolverPos_recursive,
    ChainIkSolverVel_pinv,
    ChainIkSolverPos_NR,
    JntArray,
    Rotation,
)

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads
class Robot:
    """Set robot chain and calc ik/fk

    Args:
        robot_chain: robot chain data
    """
    def __init__(self, robot_chain):
        # self.robot_model = robot_model
        self.chain = robot_chain
        self.frame = Frame()

        self.ready = 0
        self.READY_STATE = 101
        self.BUSY_STATE = 202
        self.MOVING = 303
        self.ERROR = 404
        self.ROBOT = self.READY_STATE
        self.flag = False

        ############ solver to calc kinematics(fk/ik) ##############
        self.fksolverpos = ChainFkSolverPos_recursive(self.chain)
        self.iksolver1v = ChainIkSolverVel_pinv(self.chain)
        self.iksolverpos = ChainIkSolverPos_NR(
            self.chain, self.fksolverpos, self.iksolver1v
        )
        
        self.joints = JntArray(self.chain.getNrOfJoints())
        self.goal_joints = JntArray(self.chain.getNrOfJoints())

    def update_status(self, status):
        self.ready = status

    def update_joint(self, joints):
        """update joints and calc forward kinematics

        Args:
            joints: robot current joint angles (rads)
        """
        for idx, joint in enumerate(joints):
            self.joints[idx] = joint

        self.fksolverpos.JntToCart(self.joints, self.frame)

        # # make frames for all links
        # for i in range(6):
        #     frame = Frame()
        #     self.fk.JntToCart(self.joints, frame, i + 1)
        #     self.frames.append(frame)

    def get_frame(self):
        """get current tcp frame

        Returns:
            current tcp frame
        """
        return copy.deepcopy(self.frame)
        # return self.frames

    def update_frame(self, frame):
        """update goal frame data

        Args:
            frame: goal frame data
        """
        self.iksolverpos.CartToJnt(self.joints, frame, self.goal_joints)

    def get_joint(self):
        """get goal joint angles

        Returns:
            goal joint angles
        """
        return self.goal_joints

    def get_distance(self, v1, v2):
        """calc distance v1 to v2

        Args:
            v1: vector 1 (start point)
            v2: vector 2 (end point)
        """
        return math.sqrt(
            math.pow((v1.x - v2.x), 2) +
            math.pow((v1.y - v2.y), 2) +
            math.pow((v1.z - v2.z), 2)
        )

    def get_pick_pose(self, pick_frame, _x=0.0, _y=0.0, _z=0.0):
        """adjust pick pose to real object

        Args:
            pick_frame: object frame
            _x: object x-axis offset
            _y: object y-axis offset
            _z: object z-axis offset
        
        Returns:
            prepick pose and postpick pose (2 frames)
        """
        # pre_pick -> object z-axis offset
        # post_pick -> world z-axis offset
        pre_pick = pick_frame * Frame(
            Vector(
                0 + _x,
                0 + _y,
                0 + _z))
        post_pick = copy.deepcopy(pick_frame)
        post_pick.p = Vector(
                post_pick.p[0],
                post_pick.p[1],
                post_pick.p[2] + 0.5
            )
        return copy.deepcopy(pre_pick), copy.deepcopy(post_pick)

    def pose_to_frame(self, _pose):
        """transform Pose() to Frame()
        
        Returns:
            transformed frame data
        """
        _frame = Frame(
            Rotation.Quaternion(
                _pose.orientation.x,
                _pose.orientation.y,
                _pose.orientation.z,
                _pose.orientation.w
            ),
            Vector(
                _pose.position.x,
                _pose.position.y,
                _pose.position.z,
            )
        )
        return copy.deepcopy(_frame)
    
    def frame_to_pose(self, _frame):
        """transform Frame() to Pose()
        
        Returns:
            transformed pose data
        """
        _pose = Pose()
        trans = _frame.p
        rot = Rotation.GetQuaternion(_frame.M)
        _pose.position.x = trans[0]
        _pose.position.y = trans[1]
        _pose.position.z = trans[2]
        _pose.orientation.x = rot[0]
        _pose.orientation.y = rot[1]
        _pose.orientation.z = rot[2]
        _pose.orientation.w = rot[3]
        return copy.deepcopy(_pose)

    def get_frame_XYZ(self, frame, _x=0.0, _y=0.0, _z=0.0):
        """vector move from frame data

        Args:
            frame: frame that want to move
            _x, _y, _z: each axis offset (unit: m)

        Returns:
            moved frame data
        """
        ##########   world coordinate    ############
        _frame = Frame(
            frame.M,
            Vector(
                frame.p[0] + _x,
                frame.p[1] + _y,
                frame.p[2] + _z
            )
        )
        return copy.deepcopy(_frame)
  
    def get_frame_Rxyz(self, frame, rx=0.0, ry=0.0, rz=0.0):
        """rotation from frame data

        Args:
            frame: frame that want to rotate
            rx, ry, rz: each axis offset (unit: rad)

        Returns:
            rotated frame data
        """
        ##########   World coordinate    ############
        rot = Rotation.GetRPY(frame.M)
        _frame = Frame(
            Rotation.RPY(
                rot[0] + rx,
                rot[1] + ry,
                rot[2] + rz
            ),
            frame.p
        )
        return copy.deepcopy(_frame)

    def get_frame_XYZ_tcp(self, frame, _x=0.0, _y=0.0, _z=0.0):
        """vector move from frame data in tcp coordinate

        Args:
            frame: frame that want to move
            _x, _y, _z: each axis offset (unit: m)

        Returns:
            moved frame data
        """
        ###########   World coordinate    ############
        _frame = frame * Frame(Vector(_x, _y, _z))
        return copy.deepcopy(_frame)
  
    def get_frame_Rxyz_tcp(self, frame, rx=0.0, ry=0.0, rz=0.0):
        """rotation from frame data in tcp coordinate

        Args:
            frame: frame that want to rotate
            rx, ry, rz: each axis offset (unit: rad)

        Returns:
            rotated frame data
        """
        ##########   User coordinate    ############
        _frame = frame * Frame(Rotation.RPY(rx, ry, rz))
        return copy.deepcopy(_frame)
