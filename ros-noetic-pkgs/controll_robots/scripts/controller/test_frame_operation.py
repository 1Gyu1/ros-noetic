#!/usr/bin/python3

from copyreg import pickle
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

offset = Vector3()
waypoint_rb = [0 for l in range(13)]
waypoint_indy = [0 for l in range(13)]
waypoint = [0 for l in range(13)]
pick_name_rb = ["r_prepick", "r_pick", "r_postpick", "r_tcp", "r_flange"]
pick_name_indy = ["i_prepick", "i_pick", "i_postpick", "i_tcp", "i_flange"]
chain = Chain()
Robot = Robot(chain)


def broadcast_tf_rb_object():
    global waypoint_rb
    idx = 3
    if waypoint_rb[idx] != 0:
        pose = Pose()
        trans = waypoint_rb[idx].p
        rot = Rotation.GetQuaternion(waypoint_rb[idx].M)
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        br = tf.TransformBroadcaster()
        br.sendTransform(
            (
                pose.position.x,
                pose.position.y, 
                pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y, 
                pose.orientation.z,
                pose.orientation.w), 
            rospy.Time.now(), "r_object", 'Axis_0') 
      
def broadcast_tf_indy_object():
    global waypoint_indy
    idx = 3
    if waypoint_indy[idx] != 0:
        pose = Pose()
        trans = waypoint_indy[idx].p
        rot = Rotation.GetQuaternion(waypoint_indy[idx].M)
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        br = tf.TransformBroadcaster()
        br.sendTransform(
            (
                pose.position.x,
                pose.position.y, 
                pose.position.z + 0.177),
            (
                pose.orientation.x,
                pose.orientation.y, 
                pose.orientation.z,
                pose.orientation.w), 
            rospy.Time.now(), "i_object", 'world') 


def broadcast_tf_rb():
    global waypoint_rb
    
    for idx in range(len(waypoint_rb)):
        if waypoint_rb[idx] != 0:
            pose = Pose()
            trans = waypoint_rb[idx].p
            rot = Rotation.GetQuaternion(waypoint_rb[idx].M)
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]

            br = tf.TransformBroadcaster()
            if (idx > 1) and (idx < 7) :
                br.sendTransform(
                    (
                        pose.position.x,
                        pose.position.y, 
                        pose.position.z),
                    (
                        pose.orientation.x,
                        pose.orientation.y, 
                        pose.orientation.z,
                        pose.orientation.w), 
                    rospy.Time.now(), pick_name_rb[idx-2], 'Axis_0') 
            else:
                br.sendTransform(
                (
                    pose.position.x,
                    pose.position.y, 
                    pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y, 
                    pose.orientation.z,
                    pose.orientation.w), 
                rospy.Time.now(), "R_" + str(idx), 'Axis_0') 

def broadcast_tf_indy():
    global waypoint_indy
    
    for idx in range(len(waypoint_indy)):
        if waypoint_indy[idx] != 0:
            pose = Pose()
            trans = waypoint_indy[idx].p
            rot = Rotation.GetQuaternion(waypoint_indy[idx].M)
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]

            br = tf.TransformBroadcaster()
            if (idx > 1) and (idx < 7) :
                br.sendTransform(
                    (
                        pose.position.x,
                        pose.position.y, 
                        pose.position.z),
                    (
                        pose.orientation.x,
                        pose.orientation.y, 
                        pose.orientation.z,
                        pose.orientation.w), 
                    rospy.Time.now(), pick_name_indy[idx-2], 'world') 
            else:
                br.sendTransform(
                (
                    pose.position.x,
                    pose.position.y, 
                    pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y, 
                    pose.orientation.z,
                    pose.orientation.w), 
                rospy.Time.now(), "R_" + str(idx), 'world') 

def pose_to_frame(_pose):
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
    return _frame

def convert_pick_pose(pick_frame, waypoints, pick_offset=0.0, pre_offset=0.1, post_offset=0.1):
    """adjust pick pose to real object
    """
    # global pick_frame
    # pre_pick -> object z-axis offset
    # post_pick -> world z-axis offset

    pick_pose = pick_frame * Frame(Vector(0, 0, pick_offset))

    pre_pick = pick_pose * Frame(Vector(0, 0, pre_offset))
    post_pick = copy.deepcopy(pick_pose)
    post_pick.p = Vector(
            post_pick.p[0],
            post_pick.p[1],
            post_pick.p[2] + post_offset
        )
    waypoints[3] = pick_pose
    waypoints[2] = pre_pick
    waypoints[4] = post_pick

    return waypoints

def convert_tcp(pick_frame, waypoints):
    """add tool and
    """
    # global pick_frame
    tool_flange = pick_frame

    #####    RB10    #####
    # tcp = tool_flange * Frame(
    #     Rotation.RPY(1.57079, -1.57079, 0),
    #     Vector(offset.x, offset.y, offset.z))

    #####    indy7    #####
    tcp = tool_flange * Frame(
        Rotation.RPY(0, 0, 3.141592),
        Vector(offset.x, offset.y, offset.z))

    waypoints[5] = tcp
    waypoints[6] = reverse_tcp(tcp)

    return waypoints

def reverse_tcp(_frame):
    """get tool flange frame

    Returns:
        flange: reverse tcp frame with tcp offset
    """
    #####    RB10    #####
    # flange = _frame * Frame(
    #     Rotation.RPY(-1.57079, 0, -1.57079),
    #     Vector(
    #         -1.0 * offset.z,
    #         offset.x,
    #         offset.y))

    #####    indy7    #####
    flange = _frame * Frame(
            Rotation.RPY(0, 0, 3.141592),
            Vector(
                offset.x,
                offset.y,
                -1.0 * offset.z))
    return flange

def touch_object(cmd, object):
    if cmd.data == 'ww':
        object = Robot.get_frame_XYZ(object, _x=0.02)
    elif cmd.data == 'xx':
        object = Robot.get_frame_XYZ(object, _x=-0.02)
    elif cmd.data == 'aa':
        object = Robot.get_frame_XYZ(object, _y=0.02)
    elif cmd.data == 'dd':
        object = Robot.get_frame_XYZ(object, _y=-0.02)
    elif cmd.data == 'rr':
        object = Robot.get_frame_XYZ(object, _z=0.02)
    elif cmd.data == 'ff':
        object = Robot.get_frame_XYZ(object, _z=-0.02)
    
    elif cmd.data == 'yy':
        object = Robot.get_frame_Rxyz(object, rx=0.02)
    elif cmd.data == 'uu':
        object = Robot.get_frame_Rxyz(object, rx=-0.02)
    elif cmd.data == 'hh':
        object = Robot.get_frame_Rxyz(object, ry=0.02)
    elif cmd.data == 'jj':
        object = Robot.get_frame_Rxyz(object, ry=-0.02)
    elif cmd.data == 'nn':
        object = Robot.get_frame_Rxyz(object, rz=0.02)
    elif cmd.data == 'mm':
        object = Robot.get_frame_Rxyz(object, rz=-0.02)
    else:
        print("MESSAGE ERROR!!")
        return 0

    return object


def get_object_cb_rb(object):
    global pick_frame_rb, waypoint_rb
    print("get object")
    pick_frame_rb = pose_to_frame(object)  
    waypoint_rb =  convert_pick_pose(pick_frame_rb, waypoint_rb)

def get_object_cb_indy(object):
    global pick_frame_indy, waypoint_indy
    print("get object")
    pick_frame_indy = pose_to_frame(object)  
    waypoint_indy = convert_pick_pose(pick_frame_indy, waypoint_indy)

def get_cmd_msg_rb(cmd):
    global pick_frame_rb, waypoint_rb, obj_pub_rb

    if cmd.data == 'pub_obj':
        obj = Robot.frame_to_pose(pick_frame_rb)
        obj_pub_rb.publish(obj)
    else:
        pick_frame_rb = touch_object(cmd, pick_frame_rb)
    
    waypoint_rb = convert_pick_pose(pick_frame_rb, waypoint_rb)
    waypoint_rb = convert_tcp(pick_frame_rb, waypoint_rb)

def get_cmd_msg_indy(cmd):
    global pick_frame_indy, waypoint_indy, obj_pub_indy

    if cmd.data == 'pub_obj':
        obj = Robot.frame_to_pose(pick_frame_indy)
        obj_pub_indy.publish(obj)
    else:
        pick_frame_indy = touch_object(cmd, pick_frame_indy)
    
    waypoint_indy = convert_pick_pose(pick_frame_indy, waypoint_indy)
    waypoint_indy = convert_tcp(pick_frame_indy, waypoint_indy)


def get_tcp_offset_rb(data):
    global pick_frame_rb, waypoint_rb
    # offset.x = data.x
    # offset.y = data.y
    # offset.z = data.z
    offset.z = data.x
    offset.x = data.y
    offset.y = -1 * data.z
    convert_tcp(pick_frame_rb, waypoint_rb)

def get_tcp_offset_indy(data):
    global pick_frame_indy, waypoint_indy
    # offset.x = data.x
    # offset.y = data.y
    # offset.z = data.z
    offset.x = -1.0 * data.x
    offset.y = data.y
    offset.z = data.z
    convert_tcp(pick_frame_indy, waypoint_indy)


def main():
    global obj_pub_rb, obj_pub_indy
    rospy.init_node("frame_operation_Test")
    rospy.Subscriber("/ui/rb10/object_pose", Pose, get_object_cb_rb)
    rospy.Subscriber("/ui/indy7/object_pose", Pose, get_object_cb_indy)
    # rospy.Subscriber("/ui/cmd_msg", String, get_cmd_msg)
    rospy.Subscriber("/ui/rb10/cmd_msg", String, get_cmd_msg_rb)
    rospy.Subscriber("/ui/indy7/cmd_msg", String, get_cmd_msg_indy)
    # rospy.Subscriber("/rb10/tcp_offset", Vector3, get_tcp_offset_rb)
    # rospy.Subscriber("/indy7/tcp_offset", Vector3, get_tcp_offset_indy)
    obj_pub_rb = rospy.Publisher("/rb10/object_pose", Pose, queue_size=1)
    obj_pub_indy = rospy.Publisher("/indy7/object_pose", Pose, queue_size=1)
  
    while not rospy.is_shutdown():
        # broadcast_tf_rb()
        # broadcast_tf_indy()
        broadcast_tf_rb_object()
        broadcast_tf_indy_object()
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
