#!/usr/bin/env python
"""
Building a KDL chain
Listening a joint values and updating the chain 

[Mode 1]
Input : joint values
Process : forward kinematics 
Output : publishing each link frames 

[Mode 2]
Input : target frame of the last link
Process : inverse kinematics, MoveJ or MoveL 
Output : 
"""

# from tkinter import Frame
import rospy
from math import radians
from robot_model import Robot
from PyKDL import Frame ,Rotation, Vector
from kdl_chain.msg import JointArray
from geometry_msgs.msg import Pose, PoseArray
class Controller :
  def __init__(self):
    self.robot_model = Robot()
    
    self.link_pub = rospy.Publisher('/robot_link', PoseArray, queue_size=1) 
    rospy.Subscriber('robot_joints', JointArray, self.cb_joints)
    rospy.init_node('robot_controller')
    rospy.spin()      
    
  def get_PoseArray(self, link, frames) :
    tmp_pose = Pose()
    tmp_trans = frames.p 
    tmp_rot = Rotation.GetQuaternion(frames.M)

    tmp_pose.position.x = float(tmp_trans[0])
    tmp_pose.position.y = float(tmp_trans[1])
    tmp_pose.position.z = float(tmp_trans[2])
    tmp_pose.orientation.x = float(tmp_rot[0]) 
    tmp_pose.orientation.y = float(tmp_rot[1])
    tmp_pose.orientation.z = float(tmp_rot[2])
    tmp_pose.orientation.w = float(tmp_rot[3])

    link.poses.append(tmp_pose)


  def cb_joints(self, data):
    link = PoseArray()
    tmp_frame = Frame()
    self.joints_rad = [radians(data.joints[0].data),
                       radians(data.joints[1].data),
                       radians(data.joints[2].data),
                       radians(data.joints[3].data),
                       radians(data.joints[4].data),
                       radians(data.joints[5].data)]  
              
    self.robot_model.update_joint(self.joints_rad)    
    frames = self.robot_model.get_frames()
    # print(frames)
    
    for i in range(6) :
      self.get_PoseArray(link, frames[i])
      # print(frames[i].p)

    self.link_pub.publish(link) 

if __name__ == "__main__":
  Controller()