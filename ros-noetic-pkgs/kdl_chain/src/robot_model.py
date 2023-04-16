#!/usr/bin/env python

from PyKDL import Chain, Segment, Joint, Frame, Vector, ChainFkSolverPos_recursive, JntArray, Rotation

class Robot:
  """
  Robot class 
  """
  
  def __init__(self):
    self.chain = Chain()
    self.segments = [Segment(Joint(Joint.RotZ), 
                             Frame(Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
                             Vector(0.0, -0.109, 0.222) )),
                     Segment(Joint(Joint.RotZ), 
                             Frame(Rotation.Quaternion(0, 0, 0, 1),
                             Vector(-0.45, 0.0, -0.0305))),
                     Segment(Joint(Joint.RotZ), 
                             Frame(Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
                             Vector(-0.267, 0.0, -0.075))),
                     Segment(Joint(Joint.RotZ), 
                             Frame(Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
                             Vector(0.0, -0.114, 0.083))),
                     Segment(Joint(Joint.RotZ), 
                             Frame(Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
                             Vector(-0.168, 0.0, 0.069))),
                     Segment(Joint(Joint.RotZ), 
                             Frame(Rotation.Quaternion(0, 0, 0, 1),
                             Vector(0.0, 0.0, 0.06)))]
    
    # self.chain.addSegment(Segment(Joint(Joint.RotZ),
    #                                   Frame(Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
    #                                         Vector(0.0, -0.109, 0.222))))
    # self.chain.addSegment(Segment(Joint(Joint.RotZ),
    #                                   Frame(Rotation.Quaternion(0, 0, 0, 1),
    #                                         Vector(-0.45, 0.0, -0.0305))))
    # self.chain.addSegment(Segment(Joint(Joint.RotZ),
    #                                   Frame(Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
    #                                         Vector(-0.267, 0.0, -0.075))))
    # self.chain.addSegment(Segment(Joint(Joint.RotZ),
    #                                   Frame(Rotation.Quaternion(0.5, 0.5, -0.5, 0.5),
    #                                         Vector(0.0, -0.114, 0.083))))
    # self.chain.addSegment(Segment(Joint(Joint.RotZ),
    #                                   Frame(Rotation.Quaternion(-0.5, -0.5, 0.5, 0.5),
    #                                         Vector(-0.168, 0.0, 0.069))))
    # self.chain.addSegment(Segment(Joint(Joint.RotZ),
    #                                   Frame(Rotation.Quaternion(0, 0, 0, 1),
    #                                         Vector(0.0, 0.0, 0.06))))   

    for segment in self.segments:      
      self.chain.addSegment(segment)
      
    self.fk = ChainFkSolverPos_recursive(self.chain)
    self.joints = JntArray(6)
    
  def update_joint(self, joints):
    for idx, joint in enumerate(joints):
      self.joints[idx] = joint
    
    # make frames for all links
    self.frames = []
    for i in range(6):      
      frame = Frame()
      self.fk.JntToCart(self.joints, frame, i+1)
      self.frames.append(frame)
    
  def get_frames(self):
    return self.frames
  