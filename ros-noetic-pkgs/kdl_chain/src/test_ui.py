#!/usr/bin/env python
"""
pyqt
ui : joint values
publising joints

"""
import sys
import rospy
from kdl_chain.msg import JointArray
from std_msgs.msg import Float32

from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
from PyQt5.uic import loadUi

from ui_controller import Ui_Dialog

class Dialog(QDialog, Ui_Dialog):
    def __init__(self, parent=None):
      super(QDialog, self).__init__()      
      # loadUi("../kdl_chain/ui/controller.ui", self)
      loadUi("/home/gyu/ros_devel/catkin_ws/src/pickit-korea/kdl_chain/ui/controller.ui", self)
      self.ui = Ui_Dialog()
      
      self.joints = [0., 0., 0., 0., 0., 0.]
      self.pub = rospy.Publisher('robot_joints', JointArray, queue_size=10, latch=True)
      rospy.init_node('ui', anonymous=True)
      
      # signal & slot
      self.slide_joint_1.valueChanged.connect(self.slide_joint_1_changed)
      self.slide_joint_2.valueChanged.connect(self.slide_joint_2_changed)
      self.slide_joint_3.valueChanged.connect(self.slide_joint_3_changed)
      self.slide_joint_4.valueChanged.connect(self.slide_joint_4_changed)
      self.slide_joint_5.valueChanged.connect(self.slide_joint_5_changed)
      self.slide_joint_6.valueChanged.connect(self.slide_joint_6_changed)
      
    def slide_joint_1_changed(self, value):
      self.value_joint_1.setValue(value)
      self.joints[0] = value
      self.pub_joints()
    
    def slide_joint_2_changed(self, value):
      self.value_joint_2.setValue(value)
      self.joints[1] = value
      self.pub_joints()
      
    def slide_joint_3_changed(self, value):
      self.value_joint_3.setValue(value)
      self.joints[2] = value
      self.pub_joints()
    
    def slide_joint_4_changed(self, value):
      self.value_joint_4.setValue(value)
      self.joints[3] = value
      self.pub_joints()
      
    def slide_joint_5_changed(self, value):
      self.value_joint_5.setValue(value)
      self.joints[4] = value
      self.pub_joints()
      
    def slide_joint_6_changed(self, value):
      self.value_joint_6.setValue(value)
      self.joints[5] = value
      self.pub_joints()
      
    def pub_joints(self):
      msg = JointArray()
      msg.joints = [Float32(self.joints[0]),
                    Float32(self.joints[1]),
                    Float32(self.joints[2]),
                    Float32(self.joints[3]),
                    Float32(self.joints[4]),
                    Float32(self.joints[5])] 
      self.pub.publish(msg)
      

if __name__ == "__main__" :

    app = QApplication(sys.argv)
    
    dialog = Dialog()
    dialog.show()
    
    sys.exit(app.exec_())