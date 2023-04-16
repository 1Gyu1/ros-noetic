#!/usr/bin/env python3

import os
import rospy
import roslaunch, rosnode
import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from std_msgs.msg import Int64MultiArray

currentPath = os.getcwd()
# os.chdir(currentPath + "/catkin_ws/src/pickit-korea/combined_tf/src")
os.chdir(currentPath + "/ros_devel/catkin_ws/src/pickit-korea/combined_tf/src")
form_class = uic.loadUiType("tf_manager_ui.ui")[0]

class WindowClass(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Publishers
        self.rb10_base_pub = rospy.Publisher("/rb_rel_indy", Int64MultiArray, queue_size=1)

        # signal & slot
        self.Start.clicked.connect(self.start)
        self.Stop.clicked.connect(self.stop)
        self.Validate.clicked.connect(self.send_rb10_base)

    # Callback func
    def start(self):
        self.node = roslaunch.core.Node('combined_tf', 'tf_manager.py')
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.script = roslaunch.launch(self.node)
        print(self.script.is_alive())
        self.script.stop()


    def stop(self, tf_manager):
        rosnode.kill_nodes([tf_manager])


    def send_rb10_base(self):
        rb10_base = Int64MultiArray()
        rb10_base.data = [0 for i in range(6)]

        rb10_base.data[0] = float(self.rb10x.text())
        rb10_base.data[1] = float(self.rb10y.text())
        rb10_base.data[2] = float(self.rb10z.text())
        rb10_base.data[3] = float(self.rb10R.text())
        rb10_base.data[4] = float(self.rb10P.text())
        rb10_base.data[5] = float(self.rb10Y.text())

        self.rb10_base_pub.publish(rb10_base)


if __name__ == "__main__":
    rospy.init_node("tf_manager_testUI")
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.exec_()