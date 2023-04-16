#!/usr/bin/env python3

import os
import sys
from time import sleep

from matplotlib.pyplot import connect
import rospy
import math
from kdl_chain.msg import JointArray
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseArray, Pose, Vector3
from controll_robots.msg import robots_info

from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
from PyQt5.uic import loadUi

from controller_ui import Ui_MainWindow

currentPath = os.getcwd()
os.chdir(currentPath + "/catkin_ws/src/pickit-korea/controll_robots/scripts/controller")
# os.chdir(currentPath + "/ros_devel/catkin_ws/src/pickit-korea/controll_robots/scripts/controller")
class Dialog(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(QMainWindow, self).__init__()
        print(currentPath)  
        loadUi("test_UI_demo.ui", self)
        self.ui = Ui_MainWindow()
        
        # self.data = robots_info()
        self.object = Pose()

        self.indy7_status = self.indy_check = 5
        self.rb10_status = self.rb_check = 5
        self.coordinate = "world"

        self.idx = 0

        # Publishers
        self.pub_test_object_indy = rospy.Publisher("/ui/indy7/object_pose", Pose, queue_size=1)
        self.pub_test_object_rb = rospy.Publisher("/ui/rb10/object_pose", Pose, queue_size=1)
        self.pub_cmd_rb10 = rospy.Publisher("/rb10/cmd_msg", String, queue_size=1)
        self.pub_cmd_indy7 = rospy.Publisher("/indy7/cmd_msg", String, queue_size=1)
        self.pub_cmd_UI_rb10 = rospy.Publisher("/ui/rb10/cmd_msg", String, queue_size=1)
        self.pub_cmd_UI_indy7 = rospy.Publisher("/ui/indy7/cmd_msg", String, queue_size=1)
        self.tcp_offset_pub_rb10 = rospy.Publisher("/rb10/tcp_offset", Vector3, queue_size=1)
        self.tcp_offset_pub_indy7 = rospy.Publisher("/indy7/tcp_offset", Vector3, queue_size=1)
        self.task_control_unit_pub_rb10 = rospy.Publisher("/rb10/task_unit", Float32, queue_size=1)
        self.task_control_unit_pub_indy7 = rospy.Publisher("/indy7/task_unit", Float32, queue_size=1)

        self.pub_cmd = self.pub_cmd_indy7
        self.tcp_offset_pub = self.tcp_offset_pub_indy7
        self.pub_cmd_UI = self.pub_cmd_UI_indy7
        self.task_control_unit_pub = self.task_control_unit_pub_indy7


        # self.tcp_offset_pub = rospy.Publisher("/tcp_offset", Vector3, queue_size=1)

    

        # Subscribers
        rospy.Subscriber("/rb10/r_info", robots_info, self.robot_data_cb_rb)
        rospy.Subscriber("/indy7/r_info", robots_info, self.robot_data_cb_indy)

        # signal & slot
        self.POINT_SAVE_BTN.clicked.connect(self.save_btn_clicked)
        self.POINT_DEL_BTN.clicked.connect(self.del_btn_clicked)
        self.POINT_PLAY_BTN.clicked.connect(self.play_btn_clicked)
        self.BTN_MODE_INDY7.clicked.connect(self.mode_indy7)
        self.BTN_MODE_RB10.clicked.connect(self.mode_rb10)
        self.BTN_SEND_CMD.clicked.connect(self.send_cmd_msg)
        self.BTN_MOTION_HALT.clicked.connect(self.send_stop_sig)
        self.TEST_POINT_INDY1.clicked.connect(self.test_point_indy1)
        self.TEST_POINT_INDY2.clicked.connect(self.test_point_indy2)
        self.TEST_POINT_INDY3.clicked.connect(self.test_point_indy3)
        self.TEST_POINT_RB1.clicked.connect(self.test_point_rb1)
        self.TEST_POINT_RB2.clicked.connect(self.test_point_rb2)
        self.TEST_POINT_RB3.clicked.connect(self.test_point_rb3)
        self.VECTOR_X_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("ww"))
        self.VECTOR_X_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("xx"))
        self.VECTOR_Y_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("aa"))
        self.VECTOR_Y_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("dd"))
        self.VECTOR_Z_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("rr"))
        self.VECTOR_Z_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("ff"))
        self.ROTATION_RX_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("yy"))
        self.ROTATION_RX_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("uu"))
        self.ROTATION_RY_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("hh"))
        self.ROTATION_RY_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("jj"))
        self.ROTATION_RZ_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("nn"))
        self.ROTATION_RZ_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn_UI("mm"))
        self.TCP_X_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"w"))
        self.TCP_X_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"x"))
        self.TCP_Y_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"a"))
        self.TCP_Y_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"d"))
        self.TCP_Z_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"r"))
        self.TCP_Z_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"f"))
        self.TCP_RX_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"y"))
        self.TCP_RX_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"u"))
        self.TCP_RY_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"g"))
        self.TCP_RY_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"h"))
        self.TCP_RZ_PLUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"b"))
        self.TCP_RZ_MINUS.clicked.connect(lambda:self.send_cmd_msg_btn(self.coordinate+"n"))
        self.HOME_BTN.clicked.connect(lambda:self.send_cmd_msg_btn("1"))
        self.ZERO_BTN.clicked.connect(lambda:self.send_cmd_msg_btn("0"))
        self.OBJ_PUB_BTN.clicked.connect(lambda:self.send_cmd_msg_btn_UI("pub_obj"))
        self.TCP_OFFSET_PUB_BTN.clicked.connect(self.send_tcp_offset)
        self.PICK_N_PLACE_DEMO_BTN.clicked.connect(lambda:self.send_cmd_msg_btn("pick"))
        self.GET_TASK_UNIT_BTN.clicked.connect(self.get_task_unit)
        self.COORDINATE_WORLD_BTN.clicked.connect(lambda:self.coordinate_system_change("world"))
        self.COORDINATE_TCP_BTN.clicked.connect(lambda:self.coordinate_system_change("tcp"))

        self.tcp_offset_x.setText("0.0")
        self.tcp_offset_y.setText("0.0")
        self.tcp_offset_z.setText("0.0")

    
    def status_indicator(self):
        # while True:
            if self.indy_check != self.indy7_status:
                self.indy_check = self.indy7_status
                self.robot_status_check_indy(self.indy_check)
            if self.rb_check != self.rb10_status:
                self.rb_check = self.rb10_status
                self.robot_status_check_rb(self.rb_check)
            rospy.sleep(0.1)

        

    # Callback func
    def robot_data_cb_rb(self, rb):
        self.data_update_rb(rb)
        # if self.rb10_status != rb.robot_status:
        #     self.rb10_status = rb.robot_status
        #     self.robot_status_check_rb(self.rb10_status)

    def robot_data_cb_indy(self, indy):
        self.data_update_indy(indy)
        # if self.indy7_status != indy.robot_status:
        #     self.indy7_status = indy.robot_status
        #     self.robot_status_check_indy(self.indy7_status)
        
    # slots
    def mode_indy7(self): 
        # set indy7 box green color
        # set rb10 box white(normal) color
        self.pub_cmd = self.pub_cmd_indy7
        self.tcp_offset_pub = self.tcp_offset_pub_indy7
        self.pub_cmd_UI = self.pub_cmd_UI_indy7
        self.task_control_unit_pub = self.task_control_unit_pub_indy7
        self.BTN_MODE_INDY7.setStyleSheet("background-color: rgb(77, 237, 48);")
        self.BTN_MODE_RB10.setStyleSheet("background-color: white")
        self.label_31.setText("INDY7")

    def mode_rb10(self):
        # set rb10 box green color
        # set indy7 box white(normal) color
        self.pub_cmd = self.pub_cmd_rb10
        self.tcp_offset_pub = self.tcp_offset_pub_rb10
        self.pub_cmd_UI = self.pub_cmd_UI_rb10
        self.task_control_unit_pub = self.task_control_unit_pub_rb10
        self.BTN_MODE_RB10.setStyleSheet("background-color: rgb(77, 237, 48);")
        self.BTN_MODE_INDY7.setStyleSheet("background-color: white")
        self.label_31.setText("RB10")

    def coordinate_system_change(self, coordi):
        self.coordinate = coordi
        if self.coordinate == "world":
            self.COORDINATE_WORLD_BTN.setStyleSheet("background-color: rgb(77, 237, 48);")
            self.COORDINATE_TCP_BTN.setStyleSheet("background-color: white")
        elif self.coordinate == "tcp":
            self.COORDINATE_WORLD_BTN.setStyleSheet("background-color: white")
            self.COORDINATE_TCP_BTN.setStyleSheet("background-color: rgb(77, 237, 48);")

    # def task_move_slot(self, data):
    #     msg = self.coordinate + data
    #     print(msg)
    #     self.send_cmd_msg_btn_UI(msg)

    def save_btn_clicked(self): 
        cmd = "way" + str(self.idx) + "j"
        self.idx += 1
        self.pub_cmd.publish(cmd)

    def del_btn_clicked(self):
        if self.idx < 1 :
            self.idx = 0
        else:
            self.idx -= 1
        cmd = "del" + str(self.idx)
        self.pub_cmd.publish(cmd)

    def play_btn_clicked(self):
        cmd = "go"
        self.pub_cmd.publish(cmd)

    def send_cmd_msg(self):
        cmd = self.cmd_msg_box.text()
        self.cmd_msg_box.setText("")
        self.pub_cmd.publish(cmd)
    
    def send_tcp_offset(self):
        offset = Vector3()
        offset.x = float(self.tcp_offset_x.text())
        offset.y = float(self.tcp_offset_y.text())
        offset.z = float(self.tcp_offset_z.text())
        # self.cmd_msg_box.setText("")
        self.tcp_offset_pub.publish(offset)
    
    def send_cmd_msg_btn(self, msg):
        cmd = msg
        self.pub_cmd.publish(cmd)

    def send_cmd_msg_btn_UI(self, msg):
        cmd = msg
        self.pub_cmd_UI.publish(cmd)

    def send_stop_sig(self):
        cmd = "s"
        self.mode_indy7()
        self.pub_cmd.publish(cmd)
        self.mode_rb10()
        self.pub_cmd.publish(cmd)


    def test_point_indy1(self):
        object = self.list_to_pose(self.get_point(1))
        self.pub_test_object_indy.publish(object)

    def test_point_indy2(self):
        object = self.list_to_pose(self.get_point(2))
        self.pub_test_object_indy.publish(object)

    def test_point_indy3(self):
        object = self.list_to_pose(self.get_point(3))
        self.pub_test_object_indy.publish(object)

    def test_point_rb1(self):
        object = self.list_to_pose(self.get_point(1))
        self.pub_test_object_rb.publish(object)

    def test_point_rb2(self):
        object = self.list_to_pose(self.get_point(2))
        self.pub_test_object_rb.publish(object)

    def test_point_rb3(self):
        object = self.list_to_pose(self.get_point(3))
        self.pub_test_object_rb.publish(object)

    def get_point(self, idx):
        object = dict()
        object = {
            1: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 1.0],
            2: [0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 1.0],
            3: [0.3, 0.3, 0.3, 0.0, 0.0, 0.0, 1.0]
            }
        return object[idx]

    def get_task_unit(self):
        unit = Float32()
        unit.data = float(self.task_control_unit.text())
        self.task_control_unit_pub.publish(unit)
        

    
    def list_to_pose(self, obj_list):
        object = Pose()
        object.position.x = obj_list[0]
        object.position.y = obj_list[1]
        object.position.z = obj_list[2]
        object.orientation.x = obj_list[3]
        object.orientation.y = obj_list[4]
        object.orientation.z = obj_list[5]
        object.orientation.w = obj_list[6]
        return object

    def data_update_rb(self, data):
        """UI data print rb
        """
        # print("data_update")
        n = 1
        nn = 3
        # RB10 joint angles
        # rb_j1 = int(math.degrees(data.joint[0]) * (10**n)) / (10**n)
        # rb_j2 = int(math.degrees(data.joint[1]) * (10**n)) / (10**n)
        # rb_j3 = int(math.degrees(data.joint[2]) * (10**n)) / (10**n)
        # rb_j4 = int(math.degrees(data.joint[3]) * (10**n)) / (10**n)
        # rb_j5 = int(math.degrees(data.joint[4]) * (10**n)) / (10**n)
        # rb_j6 = int(math.degrees(data.joint[5]) * (10**n)) / (10**n)
        rb_j1 = round(math.degrees(data.joint[0]), n)
        rb_j2 = round(math.degrees(data.joint[1]), n)
        rb_j3 = round(math.degrees(data.joint[2]), n)
        rb_j4 = round(math.degrees(data.joint[3]), n)
        rb_j5 = round(math.degrees(data.joint[4]), n)
        rb_j6 = round(math.degrees(data.joint[5]), n)

        self.RB_JNT_1.setText(str(rb_j1))
        self.RB_JNT_2.setText(str(rb_j2))
        self.RB_JNT_3.setText(str(rb_j3))
        self.RB_JNT_4.setText(str(rb_j4))
        self.RB_JNT_5.setText(str(rb_j5))
        self.RB_JNT_6.setText(str(rb_j6))

        # RB10 tcp 
        # rb_tcp1 = int(data.tcp[0] * (10**nn)) / (10**nn)
        # rb_tcp2 = int(data.tcp[1] * (10**nn)) / (10**nn)
        # rb_tcp3 = int(data.tcp[2] * (10**nn)) / (10**nn)
        # rb_tcp4 = int(data.tcp[3] * (10**nn)) / (10**nn)
        # rb_tcp5 = int(data.tcp[4] * (10**nn)) / (10**nn)
        # rb_tcp6 = int(data.tcp[5] * (10**nn)) / (10**nn)
        rb_tcp1 = round(data.tcp[0], nn)
        rb_tcp2 = round(data.tcp[1], nn)
        rb_tcp3 = round(data.tcp[2], nn)
        rb_tcp4 = round(data.tcp[3], nn)
        rb_tcp5 = round(data.tcp[4], nn)
        rb_tcp6 = round(data.tcp[5], nn)

        self.RB_TCP_X.setText(str(rb_tcp1))
        self.RB_TCP_Y.setText(str(rb_tcp2))
        self.RB_TCP_Z.setText(str(rb_tcp3))
        self.RB_TCP_RX.setText(str(rb_tcp4))
        self.RB_TCP_RY.setText(str(rb_tcp5))
        self.RB_TCP_RZ.setText(str(rb_tcp6))


    def data_update_indy(self, data):
        """UI data print indy
        """
        # print("data_update")
        n = 1
        nn = 3

        # Indy7 joint angles
        # indy_j1 = int(math.degrees(data.joint[0]) * (10**n)) / (10**n)
        # indy_j2 = int(math.degrees(data.joint[1]) * (10**n)) / (10**n)
        # indy_j3 = int(math.degrees(data.joint[2]) * (10**n)) / (10**n)
        # indy_j4 = int(math.degrees(data.joint[3]) * (10**n)) / (10**n)
        # indy_j5 = int(math.degrees(data.joint[4]) * (10**n)) / (10**n)
        # indy_j6 = int(math.degrees(data.joint[5]) * (10**n)) / (10**n)
        indy_j1 = round(math.degrees(data.joint[0]),  n)
        indy_j2 = round(math.degrees(data.joint[1]),  n)
        indy_j3 = round(math.degrees(data.joint[2]),  n)
        indy_j4 = round(math.degrees(data.joint[3]),  n)
        indy_j5 = round(math.degrees(data.joint[4]),  n)
        indy_j6 = round(math.degrees(data.joint[5]),  n)

        self.INDY_JNT_1.setText(str(indy_j1))
        self.INDY_JNT_2.setText(str(indy_j2))
        self.INDY_JNT_3.setText(str(indy_j3))
        self.INDY_JNT_4.setText(str(indy_j4))
        self.INDY_JNT_5.setText(str(indy_j5))
        self.INDY_JNT_6.setText(str(indy_j6))

        # Indy7 tcp
        # indy_tcp1 = int(data.tcp[0] * (10**nn)) / (10**nn)
        # indy_tcp2 = int(data.tcp[1] * (10**nn)) / (10**nn)
        # indy_tcp3 = int(data.tcp[2] * (10**nn)) / (10**nn)
        # indy_tcp4 = int(data.tcp[3] * (10**nn)) / (10**nn)
        # indy_tcp5 = int(data.tcp[4] * (10**nn)) / (10**nn)
        # indy_tcp6 = int(data.tcp[5] * (10**nn)) / (10**nn)
        indy_tcp1 = round(data.tcp[0], nn)
        indy_tcp2 = round(data.tcp[1], nn)
        indy_tcp3 = round(data.tcp[2], nn)
        indy_tcp4 = round(data.tcp[3], nn)
        indy_tcp5 = round(data.tcp[4], nn)
        indy_tcp6 = round(data.tcp[5], nn)

        self.INDY_TCP_X.setText(str(indy_tcp1))
        self.INDY_TCP_Y.setText(str(indy_tcp2))
        self.INDY_TCP_Z.setText(str(indy_tcp3))
        self.INDY_TCP_RX.setText(str(indy_tcp4))
        self.INDY_TCP_RY.setText(str(indy_tcp5))
        self.INDY_TCP_RZ.setText(str(indy_tcp6))

    def robot_status_check_rb(self, data):        
        if data == 1:
            # idle box set green color
            # moving box set white(normal) color
            self.RB_ROBOT_STATE_IDLE.setStyleSheet("background-color: rgb(77, 237, 48)")
            self.RB_ROBOT_STATE_MOVING.setStyleSheet("background-color: white")
        elif data == 3:
            # moving box set green color
            # idle box set white(normal) color
            self.RB_ROBOT_STATE_MOVING.setStyleSheet("background-color: rgb(77, 237, 48)")
            self.RB_ROBOT_STATE_IDLE.setStyleSheet("background-color: white")

    def robot_status_check_indy(self, data):
        if data == 0:
            # idle box set green color
            # moving box set white(normal) color
            self.INDY_ROBOT_STATE_IDLE.setStyleSheet("background-color: rgb(77, 237, 48)")
            self.INDY_ROBOT_STATE_MOVING.setStyleSheet("background-color: white")
        elif data == 1: 
            # moving box set green color
            # idle box set white(normal) color
            self.INDY_ROBOT_STATE_MOVING.setStyleSheet("background-color: rgb(77, 237, 48)")
            self.INDY_ROBOT_STATE_IDLE.setStyleSheet("background-color: white")
        else:
            print("indy status error")


if __name__ == "__main__" :
    rospy.init_node("Test_UI", anonymous=True)

    app = QApplication(sys.argv)
    
    dialog = Dialog()
    dialog.show()
    
    # dialog.status_indicator()
    sys.exit(app.exec_())
