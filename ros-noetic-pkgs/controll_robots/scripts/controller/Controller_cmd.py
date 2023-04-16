#!/usr/bin/env python3

import sys
import rospy

from std_msgs.msg import String


def Cmd_talker():
    """Publish User command to TaskController"""
    
    # pub_rb10 = rospy.Publisher("/cmd_msg_rb10", String, queue_size=1)
    # pub_indy7 = rospy.Publisher("/cmd_msg_indy7", String, queue_size=1)
    pub_cmd = rospy.Publisher("cmd_msg", String, queue_size=1)
    rospy.init_node("cmd_talker", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        cmd_msg = input("enter your command (terminate: q) : ")
        # rospy.loginfo(cmd_msg)
        # pub_rb10.publish(cmd_msg)
        # pub_indy7.publish(cmd_msg)
        pub_cmd.publish(cmd_msg)
        rate.sleep()

        if cmd_msg == "q":
            # sys.exit()
            break
        # elif cmd_msg == "s":
        #     for i in range(100):
        #         pub_cmd.publish(cmd_msg)


if __name__ == "__main__":
    try:
        Cmd_talker()
    except rospy.ROSInterruptException:
        pass
