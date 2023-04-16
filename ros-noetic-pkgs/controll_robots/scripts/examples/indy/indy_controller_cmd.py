#!/usr/bin/env python3

import sys
import rospy

from std_msgs.msg import String


def Cmd_talker():
    """
    Publish topics that are user command

    Args:
        cmd_msg : user input by terminal


    """
    pub = rospy.Publisher("/cmd_msg", String, queue_size=1)
    rospy.init_node("cmd_talker", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        cmd_msg = input("enter point(1~16). robot stop(s). : ")
        # rospy.loginfo(cmd_msg)
        pub.publish(cmd_msg)
        rate.sleep()

        if cmd_msg == "q":
            # sys.exit()
            break


if __name__ == "__main__":
    try:
        Cmd_talker()
    except rospy.ROSInterruptException:
        pass
