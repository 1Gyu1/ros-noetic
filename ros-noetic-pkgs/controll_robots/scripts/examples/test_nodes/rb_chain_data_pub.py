#!/usr/bin/env python3

import sys
import rospy

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Bool

CHECK = 0
link = PoseArray()


def node_quit(data):
    global CHECK
    if data.data == True:
        CHECK = 1


def append_pose(axis, data):

    link_temp = Pose()

    if axis == "x":
        link_temp.position.x = data

    elif axis == "y":
        link_temp.position.y = data

    elif axis == "z":
        link_temp.position.z = data

    link_temp.orientation.w = 1

    link.poses.append(link_temp)


def Chain_data_pub():
    """
    Publish topics that are user command

    Args:
        cmd_msg : user input by terminal


    """

    rospy.init_node("chain_data_publisher", anonymous=True)
    global CHECK
    pub = rospy.Publisher("/chain_data", PoseArray, queue_size=1)
    sub = rospy.Subscriber("/get_data", Bool, node_quit)
    rate = rospy.Rate(10)  # 10hz
    pose = "x"

    # /base to /shoulder
    pose = "z"
    append_pose(pose, 0.197)

    # /shoulder to /elbow
    append_pose(pose, 0.6127)

    # /elbow to /wrist1
    append_pose(pose, 0.57015)

    # /wrist1 to /wrist2
    pose = "y"
    append_pose(pose, -0.15625)

    # /wrist2 to /wrist3
    pose = "z"
    append_pose(pose, 0.11715)

    # / wrist3 to /Tool_Center_Point
    pose = "y"
    append_pose(pose, -0.1153)

    while not rospy.is_shutdown():
        pub.publish(link)
        rate.sleep()

        if CHECK == 1:
            # sys.exit()
            break


if __name__ == "__main__":
    try:
        Chain_data_pub()
    except rospy.ROSInterruptException:
        pass
