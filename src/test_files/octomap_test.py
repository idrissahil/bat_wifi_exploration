#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from octomap_msgs.msg import Octomap

rospy.init_node('octomap_test')

battery_pub = rospy.Publisher('octomap_free', PoseStamped, queue_size=1)
rate = rospy.Rate(50)

def callback_octomap(octomap):


def main():
    gps_sub = rospy.Subscriber('/aeplanner/octomap', Octomap , callback_octomap)
    rospy.spin()


if _name_ == '_main_':
    main()