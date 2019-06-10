#! /usr/bin/env python

import rospy
import math
import time
from scipy.optimize import *
from numpy import *
from sensor_msgs.msg import BatteryState, Temperature
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('dist_travelled')

dist_pub = rospy.Publisher('dist_travelled', PoseStamped, queue_size=1)
rate = rospy.Rate(50)




def callback_gps(gps):
    time_now = rospy.get_rostime()
    if time_now%5 ==0:






    dist_pub.publish(distribution)




def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)

    rospy.spin()


if __name__ == '__main__':
    main()
