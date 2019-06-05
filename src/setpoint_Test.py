#! /usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import PositionTarget
import random
import math



rospy.init_node('setpoint_test')

search_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=1)
rate = rospy.Rate(50)

ranges=[]


def callback_lidar(scan):
    global ranges
    global angle_min
    #global increment

    search_points = PoseStamped()
    search_points.pose.position.x = 2
    search_points.pose.position.y = 2
    search_points.pose.position.z = 10
    search_points.pose.orientation.w = 1
    search_points.pose.orientation.z = 1

    search_points.header.frame_id = "map"




    #search_points = PositionTarget()
    #search_points.position.x= 0
    #search_points.position.y= 0
    #search_points.position.z= 10
    #search_points.acceleration_or_force.x= 1
    #search_points.acceleration_or_force.y= 1
    #search_points.acceleration_or_force.z= 1
    #search_points.pose.orientation.x = 10
    #search_points.pose.orientation.y = 10

    #search_points.pose.orientation.z = 10
    #search_points.pose.orientation.w = 10
    #search_points.header.frame_id = "map"
    #search_points.header.stamp = now

    ranges = scan.ranges
    print('exploring mode')
    print(min(ranges))
    if min(ranges)<1:
        #search_points.position.z = 10
        print('avoiding move')
    #angle_min = scan.angle_min
    #increment = scan.angle_increment
    search_pub.publish(search_points)




def main():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    print('spinning')
    rospy.spin()

if __name__ == '__main__':
    main()