#! /usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import random
import math



sigma_x=5
x_my=0
wifi_strength_minimum = 0.55

rospy.init_node('wifi_circle')

wifi_circ_pub = rospy.Publisher('wifi_circle', Marker,queue_size=1)
rate = rospy.Rate(50)




def callback_lidar(scan):
    diameter =  math.sqrt((-math.log(wifi_strength_minimum))*(2*sigma_x*sigma_x))*2 + x_my
    print("diamter", diameter)
    marker=Marker()
    marker.header.frame_id = "map"
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 1

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    wifi_circ_pub.publish(marker)

def main():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)

    rospy.spin()

if __name__ == '__main__':
    main()
