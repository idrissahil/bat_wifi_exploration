#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState, Temperature
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('current_wifi_node')

wifi_pub = rospy.Publisher('wifi_curr', Temperature, queue_size=1)
rate = rospy.Rate(50)

sigma_x=5
sigma_y=6
sigma_z=7

x_my=1
y_my=2
z_my=3

A=1


def callback_gps(gps):
    curr_location_x = gps.pose.position.x
    curr_location_y = gps.pose.position.y
    curr_location_z = gps.pose.position.z
    wifi_strength =1/(3*(1+sigma_x*abs(curr_location_x-x_my))) + 1/(3*(1+sigma_y*abs(curr_location_y-y_my))) + 1/(3*(1+sigma_z*abs(curr_location_z-z_my)))
    wifi=Temperature()
    wifi.header.frame_id="map"
    wifi.temperature=wifi_strength
    wifi_pub.publish(wifi)



def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    rospy.spin()


if __name__ == '__main__':
    main()
