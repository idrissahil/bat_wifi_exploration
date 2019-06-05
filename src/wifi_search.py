#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState, Temperature
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('search_wifi_node')

search_wifi_pub = rospy.Publisher('wifi_search', Temperature, queue_size=1)
rate = rospy.Rate(50)

sigma_x=5
sigma_y=5
sigma_z=5

x_my=1
y_my=1
z_my=1

A=1


def callback_setpoint(search_point):
    search_x = search_point.pose.position.x
    search_y  = search_point.pose.position.y
    search_z = search_point.pose.position.z


    wifi_strength = A * math.exp(-((((search_x - x_my) * (search_x - x_my)) / (2 * sigma_x * sigma_x)) + (((search_y - y_my) *(search_y - y_my)) / (2 * sigma_y * sigma_y)) + (
                ((search_z - z_my) * (search_z - z_my)) / (2 * sigma_z * sigma_z))))
    wifi=Temperature()
    wifi.temperature=wifi_strength
    search_wifi_pub.publish(wifi)



def main():
    search_sub = rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, callback_setpoint)
    rospy.spin()


if __name__ == '__main__':
    main()
