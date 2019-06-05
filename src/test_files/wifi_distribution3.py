#! /usr/bin/env python

import rospy
import math
from sympy import apart
from sympy.abc import x, y, z
from sympy import solve, symbols, solveset, S
from sensor_msgs.msg import BatteryState, Temperature
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('dist_wifi_node')

wifi_pub = rospy.Publisher('wifi_dist', PoseStamped, queue_size=1)
rate = rospy.Rate(50)


curr_wifi=0

sigma_x=5
sigma_y=5
sigma_z=5

x_my=1
y_my=2
z_my=3

A=1

wifi_aware=False

wifi_list=[]
past_locations_x=[]
past_locations_y=[]
past_locations_z=[]


def callback_gps(gps):
    if not curr_wifi==0:
        x, y, z = symbols('x,y,z')
        curr_location_x = gps.pose.position.x
        curr_location_y = gps.pose.position.y
        curr_location_z = gps.pose.position.z

        past_locations_x.append(curr_location_x)
        past_locations_y.append(curr_location_y)
        past_locations_z.append(curr_location_z)

        if len(past_locations_x)%500==0:

            sol4 = solve([-wifi_list[-400] + 1/(3*(1+x*abs(past_locations_x[-400]-x_my))) + 1/(3*(1+y*abs(past_locations_y[-400]-y_my))) + 1/(3*(1+z*abs(past_locations_z[-400]-z_my))), \
                          -wifi_list[-200] + 1/(3*(1+x*abs(past_locations_x[-200]-x_my))) + 1/(3*(1+y*abs(past_locations_y[-200]-y_my))) + 1/(3*(1+z*abs(past_locations_z[-200]-z_my))), \
                          -wifi_list[-1] + 1/(3*(1+x*abs(past_locations_x[-1]-x_my))) + 1/(3*(1+y*abs(past_locations_y[-1]-y_my))) + 1/(3*(1+z*abs(past_locations_z[-1]-z_my)))], [x, y, z])
            #first_set = sol4[0]
            print(sol4)
            #print("past location 1",past_locations_x[-1], past_locations_y[-1], past_locations_z[-1], wifi_list[-1] )
            #print("past location 200",past_locations_x[-200], past_locations_y[-200], past_locations_z[-200], wifi_list[-200])
            #print("past location 400",past_locations_x[-400], past_locations_y[-400], past_locations_z[-400], wifi_list[-400])

        distribution=PoseStamped()
        distribution.pose.position.x =sigma_x
        distribution.pose.position.y =sigma_y
        distribution.pose.position.z =sigma_z
        if wifi_aware==True:
            distribution.pose.orientation.w = 1

        wifi_pub.publish(distribution)


def callback_wifi(wifi):
    global curr_wifi
    curr_wifi=wifi.temperature
    wifi_list.append(curr_wifi)

def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    wifi_sub = rospy.Subscriber('wifi_curr', Temperature, callback_wifi)

    rospy.spin()


if __name__ == '__main__':
    main()
