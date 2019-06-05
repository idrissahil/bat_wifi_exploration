#! /usr/bin/env python

import rospy
import math
from scipy.optimize import *
from numpy import *
from sensor_msgs.msg import BatteryState, Temperature
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('dist_wifi_node')

wifi_pub = rospy.Publisher('wifi_dist', PoseStamped, queue_size=1)
rate = rospy.Rate(50)


curr_wifi=0
first_set=[]
initial_sigma_x=2
initial_sigma_y=2
initial_sigma_z=2

x_my=0
y_my=0
z_my=0

A=1

wifi_aware=True

wifi_list=[]
past_locations_x=[]
past_locations_y=[]
past_locations_z=[]
p=[]

def myfunction(p, curr_location_x, curr_location_y, curr_location_z, f_list):
    x = p[0]
    y = p[1]
    z = p[2]
    f= empty(3)

    f[0] = math.log(f_list[2]) + ((curr_location_x[2]-x_my)**2)/(2*(x**2)) + ((curr_location_y[2]-y_my)**2)/(2*(y**2)) + ((curr_location_z[2]-z_my)**2)/(2*(z**2))
    f[1] = math.log(f_list[1]) + ((curr_location_x[1]-x_my)**2)/(2*(x**2)) + ((curr_location_y[1]-y_my)**2)/(2*(y**2)) + ((curr_location_z[1]-z_my)**2)/(2*(z**2))
    f[2] = math.log(f_list[0]) + ((curr_location_x[0]-x_my)**2)/(2*(x**2)) + ((curr_location_y[0]-y_my)**2)/(2*(y**2)) + ((curr_location_z[0]-z_my)**2)/(2*(z**2))

    return f


def callback_gps(gps):
    global p
    if not curr_wifi==0:
        zGuess = array([initial_sigma_x, initial_sigma_y, initial_sigma_z])
        curr_location_x = gps.pose.position.x
        curr_location_y = gps.pose.position.y
        curr_location_z = gps.pose.position.z

        past_locations_x.append(curr_location_x)
        past_locations_y.append(curr_location_y)
        past_locations_z.append(curr_location_z)

        if len(past_locations_x)%500==0:
            curr_location_x_3=[past_locations_x[-1], past_locations_x[-200], past_locations_x[-400]]
            curr_location_y_3=[past_locations_y[-1], past_locations_y[-200], past_locations_y[-400]]
            curr_location_z_3=[past_locations_z[-1], past_locations_z[-200], past_locations_z[-400]]
            f_list_3=[wifi_list[-1],wifi_list[-200], wifi_list[-400] ]

            p = fsolve(myfunction, zGuess, (curr_location_x_3, curr_location_y_3, curr_location_z_3, f_list_3))
            print("estimated sigma", p)


            #print("past location 1",past_locations_x[-1], past_locations_y[-1], past_locations_z[-1], wifi_list[-1] )
            #print("past location 200",past_locations_x[-200], past_locations_y[-200], past_locations_z[-200], wifi_list[-200])
            #print("past location 400",past_locations_x[-400], past_locations_y[-400], past_locations_z[-400], wifi_list[-400])
        distribution=PoseStamped()
        distribution.pose.position.x =initial_sigma_x
        distribution.pose.position.y =initial_sigma_y
        distribution.pose.position.z =initial_sigma_z
        if wifi_aware==True:
            distribution.pose.orientation.w = 1
        if len(past_locations_x)>501:
            distribution.pose.position.x = p[0]
            distribution.pose.position.y = p[1]
            distribution.pose.position.z = p[2]
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
