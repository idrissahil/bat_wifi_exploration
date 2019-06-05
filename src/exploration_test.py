#! /usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
import random
import math

rospy.init_node('exploration_test_node')


search_pub = rospy.Publisher('/setpoint',PoseStamped,queue_size=1)
rate = rospy.Rate(50)





global battery_perc


counter=0

global counter
ranges=[]
x_charge=3
y_charge=3
z_charge = 1
battery_constant = 0.20
x_search=2
y_search=2
z_search = 2
battery_minimum = 10
state=0
battery_perc=100

def callback_gps(gps):
    global x_charge
    global y_charge
    global z_charge
    global x_search
    global y_search
    global z_search
    global battery_constant
    global battery_minimum
    global state
    global battery_perc
    if not battery_perc is None and not len(ranges)==0 :
        if battery_perc > battery_minimum:
            if state==0:
                x_search = random.uniform(0, 10)
                y_search = random.uniform(0, 10)
                z_search = 2

                state=1
                print("explore 1")
            curr_location_x = gps.pose.position.x
            curr_location_y = gps.pose.position.y
            curr_location_z = gps.pose.position.z
            diff_point =(math.pow((curr_location_x - x_search), 2) + math.pow((curr_location_y - y_search), 2) + math.pow((curr_location_z - z_search), 2))
            print(diff_point)
            if diff_point <2: # add time as well
                x_search = random.uniform(0, 10)
                y_search = random.uniform(0, 10)
                z_search = 2
                state=1
                print("explore 2")
            if min(ranges)<1:
                state=2
                print("too close to obstacle")
                x_search=curr_location_x
                y_search=curr_location_y
                z_search=10
            search_points=PoseStamped()
            search_points.pose.position.x = x_search
            search_points.pose.position.y = y_search
            search_points.pose.position.z = z_search
            search_points.pose.orientation.w = 1
            search_points.header.frame_id = "map"
            search_pub.publish(search_points)
            battery_minimum = battery_constant * (math.pow((curr_location_x - x_charge), 2) + math.pow((curr_location_y - y_charge), 2) + math.pow((curr_location_z - z_charge), 2))


            #battery_minimum =battery_constant*(math.pow((x_search-x_charge), 2) + math.pow((y_search-y_charge), 2) + math.pow((z_search-z_charge), 2))

        if battery_perc<battery_minimum:
            print(battery_perc)
            print(battery_minimum)
            charge_points = PoseStamped()
            charge_points.pose.position.x = x_charge
            charge_points.pose.position.y = y_charge
            charge_points.pose.position.z = z_charge
            charge_points.pose.orientation.w = 1
            charge_points.header.frame_id = "map"
            search_pub.publish(charge_points)
            print("travelling to charging station")


def callback_lidar(scan):
    global ranges
    global angle_min
    global increment
    if not battery_perc is None:    #both subscribers dont start same time
        ranges = scan.ranges
        angle_min = scan.angle_min
        increment = scan.angle_increment


def callback_battery(battery):
    global battery_perc
    battery_perc = battery.percentage

def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    battery_sub = rospy.Subscriber('battery_percentage', BatteryState, callback_battery)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)

    rospy.spin()

if __name__ == '__main__':
    main()