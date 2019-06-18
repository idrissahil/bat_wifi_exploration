#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('control_decision_drone')

control_decision_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

state=1
curr_pos = [0,0,0]
rrt_list=[]
battery_percentage = 1000
exploration_point_x = 1000

def callback_gps(gps):
    global curr_pos
    global rrt_list
    global state
    global battery_percentage
    global exploration_point_x
    if battery_percentage<1000 and exploration_point_x <1000:
        if state==1:
            curr_pos[0]=gps.pose.position.x
            curr_pos[1]=gps.pose.position.y
            curr_pos[2]=gps.pose.position.z

        if battery_percentage < 0.1:
            state=2
            print(state)
            hold_position=PoseStamped()
            #hold_position.pose.position.x= 0
            #hold_position.pose.position.y = 14
            #hold_position.pose.position.z= 1

            hold_position.pose.position.x= curr_pos[0]
            hold_position.pose.position.y = curr_pos[1]
            hold_position.pose.position.z= curr_pos[2]
            hold_position.header.frame_id = "map"
            control_decision_pub.publish(hold_position)

def callback_battery(battery):
    global battery_percentage
    battery_percentage = battery.pose.position.x

def callback_exploration(explore):
    global state
    global exploration_point_x
    exploration_point_x = explore.pose.position.x
    print(state)
    if state ==1:
        control_decision_pub.publish(explore)



def main():
    exploration_sub = rospy.Subscriber('/mavros/setpoint_position/local1', PoseStamped, callback_exploration)
    battery_sub = rospy.Subscriber('battery_percentage', PoseStamped, callback_battery)
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)

    rospy.spin()

if __name__ == '__main__':
    main()

