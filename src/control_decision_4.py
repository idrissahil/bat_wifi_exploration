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
index=0

def callback_gps(gps):
    global curr_pos
    global rrt_list
    global state
    global index
    curr_pos[0] = gps.pose.position.x
    curr_pos[1] = gps.pose.position.y
    curr_pos[2] = gps.pose.position.z
    if state==1:
        print(state)
        #curr_pos[0]=gps.pose.position.x
        #curr_pos[1]=gps.pose.position.y
        #curr_pos[2]=gps.pose.position.z

    if len(rrt_list)>1:
        state=2
        print(state)
        dist_point = math.sqrt(math.pow(rrt_list[index].x - curr_pos[0],2)+math.pow(rrt_list[index].y - curr_pos[1],2)+math.pow(rrt_list[index].z - curr_pos[2],2))
        if dist_point<0.1:
            index=index+1
        curr_position=PoseStamped()
        #hold_position.pose.position.x= 0
        #hold_position.pose.position.y = 14
        #hold_position.pose.position.z= 1

        curr_position.pose.position.x= rrt_list[index].x
        curr_position.pose.position.y= rrt_list[index].y
        curr_position.pose.position.z= rrt_list[index].z
        curr_position.header.frame_id = "map"
        control_decision_pub.publish(curr_position)

def callback_battery(rrt):
    global state
    global curr_pos
    global rrt_list
    rrt_list=rrt.poses

def callback_exploration(explore):
    global state
    global exploration_point_x
    exploration_point_x = explore.pose.position.x
    print(state)
    if state ==1:
        control_decision_pub.publish(explore)



def main():
    exploration_sub = rospy.Subscriber('/mavros/setpoint_position/local1', PoseStamped, callback_exploration)
    battery_sub = rospy.Subscriber('visual_marker_rrt', PoseArray, callback_battery)
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)

    rospy.spin()

if __name__ == '__main__':
    main()

