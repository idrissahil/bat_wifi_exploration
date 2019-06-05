#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('control_decision_drone')

control_decision_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)


state=1


def callback_battery(battery_explore):
    global state
    global exploration_point_x
    global battery_point_x
    #if not exploration_point_x is None:    #both subscribers dont start same time
    battery_point_x=battery_explore.pose.position.x
    if not battery_point_x is None:
        state=2
        print(state)
        control_decision_pub.publish(battery_explore)

def callback_exploration(explore):
    global state
    global exploration_point_x
    exploration_point_x = explore.pose.position.x
    print(state)
    if state ==1:
        control_decision_pub.publish(explore)



def main():
    exploration_sub = rospy.Subscriber('/mavros/setpoint_position/local1', PoseStamped, callback_exploration)
    battery_sub = rospy.Subscriber('/mavros/setpoint_position/local2', PoseStamped, callback_battery)

    rospy.spin()

if __name__ == '__main__':
    main()

