#! /usr/bin/env python

import rospy
import math
import tf
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('control_decision_drone')

control_decision_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)

state=1
curr_pos = [0,0,0]
rrt_list=[]
battery_percentage = 1000
exploration_point_x = 1000
x_charge=0
y_charge=0
z_charge = 1
dist_home=0
def callback_gps(gps):
    global curr_pos
    global rrt_list
    global state
    global battery_percentage
    global exploration_point_x
    global dist_home
    if battery_percentage<1000 and exploration_point_x <1000:
        if state==1:
            curr_pos[0]=gps.pose.position.x
            curr_pos[1]=gps.pose.position.y
            curr_pos[2]=gps.pose.position.z
            dist_home = math.sqrt((math.pow((curr_pos[0]-x_charge), 2) + math.pow((curr_pos[1]-y_charge), 2)+ math.pow((curr_pos[2]-z_charge), 2)))
            print("dist home", dist_home)


        if battery_percentage < dist_home +5:
            state=2
            print(state)
            go_to_home=PoseStamped()
            go_to_home.pose.position.x= 0
            go_to_home.pose.position.y = 0
            go_to_home.pose.position.z= 1

            dx = x_charge - curr_pos[0]
            dy = y_charge - curr_pos[1]
            dz = z_charge - curr_pos[2]

            yaw=math.atan2(dy, dx)
            pitch = math.atan2(math.sqrt(dy * dy + dx * dx), dz)
            quat = tf.transformations.quaternion_from_euler(0, pitch, yaw, 'syxz')
            go_to_home.pose.orientation.x = quat[0]
            go_to_home.pose.orientation.y= quat[1]
            go_to_home.pose.orientation.z = quat[2]
            go_to_home.pose.orientation.w = quat[3]

            go_to_home.header.frame_id = "map"
            control_decision_pub.publish(go_to_home)

def callback_battery(battery):
    global battery_percentage
    battery_percentage=battery.percentage

def callback_exploration(explore):
    global state
    global exploration_point_x
    exploration_point_x = explore.pose.position.x
    print(state)
    if state ==1:
        control_decision_pub.publish(explore)



def main():
    exploration_sub = rospy.Subscriber('/mavros/setpoint_position/local1', PoseStamped, callback_exploration)
    battery_sub = rospy.Subscriber('battery_percentage', BatteryState, callback_battery)
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)

    rospy.spin()

if __name__ == '__main__':
    main()

