#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('battery_percentage_node')

battery_pub = rospy.Publisher('battery_percentage', PoseStamped, queue_size=1)
dist_pub = rospy.Publisher('distance_flown', PoseStamped, queue_size=1)

rate = rospy.Rate(50)
battery_percentage=1
battery_constant=1
#save previous state of gps, calculate difference and use constant that calculates how much battery drained
old_location_x=1
old_location_y=1
old_location_z=1
total_dist_flown = 0
x_charge=0
y_charge=0
z_charge = 1
time_begin=None
counter = 0

def callback_gps(gps):
    global total_dist_flown
    global x_charge
    global y_charge
    global z_charge
    global battery_constant
    global battery_percentage
    global old_location_x
    global old_location_y
    global old_location_z
    global time_begin
    global counter
    battery_cost=True
    time_now = rospy.get_rostime()
    if time_begin==None:
        time_begin = rospy.get_rostime()

    if not time_now>time_begin:
        print('not_yet_started')
        old_location_x=gps.pose.position.x
        old_location_y=gps.pose.position.y
        old_location_z=gps.pose.position.z

        battery = PoseStamped()
        battery_percentage=100
        battery.pose.position.x = battery_percentage
        if battery_cost == True:
            battery.pose.position.y = 1
        battery_pub.publish(battery)

    if time_now>time_begin:
        print('started')
        new_location_x = gps.pose.position.x
        new_location_y = gps.pose.position.y
        new_location_z = gps.pose.position.z

        percentage_loss=battery_constant*math.sqrt((math.pow((new_location_x-old_location_x), 2) + math.pow((new_location_y-old_location_y), 2)+ math.pow((new_location_z-old_location_z), 2)))
        print("percentage lossp", percentage_loss)
        battery_percentage=battery_percentage-percentage_loss
        charge_diff=math.sqrt((math.pow((new_location_x-x_charge), 2) + math.pow((new_location_y-y_charge), 2)+ math.pow((new_location_z-z_charge), 2)))
        total_dist_flown=total_dist_flown+abs(percentage_loss/battery_constant)
        if battery_percentage < 0.1:
            battery_percentage = 0
            print("battery drained")
            # 83 m is the total distance that you can travel with the drone.
        if charge_diff<0.5:
            battery_percentage=battery_percentage
            if battery_percentage>100:
                battery_percentage=100
                print("fully charged")

        battery = PoseStamped()
        battery.pose.position.x = battery_percentage
        if battery_cost == True:
            battery.pose.position.y = 1
        battery_pub.publish(battery)

        old_location_x=new_location_x
        old_location_y=new_location_y
        old_location_z=new_location_z
        pose_dist=PoseStamped()
        pose_dist.pose.orientation.w=total_dist_flown
        dist_pub.publish(pose_dist)
    counter = counter + 1
    print('battery_percentage', battery_percentage)
    print("total dist travelled", total_dist_flown)




def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    rospy.spin()


if __name__ == '__main__':
    main()