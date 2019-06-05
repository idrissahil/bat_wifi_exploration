#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped

rospy.init_node('battery_ae_explore')
battery_pub = rospy.Publisher('battery_percentage', PoseStamped, queue_size=1)
ae_pub = rospy.Publisher('/mavros/setpoint_position/local2', PoseStamped, queue_size=1)
rate = rospy.Rate(50)
battery_percentage=1
approx_curr=1
stationary_drain=0.0001
#save previous state of gps, calculate difference and use constant that calculates how much battery drained
old_location_x=1
old_location_y=1
old_location_z=1

default_velocity=2

x_charge=3
y_charge=3
z_charge = 1
def callback_gps(gps):
    global x_charge
    global y_charge
    global z_charge
    global approx_curr
    global battery_percentage
    global stationary_drain
    global default_velocity
    global old_location_x
    global old_location_y
    global old_location_z
    time_begin = rospy.Time(720)
    time_now = rospy.get_rostime()

    if time_now<time_begin:
        print('not_yet_started')
        old_location_x=gps.pose.position.x
        old_location_y=gps.pose.position.y
        old_location_z=gps.pose.position.z

        battery = PoseStamped()
        battery_percentage=1
        battery.pose.position.x = battery_percentage
        battery_pub.publish(battery)

    if time_now>time_begin:
        print('started')
        new_location_x = gps.pose.position.x
        new_location_y = gps.pose.position.y
        new_location_z = gps.pose.position.z

        percentage_loss=approx_curr*(math.pow((new_location_x-old_location_x), 2) + math.pow((new_location_y-old_location_y), 2)+ math.pow((new_location_z-old_location_z), 2))/default_velocity + stationary_drain
        print("percentage lossp", percentage_loss)
        battery_percentage=battery_percentage-percentage_loss
        charge_diff=(math.pow((new_location_x-x_charge), 2) + math.pow((new_location_y-y_charge), 2)+ math.pow((new_location_z-z_charge), 2))
        if battery_percentage < 0.1:
            battery_percentage = 0
            print("battery drained")
        if charge_diff<0.5:
            battery_percentage=battery_percentage+0.1
            if battery_percentage>100:
                battery_percentage=100
                print("fully charged")

        battery = PoseStamped()
        battery.pose.position.x = battery_percentage
        battery_pub.publish(battery)

        charging_setpoint=PoseStamped()
        charging_setpoint.pose.position.x=3
        charging_setpoint.pose.position.y=3
        charging_setpoint.pose.position.z=7
        charging_setpoint.pose.orientation.z = 1
        charging_setpoint.pose.orientation.w = 1
        #charging_setpoint.header.frame_id = "map"
        ae_pub.publish(charging_setpoint)




        old_location_x=new_location_x
        old_location_y=new_location_y
        old_location_z=new_location_z
    print('battery_percentage', battery_percentage)




def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    rospy.spin()


if __name__ == '__main__':
    main()