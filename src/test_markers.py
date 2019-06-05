#! /usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import PositionTarget
import random
import math



rospy.init_node('marker_Test')
test_pub = rospy.Publisher('visual_marker_test', PoseArray, queue_size=1)
rate = rospy.Rate(50)

ranges=[]





def callback_lidar(scan):
    global ranges
    global angle_min

    test_visual_poses = PoseArray()
    test_visual_poses.header.stamp = rospy.Time.now()
    test_visual_poses.header.frame_id = 'map'

    test_point = Pose()
    test_point.position.x = 2
    test_point.position.y = 3
    test_point.position.z = 2
    test_point.orientation.z = -3.14/2
    test_point.orientation.x = 2



    test_point2 = Pose()
    test_point2.position.x = 2
    test_point2.position.y = 3
    test_point2.position.z = 3
    test_point2.orientation.z = 3
    test_point.orientation.x = 2




    test_visual_poses.poses.append(test_point)
    test_visual_poses.poses.append(test_point2)

    test_pub.publish(test_visual_poses)




def main():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    main()