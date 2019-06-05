#! /usr/bin/env python

import rospy
import math
import octomap
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from octomap_msgs.msg import Octomap

rospy.init_node('octomap_test')

#free_pub = rospy.Publisher('octomap_free', PoseStamped, queue_size=1)
rate = rospy.Rate(50)

free_cells=0

def callback_octomap(octomap):
    global free_cells
    for i in range(len(octomap.data)):
        if octomap.data[i]==0:
            free_cells=free_cells+1
    print("octomap, 64 cell", octomap.data[64])
    print("octomap, size", len(octomap.data))
    print("free cells", free_cells)
    print("free_cells ratio", (free_cells)/(len(octomap.data)))
    free_cells=0




def main():
    gps_sub = rospy.Subscriber('/aeplanner/octomap_binary', Octomap , callback_octomap)
    rospy.spin()


if __name__ == '__main__':
    main()
