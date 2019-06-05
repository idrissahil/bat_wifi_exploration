#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped, TwistStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.msg import PositionTarget
import random
import math



rospy.init_node('rrt_charging_drone')

rrt_pub = rospy.Publisher('/mavros/setpoint_position/local2', PoseStamped, queue_size=1)
rrt_vis_pub = rospy.Publisher('visual_marker_rrt', PoseArray, queue_size=1)

rate = rospy.Rate(50)

ranges=[]
#success_node=[]
Node_list=[]
goal_node_list=[]
marks_list=[]


state_drone=1
index_rrt = 0

x_charge=19
y_charge=-3
z_charge=1


class Node():
    def __init__(self, x, y, z, x_diff, y_diff,z_diff,total_distance, parent_node = None ):
        self.x = x
        self.y = y
        self.z = z
        self.parent_node = parent_node
        self.total_parents=0
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.z_diff = z_diff
        self.total_distance = total_distance



boost_number=[0]
def rand_node(counter_boost):
    #print(goal_distance)
    x_min = -10
    y_min = -10
    z_min = -10

    x_max = 10
    y_max = 10
    z_max = 4
    x_rand = random.uniform(x_min, x_max)
    y_rand = random.uniform(y_min, y_max)
    z_rand = random.uniform(z_min, z_max)
    if counter_boost%10==0: #Boost the search towards the goal
        x_rand=x_charge
        y_rand = y_charge
        z_rand=z_charge
        #print("boost")
        boost_number[0]=boost_number[0]+1




    return x_rand, y_rand, z_rand

def find_closest_node(x_rand, y_rand, z_rand, node_list):
    dist_list = []
    for node in node_list:
        distance = math.sqrt(math.pow((node.x - x_rand), 2) + math.pow((node.y - y_rand), 2)+ math.pow((node.z - z_rand), 2))
        dist_list.append(distance)
    goal_distance = min(dist_list)
    closest_index = dist_list.index(min(dist_list))
    closest_node = node_list[closest_index]
    return closest_node, goal_distance, closest_index

def find_velocity(closest_node, x_rand, y_rand, z_rand):

    x_diff=x_rand-closest_node.x #meter per second speed
    y_diff=y_rand-closest_node.y
    z_diff=z_rand-closest_node.z
    speed_limit=1
    if x_diff >speed_limit:  #max speed set to 5 m/s
        x_diff=speed_limit
    if x_diff <-speed_limit:
        x_diff=-speed_limit
    if y_diff >speed_limit:
        y_diff=speed_limit
    if y_diff <-speed_limit:
        y_diff=-speed_limit
    if z_diff >speed_limit:
        z_diff=speed_limit
    if z_diff <-speed_limit:
        z_diff=-speed_limit

    return x_diff, y_diff, z_diff

def Collision(x, y, z, obstacle_list):
    collision = False
    for i in range(len(obstacle_list)):
        dx = x - obstacle_list[i].x
        dy = y - obstacle_list[i].y
        dz = z - obstacle_list[i].z

        if abs(dx)<0.3 and abs(dy)<0.3 and abs(dz)<0.3:
            print("abs collision check")
            collision=True

        #dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)+ math.pow(dz, 2))
        #if dist <= 0.5:
        #    collision=True
        #    print("collision 111111111")
    if x <=-20  or x > 20:
        print("2")
        collision = True
    if y <= -20  or y > 20:
        print("3")
        collision = True
    if z < -0.1 or z > 4.1:
        #print("4")
        collision = True
    return collision

def go_to_goal(near_x , near_y, near_z, x_diff, y_diff, z_diff, marks_list):
    curr_x=near_x
    curr_y=near_y
    curr_z=near_z
    distance_time=0.005
    step_num=100

    collision = False
    curr_x = curr_x + x_diff* distance_time*step_num  #Go to node for 100 *distance_time
    curr_y = curr_y + y_diff*distance_time*step_num
    curr_z = curr_z + z_diff*distance_time*step_num
    collision = Collision(curr_x, curr_y, curr_z, marks_list)
        #if collision==False:
            #break
    return curr_x, curr_y, curr_z, collision

def go_to_goal2(near_x , near_y, near_z, x_diff, y_diff, z_diff, marks_list):
    curr_x=near_x
    curr_y=near_y
    curr_z=near_z
    distance_time=0.01
    step_num=100
    counter=0
    collision = False
    while counter < 100:
        curr_x = curr_x + x_diff* distance_time #Go to node for 100 *distance_time
        curr_y = curr_y + y_diff*distance_time
        curr_z = curr_z + z_diff*distance_time
        counter = counter + 1
        collision = Collision(curr_x, curr_y, curr_z, marks_list)
        if collision==True:
            curr_x = curr_x -10* x_diff * distance_time  # Go to node for 100 *distance_time
            curr_y = curr_y -10* y_diff * distance_time
            curr_z = curr_z -10* z_diff * distance_time
            collision = Collision(curr_x, curr_y, curr_z, marks_list)
            #print(collision)
            break
    return curr_x, curr_y, curr_z, collision



def backtracking(Node_List, final_node):
    controls_x = []
    controls_y = []
    controls_z = []
    x_drone =[]
    y_drone =[]
    z_drone =[]
    goal_node_list=[]

    times = []
    #index = len(Node_List)
    index=len(Node_List)-1
    print(index)
    car_time=0
    for i in range(final_node.total_parents):

        true_node = Node_List[index]
        goal_node_list.append(true_node)
        x_drone.append(true_node.x)
        y_drone.append(true_node.y)
        z_drone.append(true_node.z)

        controls_x.append(true_node.x_diff)
        controls_y.append(true_node.y_diff)
        controls_z.append(true_node.z_diff)
        index = true_node.parent_node
    x_drone.reverse()
    y_drone.reverse()
    z_drone.reverse()
    controls_x.reverse()
    controls_y.reverse()
    controls_z.reverse()
    goal_node_list.reverse()
    return controls_x, controls_y,controls_z, x_drone, y_drone, z_drone, goal_node_list


def choose_parent(curr_x, curr_y, curr_z, node_list, closest_index):
    bounding_radius=1.2
    dist_list=[]
    parent_index=0
    best_dist=100000
    total_distance_constant=1

    for i in range(len(node_list)):
        dx = curr_x - node_list[i].x
        dy = curr_y - node_list[i].y
        dz = curr_z - node_list[i].z
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)+ math.pow(dz, 2))

        if dist<bounding_radius:
            tot_dist=dist + node_list[i].total_distance*total_distance_constant
            if i==0:
                tot_dist=tot_dist+1
            if tot_dist < best_dist:
                #print("lowered", i)
                parent_index = i
                best_dist = tot_dist
        dist_list.append(dist)
        if min(dist_list)>bounding_radius:
            #print("no nodes inside bouding area")
            parent_index=closest_index


        #print("i", i, "tot_dist", tot_dist)
    #print("best dist", best_dist, "index", parent_index)
    parent_node=node_list[parent_index]
    return parent_index, parent_node


def main_rrt(start_x, start_y,start_z, marks_list):
    start_node = Node(start_x, start_y, start_z, x_diff=0, y_diff=0, z_diff=0, total_distance=0)
    Node_List = [start_node]
    goal_reach_distance = 1
    goal_reached = False
    goal_distance2 = 10
    counter_boost = 0
    while goal_reached == False and len(Node_List) < 3000:
        x_rand, y_rand, z_rand = rand_node(counter_boost)
        closest_node, goal_distance, closest_index = find_closest_node(x_rand, y_rand, z_rand, Node_List)
        x_diff, y_diff, z_diff = find_velocity(closest_node, x_rand, y_rand, z_rand)
        curr_x, curr_y, curr_z, collision = go_to_goal(closest_node.x, closest_node.y, closest_node.z, x_diff, y_diff, z_diff, marks_list)
        parent_index, parent_node = choose_parent(curr_x, curr_y, curr_z, Node_List, closest_index)
        x_diff2, y_diff2, z_diff2 = find_velocity(parent_node, curr_x, curr_y, curr_z)
        curr_x, curr_y, curr_z, collision = go_to_goal2(parent_node.x, parent_node.y, parent_node.z, x_diff2, y_diff2, z_diff2 , marks_list)
        distance_travelled = math.sqrt(math.pow((curr_x - parent_node.x), 2) + math.pow((curr_y - parent_node.y), 2) + math.pow((curr_z - parent_node.z), 2))
        print("distance travelled",distance_travelled)
        if collision == False:
            Suc_Node = Node(x=curr_x, y=curr_y, z=curr_z, parent_node=parent_index, x_diff=x_diff, y_diff=y_diff, z_diff=z_diff, total_distance=0)
            Suc_Node.total_parents = 1 + parent_node.total_parents
            Suc_Node.total_distance = distance_travelled + parent_node.total_distance
            Node_List.append(Suc_Node)
            goal_distance_curr = math.sqrt(math.pow((x_charge - Suc_Node.x), 2) + math.pow((y_charge - Suc_Node.y), 2) + math.pow((z_charge - Suc_Node.z), 2))
            if goal_distance_curr<goal_distance2:
                goal_distance2=goal_distance_curr
        print(goal_distance2)
        if goal_distance2 <= goal_reach_distance:
            print("reached", goal_distance2)
            goal_reached = True
        print("nodes", len(Node_List))
        counter_boost = counter_boost + 1
    controls_x, controls_y,controls_z, x_drone, y_drone, z_drone, goal_node_list = backtracking(Node_List, Suc_Node)

    print("number of nodes",len(Node_List))
    print("boost number", boost_number)
    print("x last node", Suc_Node.x, "ylast node", Suc_Node.y, "z last node", Suc_Node.z)
    #print(70*0.0001*sum(controls_x)) # controls given in 70*0.0001

    #print('x_start',x_drone[0], 'x_end', x_drone[-1])
    print("total distance",Suc_Node.total_distance)
    #plt.plot(x_drone, y_drone, 'ro')
    #plt.axis([-10, 10, -10, 10])
    #plt.show()
    return Suc_Node, Node_List, goal_node_list

def callback_gps(gps):
    global ranges
    global angle_min
    global state_drone
    global Node_list
    global goal_node_list
    global success_node
    global index_rrt
    global marks_list
    #print ("state drone", state_drone)
    if marks_list is not None:
        rrt_poses = PoseArray()
        rrt_poses.header.stamp = rospy.Time.now()
        rrt_poses.header.frame_id = 'map'
        if state_drone==1:
            success_node, Node_list, goal_node_list = main_rrt(gps.pose.position.x, gps.pose.position.y, gps.pose.position.z, marks_list )
            #print("goal node",Node_list)
            state_drone = 2
        curr_rrt=PoseStamped()
        #print("index", index_rrt)
        #curr_rrt.pose.position.x=goal_node_list[index_rrt].x
        #curr_rrt.pose.position.y=goal_node_list[index_rrt].y
        #curr_rrt.pose.position.z=goal_node_list[index_rrt].z
        #curr_rrt.header.frame_id = "map"
        #rrt_pub.publish(curr_rrt)
        #distance_curr_rrt = math.sqrt(math.pow((gps.pose.position.x - goal_node_list[index_rrt].x), 2) + math.pow((gps.pose.position.y - goal_node_list[index_rrt].y), 2) + math.pow((gps.pose.position.z - goal_node_list[index_rrt].z), 2))
        #if distance_curr_rrt<0.5 and index_rrt<len(goal_node_list)-1:
        #    index_rrt=index_rrt+1
        for i in range(len(goal_node_list)):
            curr_point_rrt = Pose()
            curr_point_rrt.position.x = goal_node_list[i].x
            curr_point_rrt.position.y =  goal_node_list[i].y
            curr_point_rrt.position.z =  goal_node_list[i].z
            rrt_poses.poses.append(curr_point_rrt)
        '''

        for i in range(len(Node_list)):
            curr_point_rrt = Pose()
            curr_point_rrt.position.x = Node_list[i].x
            curr_point_rrt.position.y = Node_list[i].y
            curr_point_rrt.position.z = Node_list[i].z
            rrt_poses.poses.append(curr_point_rrt)

        '''


        rrt_vis_pub.publish(rrt_poses)

            #curr_point_rrt.orientation.z = -3.14 / 2
            #curr_point_rrt.orientation.x = 2
        #print ("state drone2", state_drone)
        #print("success node", Node_list[3].z)

def callback_markers(marks):
    global marks_list
    marks_unpacked=marks.markers[16].points
    marks_list = marks.markers[16].points
    #print("marks list", marks_list[2].x)

    #print("marks_unpacked first point", marks_unpacked[0].x)
    #print(marks_unpacked)


def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    vis_mark_sub = rospy.Subscriber('/aeplanner/occupied_cells_vis_array', MarkerArray, callback_markers)
    print('spinning')
    rospy.spin()

if __name__ == '__main__':
    main()
