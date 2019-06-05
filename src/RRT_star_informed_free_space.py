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
import tf
from collections import defaultdict

rospy.init_node('rrt_charging_drone')

rrt_pub = rospy.Publisher('/mavros/setpoint_position/local2', PoseStamped, queue_size=1)
rrt_vis_pub = rospy.Publisher('visual_marker_rrt', PoseArray, queue_size=1)

rate = rospy.Rate(50)

ranges = []
# success_node=[]
Node_list = []
goal_node_list = []
marks_list = []
free_list = []

state_drone = 1
index_rrt = 0

x_charge = 0
y_charge = 0
z_charge = 1


class Node():
    def __init__(self, x, y, z, x_diff, y_diff, z_diff, total_distance, parent_node=None, x_ori=0, y_ori=0, z_ori=0, w_ori=0):
        self.x = x
        self.y = y
        self.z = z
        self.parent_node = parent_node
        self.total_parents = 0
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.z_diff = z_diff
        self.total_distance = total_distance
        self.x_ori = x_ori
        self.y_ori = y_ori
        self.z_ori = z_ori
        self.w_ori = w_ori


boost_number = [0]


def rand_node(counter_boost, best_total_distance, min_distance, phi_rotation, x_half, y_half):
    #start_rand = time.time()
    c_best = best_total_distance
    #print(c_best)
    c_min = min_distance
    if c_best == 3000:
        # print(goal_distance) #regular rrt and first iteration
        x_min = -40
        y_min = -40
        z_min = -10

        x_max = 40
        y_max = 40
        z_max = 4
        x_rand = random.uniform(x_min, x_max)
        y_rand = random.uniform(y_min, y_max)
        z_rand = random.uniform(z_min, z_max)

    # informed rrt
    if c_best < 3000:
        print("x half", x_half)
        print("y half", y_half)
        z_min = 0
        z_max = 2
        x_max_i = c_best
        # print(x_max_i)
        print("c_best", c_best, "c_min", c_min)
        # print("error value",(c_best*c_best - c_min*c_min) )
        y_max_i = math.sqrt(c_best * c_best - c_min * c_min)
        rho_rand = random.uniform(0, 1)
        phi_rand = random.uniform(0, 2 * math.pi)
        x_rot = math.sqrt(rho_rand) * math.cos(phi_rand)
        y_rot = math.sqrt(rho_rand) * math.sin(phi_rand)

        x_rot = x_rot * x_max_i / 2
        y_rot = y_rot * y_max_i / 3

        x_rand = x_rot * math.cos(phi_rotation) - y_rot * math.sin(phi_rotation) + x_half
        y_rand = x_rot * math.sin(phi_rotation) + y_rot * math.cos(phi_rotation) + y_half
        z_rand = random.uniform(z_min, z_max)
        # print("x rand", x_rand)

    if counter_boost % 10 == 0:  # Boost the search towards the goal
        x_rand = x_charge
        y_rand = y_charge
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1
    '''
    if counter_boost % 110 == 0:  # Boost the search towards the goal
        x_rand = 500
        y_rand = 500
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1

    if counter_boost % 80 == 0:  # Boost the search towards the goal
        x_rand = 500
        y_rand = -500
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1

    if counter_boost % 90 == 0:  # Boost the search towards the goal
        x_rand = -500
        y_rand = 500
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1

    if counter_boost % 120 == 0:  # Boost the search towards the goal
        x_rand = -500
        y_rand = -500
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1
    '''
    #end_rand = time.time()
    #print("rand_node", end_rand - start_rand)

    return x_rand, y_rand, z_rand


def find_closest_node(x_rand, y_rand, z_rand, node_list):
    #start = time.time()
    dist_list = []
    for node in node_list:
        distance = math.sqrt(
            math.pow((node.x - x_rand), 2) + math.pow((node.y - y_rand), 2) + math.pow((node.z - z_rand), 2))
        dist_list.append(distance)
    goal_distance = min(dist_list)
    closest_index = dist_list.index(min(dist_list))
    closest_node = node_list[closest_index]

    #end = time.time()
    #print("find_closest_node", end - start)
    return closest_node, goal_distance, closest_index


def find_velocity(closest_node, x_rand, y_rand, z_rand):
    #start = time.time()
    x_diff = x_rand - closest_node.x  # meter per second speed
    y_diff = y_rand - closest_node.y
    z_diff = z_rand - closest_node.z
    speed_limit = 1
    if x_diff > speed_limit:  # max speed set to 1 m/s
        x_diff = speed_limit
    if x_diff < -speed_limit:
        x_diff = -speed_limit
    if y_diff > speed_limit:
        y_diff = speed_limit
    if y_diff < -speed_limit:
        y_diff = -speed_limit
    if z_diff > speed_limit:
        z_diff = speed_limit
    if z_diff < -speed_limit:
        z_diff = -speed_limit
    #end = time.time()
    #print("find_velocity", end - start)

    return x_diff, y_diff, z_diff


def round_of_rating(number, min_val):
    round_num = round(number * 2) / 2
    round_num = int((round_num - min_val) * 2)

    return round_num


def index_collision_list(obstacle_list):
    min_x = -40
    min_y = -40
    min_z = -2
    by_rounded_coords = defaultdict(list)

    for obstacle in obstacle_list:
        rounded_x = round_of_rating(obstacle.x, min_x)
        rounded_y = round_of_rating(obstacle.y, min_y)
        rounded_z = round_of_rating(obstacle.z, min_z)
        #print("rounded x", rounded_x,"rounded y", rounded_y,"rounded z", rounded_z)
        by_rounded_coords[rounded_x, rounded_y, rounded_z].append(obstacle)
    #print(by_rounded_coords[80,80,4])
    return by_rounded_coords

def local_marks_list_finder(x_rounded, y_rounded, z_rounded, indexed_list):
    local_marks_list = []
    #print("x_rounded", x_rounded, "y rounded", y_rounded, "z rounded", z_rounded)
    extend_local_marks=local_marks_list.extend
    extend_local_marks(indexed_list[x_rounded - 1,y_rounded - 1,z_rounded - 1])
    extend_local_marks(indexed_list[x_rounded - 1,y_rounded,z_rounded - 1])
    extend_local_marks(indexed_list[x_rounded - 1,y_rounded - 1,z_rounded])
    extend_local_marks(indexed_list[x_rounded,y_rounded - 1,z_rounded - 1])
    extend_local_marks(indexed_list[x_rounded - 1,y_rounded,z_rounded])
    extend_local_marks(indexed_list[x_rounded,y_rounded - 1,z_rounded])
    extend_local_marks(indexed_list[x_rounded,y_rounded,z_rounded - 1])

    extend_local_marks(indexed_list[x_rounded,y_rounded,z_rounded])
    extend_local_marks(indexed_list[x_rounded + 1,y_rounded + 1,z_rounded + 1])
    extend_local_marks(indexed_list[x_rounded + 1,y_rounded,z_rounded + 1])
    extend_local_marks(indexed_list[x_rounded + 1,y_rounded + 1,z_rounded])
    extend_local_marks(indexed_list[x_rounded,y_rounded + 1,z_rounded + 1])
    extend_local_marks(indexed_list[x_rounded + 1,y_rounded,z_rounded])
    extend_local_marks(indexed_list[x_rounded,y_rounded + 1,z_rounded])
    extend_local_marks(indexed_list[x_rounded,y_rounded,z_rounded + 1])

    return local_marks_list


def Collision(x, y, z, obstacle_list):
    #start = time.time()
    collision = False
    for i in range(len(obstacle_list)):
        dx = x - obstacle_list[i].x
        dy = y - obstacle_list[i].y
        dz = z - obstacle_list[i].z

        #print("dx", dx)
        #print("dy", dy)
        #print("dz", dz)


        if abs(dx) < 0.4 and abs(dy) < 0.4 and abs(dz) < 0.4:
            print("abs collision check")
            collision = True
            break
        # dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)+ math.pow(dz, 2))
        # if dist <= 0.5:
        #    collision=True
        #    print("collision 111111111")
    if x <= -40 or x > 40:
        print("2")
        collision = True
    if y <= -40 or y > 40:
        print("3")
        collision = True
    if z < -0.1 or z > 4.1:
        print("4")
        collision = True

    #end = time.time()
    # print("Collision", end - start)
    return collision


def in_free_cell(x_pos, y_pos, z_pos, free_cell_list, start_x, start_y, start_z):
    inside_free=False
    if len(free_cell_list)==0:
        print("empty free cell list!!!!!!!!!!!!!!!!!!!")
    dx_start = x_pos - start_x
    dy_start = y_pos - start_y
    dz_start = z_pos - start_z
    dist_start = math.sqrt(dx_start * dx_start + dy_start * dy_start + dz_start * dz_start)
    if dist_start < 1.5:
        inside_free = True

    for i in range(len(free_cell_list)):
        dx = x_pos - free_cell_list[i].x
        dy = y_pos - free_cell_list[i].y
        dz = z_pos - free_cell_list[i].z

        if abs(dx) < 1 and abs(dy) < 1 and abs(dz) < 1:
            #print("dx", dx, "dy", dy, "dz", dz)
            #print("abs free check")
            inside_free = True
    return inside_free



def go_to_goal(near_x, near_y, near_z, x_diff, y_diff, z_diff, marks_list):
    #start = time.time()
    curr_x = near_x
    curr_y = near_y
    curr_z = near_z
    distance_time = 0.005
    step_num = 100

    collision = False
    curr_x = curr_x + x_diff * distance_time * step_num  # Go to node for 100 *distance_time
    curr_y = curr_y + y_diff * distance_time * step_num
    curr_z = curr_z + z_diff * distance_time * step_num
    #collision = Collision(curr_x, curr_y, curr_z, marks_list)
    # if collision==False:
    # break
    #end = time.time()
    #print("go_to_goal", end - start)
    return curr_x, curr_y, curr_z, collision


def go_to_goal2(near_x, near_y, near_z, x_diff, y_diff, z_diff, marks_list, free_cell_list, start_x, start_y, start_z):
    #start = time.time()
    curr_x = near_x
    curr_y = near_y
    curr_z = near_z
    distance_time = 0.01
    step_num = 100
    counter = 0
    collision = False
    inside_free =True
    while counter < step_num:
        curr_x = curr_x + x_diff * distance_time  # Go to node for 100 *distance_time
        curr_y = curr_y + y_diff * distance_time
        curr_z = curr_z + z_diff * distance_time
        counter = counter + 1
        if counter % 20 == 0:
            collision = Collision(curr_x, curr_y, curr_z, marks_list)
            if collision == True:
                curr_x = curr_x - int(counter /2) * x_diff * distance_time  # Go to node for 100 *distance_time
                curr_y = curr_y - int(counter /2) * y_diff * distance_time
                curr_z = curr_z - int(counter /2) * z_diff * distance_time
                collision = Collision(curr_x, curr_y, curr_z, marks_list)
                print("break")
                break
    #end = time.time()
    #print("go_to_goal2", end - start)
    inside_free = in_free_cell(curr_x, curr_y, curr_z, free_cell_list, start_x, start_y, start_z)
    print("inside free gotogoal2", inside_free)
    return curr_x, curr_y, curr_z, collision, inside_free






def backtracking(Node_List, final_node):
    #start = time.time()
    controls_x = []
    controls_y = []
    controls_z = []
    x_drone = []
    y_drone = []
    z_drone = []
    goal_node_list = []
    next_node=None
    times = []
    # index = len(Node_List)
    index = len(Node_List) - 1
    print(index)
    car_time = 0
    for i in range(final_node.total_parents):
        if index==None:
            break
        true_node = Node_List[index]
        if next_node is not None:
            dx = next_node.x - true_node.x
            dy =  next_node.y -true_node.y
            dz =  next_node.z -true_node.z
            #print("dx orientation",dx)
            #print("dy orientation",dy)
            #print("dz orientation",dz)

            yaw=math.atan2(dy, dx)
            pitch = math.atan2(math.sqrt(dy * dy + dx * dx), dz)
            quat = tf.transformations.quaternion_from_euler(0, pitch, yaw, 'syxz')
            true_node.x_ori = quat[0]
            true_node.y_ori = quat[1]
            true_node.z_ori = quat[2]
            true_node.w_ori = quat[3]

        goal_node_list.append(true_node)
        x_drone.append(true_node.x)
        y_drone.append(true_node.y)
        z_drone.append(true_node.z)

        controls_x.append(true_node.x_diff)
        controls_y.append(true_node.y_diff)
        controls_z.append(true_node.z_diff)
        index = true_node.parent_node
        next_node=true_node
    x_drone.reverse()
    y_drone.reverse()
    z_drone.reverse()
    controls_x.reverse()
    controls_y.reverse()
    controls_z.reverse()
    goal_node_list.reverse()
    #end = time.time()
    #print("backtracking", end - start)
    return controls_x, controls_y, controls_z, x_drone, y_drone, z_drone, goal_node_list


def choose_parent(curr_x, curr_y, curr_z, node_list, closest_index):
    #start = time.time()
    bounding_radius = 0.5
    dist_list = []
    parent_index = 0
    best_dist = 100000
    total_distance_constant = 1

    # if goal_distance_curr<1:
    #    parent_index=closest_index

    for i in range(len(node_list)):
        dx = curr_x - node_list[i].x
        dy = curr_y - node_list[i].y
        dz = curr_z - node_list[i].z
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2))

        if dist < bounding_radius:
            tot_dist = dist + node_list[i].total_distance * total_distance_constant
            if i == 0:
                tot_dist = tot_dist + 1
            if tot_dist < best_dist:
                # print("lowered", i)
                parent_index = i
                best_dist = tot_dist
        dist_list.append(dist)
        if min(dist_list) > bounding_radius:
            # print("no nodes inside bouding area")
            parent_index = closest_index

        # print("i", i, "tot_dist", tot_dist)
    # print("best dist", best_dist, "index", parent_index)
    parent_node = node_list[parent_index]
    #end = time.time()
    #print("choose_parent", end - start)
    return parent_index, parent_node






max_nodes_limit = 1000


def main_rrt(Node_List, start_x, start_y, start_z, marks_list,free_list, best_total_distance=3000, min_distance=0,
             phi_rotation=0):
    goal_reach_distance = 1
    if best_total_distance < 2000:
        goal_reach_distance = 1
    goal_reached = False
    goal_distance2 = 50

    counter_boost = 0
    x_half = (start_x - x_charge) / 2
    y_half = (start_y - y_charge) / 2

    #x_half = 0
    #y_half = 0
    #print("marks list", marks_list[1])
    start_ind=time.time()
    indexed_list = index_collision_list(marks_list)
    free_indexed_list = index_collision_list(free_list)
    end_ind=time.time()
    print("index time", end_ind-start_ind)
    #print("Free indexed list", free_indexed_list)
    #print("indexed list", indexed_list)

    Node_List_append=Node_List.append
    # print("indexed_list", indexed_list[63][35][4])
    # print("indexed list", indexed_list)
    while goal_reached == False and len(Node_List) < max_nodes_limit:
        x_rand, y_rand, z_rand = rand_node(counter_boost, best_total_distance, min_distance, phi_rotation, x_half,
                                           y_half)
        closest_node, goal_distance, closest_index = find_closest_node(x_rand, y_rand, z_rand, Node_List)
        x_diff, y_diff, z_diff = find_velocity(closest_node, x_rand, y_rand, z_rand)
        x_round = round_of_rating(closest_node.x, -40)
        y_round = round_of_rating(closest_node.y, -40)
        z_round = round_of_rating(closest_node.z, -2)
        local_marks_list1 = local_marks_list_finder(x_round, y_round, z_round, indexed_list)

        curr_x, curr_y, curr_z, collision = go_to_goal(closest_node.x, closest_node.y, closest_node.z, x_diff, y_diff,
                                                       z_diff, local_marks_list1)
        parent_index, parent_node = choose_parent(curr_x, curr_y, curr_z, Node_List, closest_index)
        x_diff2, y_diff2, z_diff2 = find_velocity(parent_node, curr_x, curr_y, curr_z)

        x_round2 = round_of_rating(parent_node.x, -40)
        y_round2 = round_of_rating(parent_node.y, -40)
        z_round2 = round_of_rating(parent_node.z, -2)

        #x_start_round = round_of_rating(start_x, -40)
        #y_start_round = round_of_rating(start_y, -40)
        #z_start_round = round_of_rating(start_z, -2)


        local_marks_list2 = local_marks_list_finder(x_round2, y_round2, z_round2, indexed_list)
        local_free_list = local_marks_list_finder(x_round2, y_round2, z_round2, free_indexed_list)
        #local_free_list_start = local_marks_list_finder(x_start_round, y_start_round, z_start_round, free_indexed_list)

        #print("local free list", local_free_list)

        #print("start free list", local_free_list_start, "x start", start_x, "y_start", start_y, "z start", start_z)

        curr_x, curr_y, curr_z, collision, inside_free = go_to_goal2(parent_node.x, parent_node.y, parent_node.z, x_diff2, y_diff2,
                                                        z_diff2, local_marks_list2, local_free_list, start_x, start_y, start_z)
        distance_travelled = math.sqrt(
            math.pow((curr_x - parent_node.x), 2) + math.pow((curr_y - parent_node.y), 2) + math.pow(
                (curr_z - parent_node.z), 2))
        # print("distance travelled",distance_travelled)
        if inside_free==False:
            print("inside free false and length of free", len(free_indexed_list))
        if collision == False:
            print("inside free", inside_free)
            Suc_Node = Node(x=curr_x, y=curr_y, z=curr_z, parent_node=parent_index, x_diff=x_diff, y_diff=y_diff,
                            z_diff=z_diff, total_distance=0)
            Suc_Node.total_parents = 1 + parent_node.total_parents
            Suc_Node.total_distance = distance_travelled + parent_node.total_distance
            Node_List_append(Suc_Node)
            goal_distance_curr = math.sqrt(
                math.pow((x_charge - Suc_Node.x), 2) + math.pow((y_charge - Suc_Node.y), 2) + math.pow(
                    (z_charge - Suc_Node.z), 2))
            if goal_distance_curr < goal_distance2:
                goal_distance2 = goal_distance_curr
            print("goal distance", goal_distance2, "new node x", Suc_Node.x)
            if goal_distance_curr <= goal_reach_distance and Suc_Node.total_distance < best_total_distance:
                print("reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", goal_distance_curr)
                goal_reached = True
        print("nodes", len(Node_List))
        counter_boost = counter_boost + 1
        if counter_boost>20000:
            break
    controls_x, controls_y, controls_z, x_drone, y_drone, z_drone, goal_node_list = backtracking(Node_List, Suc_Node)

    print("number of nodes", len(Node_List))
    print("boost number", boost_number)
    print("x last node", Suc_Node.x, "ylast node", Suc_Node.y, "z last node", Suc_Node.z)
    # print(70*0.0001*sum(controls_x)) # controls given in 70*0.0001

    # print('x_start',x_drone[0], 'x_end', x_drone[-1])
    print("total distance", Suc_Node.total_distance)
    # plt.plot(x_drone, y_drone, 'ro')
    # plt.axis([-10, 10, -10, 10])
    # plt.show()
    return Suc_Node, Node_List, goal_node_list


def informed_rrt(start_x, start_y, start_z, marks_list, free_list):
    total_distance_list = []
    total_node_list = []
    min_distance = math.sqrt(math.pow((x_charge - start_x), 2) + math.pow((y_charge - start_y), 2)) - 1
    print("min distance", min_distance)
    phi_rotation = math.atan2(y_charge - start_y, x_charge - start_x)
    start_node = Node(start_x, start_y, start_z, x_diff=0, y_diff=0, z_diff=0, total_distance=0)
    Node_List = [start_node]
    nodelist_append = total_node_list.append
    total_distance_append=total_distance_list.append
    for i in range(1):  # 100 iterations of rrt
        print("informed rrt iteration", i)
        print("node list length", len(Node_List))
        '''
        if i==0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y,start_z, marks_list)
            total_node_list.append(len(Node_List))
            total_distance_list.append(success_node.total_distance)
        '''
        if i == 0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y, start_z, marks_list,free_list,min_distance*3,
                                                               min_distance, phi_rotation)
            nodelist_append(len(Node_List))
            total_distance_append(success_node.total_distance)
        if i > 0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y, start_z, marks_list,free_list,
                                                               success_node.total_distance, min_distance, phi_rotation)
            nodelist_append(len(Node_List))
            total_distance_append(success_node.total_distance)
        if len(Node_List) == max_nodes_limit:
            break
    print("total node list", total_node_list)
    print("total distance list", total_distance_list)
    return success_node, Node_List, goal_node_list


def callback_gps(gps):
    global ranges
    global angle_min
    global state_drone
    global Node_list
    global goal_node_list
    global success_node
    global index_rrt
    global marks_list
    global free_list
    # print ("state drone", state_drone)
    if not len(marks_list) == 0 and not len(free_list)==0:
        rrt_poses = PoseArray()
        rrt_poses.header.stamp = rospy.Time.now()
        rrt_poses.header.frame_id = 'map'
        if state_drone == 1:
            print("length marks list", len(marks_list))
            print("length free list", len(free_list))

            start=time.time()
            success_node, Node_list, goal_node_list = informed_rrt(gps.pose.position.x, gps.pose.position.y,
                                                                   gps.pose.position.z, marks_list, free_list)
            end=time.time()
            print("total time", end-start)
            state_drone = 2
        curr_rrt = PoseStamped()
        # print("index", index_rrt)
        # curr_rrt.pose.position.x=goal_node_list[index_rrt].x
        # curr_rrt.pose.position.y=goal_node_list[index_rrt].y
        # curr_rrt.pose.position.z=goal_node_list[index_rrt].z
        # curr_rrt.header.frame_id = "map"
        # rrt_pub.publish(curr_rrt)
        # distance_curr_rrt = math.sqrt(math.pow((gps.pose.position.x - goal_node_list[index_rrt].x), 2) + math.pow((gps.pose.position.y - goal_node_list[index_rrt].y), 2) + math.pow((gps.pose.position.z - goal_node_list[index_rrt].z), 2))
        # if distance_curr_rrt<0.5 and index_rrt<len(goal_node_list)-1:
        #    index_rrt=index_rrt+1

        for i in range(len(goal_node_list)):
            curr_point_rrt = Pose()
            curr_point_rrt.position.x = goal_node_list[i].x
            curr_point_rrt.position.y =  goal_node_list[i].y
            curr_point_rrt.position.z =  goal_node_list[i].z
            curr_point_rrt.orientation.x =  goal_node_list[i].x_ori
            curr_point_rrt.orientation.y =  goal_node_list[i].y_ori
            curr_point_rrt.orientation.z =  goal_node_list[i].z_ori
            curr_point_rrt.orientation.w =  goal_node_list[i].w_ori
            rrt_poses.poses.append(curr_point_rrt)
        '''

        for i in range(len(Node_list)):
            curr_point_rrt = Pose()
            curr_point_rrt.position.x = Node_list[i].x
            curr_point_rrt.position.y = Node_list[i].y
            curr_point_rrt.position.z = Node_list[i].z
            curr_point_rrt.orientation.x =  Node_list[i].x_ori
            curr_point_rrt.orientation.y =  Node_list[i].y_ori
            curr_point_rrt.orientation.z =  Node_list[i].z_ori
            curr_point_rrt.orientation.w =  Node_list[i].w_ori
            rrt_poses.poses.append(curr_point_rrt)
        '''
        rrt_vis_pub.publish(rrt_poses)

        # curr_point_rrt.orientation.z = -3.14 / 2
        # curr_point_rrt.orientation.x = 2
        # print ("state drone2", state_drone)
        # print("success node", Node_list[3].z)


def callback_markers(marks):
    global marks_list
    global free_list
    if not len(free_list)==0:
        #marks_unpacked = marks.markers[16].points
        marks_list = marks.markers[16].points
        # print("marks list", marks_list[2].x)

        # print("marks_unpacked first point", marks_unpacked[0].x)
        # print(marks_unpacked)

def callback_free(vis_free):
    global free_list
    if len(free_list)==0:
        #print("len free list____________________________________________________________________________________________________", len(free_list))
        free_list = vis_free.markers[16].points
    #print(free_list)



def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    vis_mark_sub = rospy.Subscriber('/stl_aeplanner/occupied_cells_vis_array', MarkerArray, callback_markers)
    free_vis_mark_sub = rospy.Subscriber('/stl_aeplanner/free_cells_vis_array', MarkerArray, callback_free)
    rospy.spin()


if __name__ == '__main__':
    main()