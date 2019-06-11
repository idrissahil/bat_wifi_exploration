#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped, TwistStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Path
import random
import math
import tf
from collections import defaultdict

rospy.init_node('rrt_charging_drone')

rrt_pub = rospy.Publisher('/mavros/setpoint_position/local2', PoseStamped, queue_size=1)
rrt_vis_pub = rospy.Publisher('visual_marker_rrt', PoseArray, queue_size=1)
rrt_tree_vis_pub = rospy.Publisher('visual_marker_rrt_tree', PoseArray, queue_size=1)
pub_line_min_dist = rospy.Publisher('line_min_dist', Marker, queue_size=1)


rate = rospy.Rate(50)

ranges = []
# success_node=[]
Node_list = []
goal_node_list = []
marks_list = []
free_list = []

state_drone = 1
index_rrt = 0
total_distance_flown = 0

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


def rand_node(counter_boost, best_total_distance, min_distance, phi_rotation, x_half, y_half, goal_distance):
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
        #print("x half", x_half)
        #print("y half", y_half)
        z_min = 0
        z_max = 2
        x_max_i = c_best
        # print(x_max_i)
        print("c_best", c_best, "c_min", c_min)
        # print("error value",(c_best*c_best - c_min*c_min) )
        y_max_i = math.sqrt(abs(c_best * c_best - c_min * c_min))
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

    if counter_boost % 20 == 0:  # Boost the search towards the goal
        x_rand = x_charge
        y_rand = y_charge
        z_rand = z_charge
        # print("boost")
        boost_number[0] = boost_number[0] + 1

    '''
    if goal_distance>10:
        if counter_boost % 44 == 0:  # Boost the search towards the goal
            x_rand = 500
            y_rand = 500
            z_rand = z_charge
            # print("boost")
            boost_number[0] = boost_number[0] + 1

        if counter_boost % 56 == 0:  # Boost the search towards the goal
            x_rand = 500
            y_rand = -500
            z_rand = z_charge
            # print("boost")
            boost_number[0] = boost_number[0] + 1

        if counter_boost % 39 == 0:  # Boost the search towards the goal
            x_rand = -500
            y_rand = 500
            z_rand = z_charge
            # print("boost")
            boost_number[0] = boost_number[0] + 1

        if counter_boost % 66 == 0:  # Boost the search towards the goal
            x_rand = -500
            y_rand = -500
            z_rand = z_charge
            # print("boost")
            boost_number[0] = boost_number[0] + 1
        #end_rand = time.time()
        #print("rand_node", end_rand - start_rand)

    '''
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
    speed_limit = 3
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

def local_marks_list_finder(x_rounded, y_rounded, z_rounded, indexed_list, size_list):
    local_marks_list = []
    extend_local_marks=local_marks_list.extend

    '''
    #print("x_rounded", x_rounded, "y rounded", y_rounded, "z rounded", z_rounded)
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

    extend_local_marks(indexed_list[x_rounded,y_rounded,z_rounded + 2])
    extend_local_marks(indexed_list[x_rounded,y_rounded,z_rounded + 2])
    '''
    for a in range(size_list):
        for b in range(size_list):
            for c in range(size_list):
                extend_local_marks(indexed_list[x_rounded +a -size_list/2, y_rounded + b-size_list/2, z_rounded + c-size_list/2])

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


        if abs(dx) < 0.36 and abs(dy) < 0.36 and abs(dz) < 0.36:
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


def in_free_cell(x_pos, y_pos, z_pos, free_cell_list, start_x, start_y, start_z, unsorted_free_list):
    inside_free=False
    '''
    if len(free_cell_list)==0:
        print("empty free cell list!!!!!!!!!!!!!!!!!!!")
        start=time.time()
        for i in range(len(unsorted_free_list)):
            dx_f = abs(x_pos - unsorted_free_list[i].x)
            dy_f = abs(y_pos - unsorted_free_list[i].y)
            dz_f = abs(z_pos - unsorted_free_list[i].z)
            dist_f = math.sqrt(dx_f * dx_f + dy_f * dy_f + dz_f * dz_f)
            if abs(dx_f) < 0.7 and abs(dy_f) < 0.7 and abs(dz_f) < 0.7:
                print("free cell near exists")
                inside_free=True
        end=time.time()
        print("time of empty free list", end-start)
    '''
    dx_start = x_pos - start_x
    dy_start = y_pos - start_y
    dz_start = z_pos - start_z
    dist_start = math.sqrt(dx_start * dx_start + dy_start * dy_start + dz_start * dz_start)
    if dist_start < 1:
        inside_free = True

    for i in range(len(free_cell_list)):
        dx = x_pos - free_cell_list[i].x
        dy = y_pos - free_cell_list[i].y
        dz = z_pos - free_cell_list[i].z

        if abs(dx) < 0.8 and abs(dy) < 0.8 and abs(dz) < 0.8:
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


def go_to_goal2(near_x, near_y, near_z, x_diff, y_diff, z_diff, marks_list, free_cell_list, start_x, start_y, start_z, unsorted_free_list):
    global goto_collision
    #start = time.time()
    goto_collision=0
    curr_x = near_x
    curr_y = near_y
    curr_z = near_z
    distance_time = 0.01
    step_num = 100
    counter = 0
    collision_inverval_check=5
    collision = False
    inside_free =True
    while counter < step_num:
        curr_x = curr_x + x_diff * distance_time  # Go to node for 100 *distance_time
        curr_y = curr_y + y_diff * distance_time
        curr_z = curr_z + z_diff * distance_time
        counter = counter + 1
        if counter % collision_inverval_check == 0:
            collision = Collision(curr_x, curr_y, curr_z, marks_list)
            if collision == True:
                curr_x = curr_x - int(counter-collision_inverval_check) * x_diff * distance_time  # Go to node for 100 *distance_time
                curr_y = curr_y - int(counter-collision_inverval_check) * y_diff * distance_time
                curr_z = curr_z - int(counter-collision_inverval_check) * z_diff * distance_time
                collision = Collision(curr_x, curr_y, curr_z, marks_list)
                if collision==True:
                    goto_collision=1
                    print("goto collision", goto_collision)
                print("collision after back collision", collision)
                break
    #end = time.time()
    #print("go_to_goal2", end - start)
    inside_free = in_free_cell(curr_x, curr_y, curr_z, free_cell_list, start_x, start_y, start_z, unsorted_free_list)
    print("inside free gotogoal2", inside_free)
    return curr_x, curr_y, curr_z, collision, inside_free, goto_collision






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
    bounding_radius = 3
    dist_list=[]
    dist_list_inside=[]
    inside_bound_list=[]
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

        if 0.1 < dist < bounding_radius:
            inside_bound_list.append(i)
            dist_list_inside.append(dist)
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
    return parent_index, parent_node, inside_bound_list, dist_list_inside


def rewire(inside_bound_list, Suc_node, Node_List, dist_list, local_marks_list, local_free_list, start_x, start_y, start_z, unsorted_free_list):
    rewired_number=0
    rewired_list=[]
    index_Suc = len(Node_List) - 1
    for i in range(len(inside_bound_list)):
        curr_node = Node_List[inside_bound_list[i]]
        curr_total_distance = curr_node.total_distance
        new_total_distance=Suc_node.total_distance + dist_list[i]
        if curr_total_distance>new_total_distance:
            x_diff, y_diff, z_diff = find_velocity(Suc_node, curr_node.x, curr_node.y, curr_node.z)
            curr_x, curr_y, curr_z, collision, inside_free, goto_collision = go_to_goal2(Suc_node.x, Suc_node.y,Suc_node.z, x_diff, y_diff, z_diff, local_marks_list, local_free_list, start_x, start_y, start_z, unsorted_free_list)
            if collision==False and abs(curr_x-curr_node.x)<0.1 and abs(curr_y-curr_node.y)<0.1 and abs(curr_z==curr_node.z) <0.1:
                curr_node.total_distance = new_total_distance
                curr_node.parent_node = index_Suc
                Node_List[inside_bound_list[i]].total_distance = new_total_distance
                Node_List[inside_bound_list[i]].parent_node = index_Suc
                print("rewired 2 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                rewired_list.append(inside_bound_list[i])
                rewired_number = rewired_number + 1
    return Node_List, rewired_number, rewired_list


'''
def find_difference(original, new):
    total_missing_in_orig = 0
    for i in original:
        diff = set(original[i]) - set(new[i])
        if len(diff)>0:
            print("diff___________________________________________________________________________________", diff)
        total_missing_in_orig += len(diff)
    return total_missing_in_orig
'''
max_nodes_limit = 500


def main_rrt(Node_List, start_x, start_y, start_z, marks_list,free_list, best_total_distance=3000, min_distance=0,
             phi_rotation=0):
    goal_reach_distance = 0.1
    if best_total_distance < 2000:
        goal_reach_distance = 1
    goal_reached = False
    goal_distance2 = 50
    free_discards=0
    goto_collision_total=0
    collision_discards=0
    rewired_number_total=0
    counter_boost = 0
    x_half = ((start_x - x_charge) / 2)
    y_half = ((start_y - y_charge) / 2)

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
                                           y_half, goal_distance2)
        closest_node, goal_distance, closest_index = find_closest_node(x_rand, y_rand, z_rand, Node_List)
        x_diff, y_diff, z_diff = find_velocity(closest_node, x_rand, y_rand, z_rand)
        x_round = round_of_rating(closest_node.x, -40)
        y_round = round_of_rating(closest_node.y, -40)
        z_round = round_of_rating(closest_node.z, -2)
        local_marks_list1 = local_marks_list_finder(x_round, y_round, z_round, indexed_list, 6)
        curr_x, curr_y, curr_z, collision = go_to_goal(closest_node.x, closest_node.y, closest_node.z, x_diff, y_diff,
                                                       z_diff, local_marks_list1)
        parent_index, parent_node, inside_bound_list, dist_list_inside = choose_parent(curr_x, curr_y, curr_z, Node_List, closest_index)
        x_diff2, y_diff2, z_diff2 = find_velocity(parent_node, curr_x, curr_y, curr_z)

        x_round2 = round_of_rating(parent_node.x, -40)
        y_round2 = round_of_rating(parent_node.y, -40)
        z_round2 = round_of_rating(parent_node.z, -2)

        #x_start_round = round_of_rating(start_x, -40)
        #y_start_round = round_of_rating(start_y, -40)
        #z_start_round = round_of_rating(start_z, -2)


        local_marks_list2 = local_marks_list_finder(x_round2, y_round2, z_round2, indexed_list, 6)
        local_free_list = local_marks_list_finder(x_round2, y_round2, z_round2, free_indexed_list,6)
        #local_free_list_start = local_marks_list_finder(x_start_round, y_start_round, z_start_round, free_indexed_list)

        #print("local free list", local_free_list)

        #print("start free list", local_free_list_start, "x start", start_x, "y_start", start_y, "z start", start_z)

        curr_x, curr_y, curr_z, collision, inside_free ,goto_collision= go_to_goal2(parent_node.x, parent_node.y, parent_node.z, x_diff2, y_diff2,
                                                        z_diff2, local_marks_list2, local_free_list, start_x, start_y, start_z, free_list)
        distance_travelled = math.sqrt(
            math.pow((curr_x - parent_node.x), 2) + math.pow((curr_y - parent_node.y), 2) + math.pow(
                (curr_z - parent_node.z), 2))
        # print("distance travelled",distance_travelled)
        #print("indexed list start",free_indexed_list[40, 40, 2])
        goto_collision_total=goto_collision_total + goto_collision
        if inside_free==False:
            free_discards=free_discards+1
            print("inside free false and length of free", len(free_indexed_list))
        if collision==True:
            collision_discards=collision_discards+1

        if collision == False:
            print("inside free", inside_free)
            Suc_Node = Node(x=curr_x, y=curr_y, z=curr_z, parent_node=parent_index, x_diff=x_diff, y_diff=y_diff,
                            z_diff=z_diff, total_distance=0)
            Suc_Node.total_parents = 1 + parent_node.total_parents
            Suc_Node.total_distance = distance_travelled + parent_node.total_distance
            Node_List_append(Suc_Node)
            Node_List, rewired_number, rewired_list = rewire(inside_bound_list, Suc_Node, Node_List, dist_list_inside, local_marks_list2, local_free_list, start_x, start_y, start_z, free_list)
            rewired_number_total=rewired_number+rewired_number_total
            goal_distance_curr = math.sqrt(math.pow((x_charge - Suc_Node.x), 2) + math.pow((y_charge - Suc_Node.y), 2) + math.pow((z_charge - Suc_Node.z), 2))

            if goal_distance_curr < goal_distance2:
                goal_distance2 = goal_distance_curr
            print("goal distance", goal_distance2, "new node x", Suc_Node.x)

            if rewired_number>0:
                for i in range(len(rewired_list)):
                    curr_node_rewired = Node_List[rewired_list[i]]
                    goal_distance_curr_rewired = math.sqrt(math.pow((x_charge - curr_node_rewired.x), 2) + math.pow((y_charge - curr_node_rewired.y), 2) + math.pow((z_charge - curr_node_rewired.z), 2))
                    print("rewired distance", goal_distance_curr_rewired)
                    if goal_distance_curr_rewired <= goal_reach_distance and curr_node_rewired.total_distance < best_total_distance:
                        print("reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", goal_distance_curr_rewired)
                        goal_reached = True
                        Suc_Node=curr_node_rewired

           
            for i in range(len(inside_bound_list)):
                curr_node_rewired = Node_List[inside_bound_list[i]]
                goal_distance_curr_rewired = math.sqrt(
                    math.pow((x_charge - curr_node_rewired.x), 2) + math.pow((y_charge - curr_node_rewired.y), 2) + math.pow(
                        (z_charge - curr_node_rewired.z), 2))

            if goal_distance_curr <= goal_reach_distance and Suc_Node.total_distance < best_total_distance:
                print("reached !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", goal_distance_curr)
                goal_reached = True
        print("nodes", len(Node_List))
        counter_boost = counter_boost + 1
        if counter_boost>40000:
            break
    controls_x, controls_y, controls_z, x_drone, y_drone, z_drone, goal_node_list = backtracking(Node_List, Suc_Node)

    print("number of nodes", len(Node_List))
    print("boost number", boost_number)
    print("x last node", Suc_Node.x, "ylast node", Suc_Node.y, "z last node", Suc_Node.z)
    # print(70*0.0001*sum(controls_x)) # controls given in 70*0.0001
    #print("total goto collison", goto_collision_total)
    print("total rewired number", rewired_number_total)
    # print('x_start',x_drone[0], 'x_end', x_drone[-1])
    print("total distance", Suc_Node.total_distance)
    print("collison discards", collision_discards, "free discards",free_discards)
    # plt.plot(x_drone, y_drone, 'ro')
    # plt.axis([-10, 10, -10, 10])
    # plt.show()
    return Suc_Node, Node_List, goal_node_list


def informed_rrt(start_x, start_y, start_z, marks_list, free_list, total_distance_flown):
    total_distance_list = []
    total_node_list = []
    min_distance = math.sqrt(math.pow((x_charge - start_x), 2) + math.pow((y_charge - start_y), 2)) - 1
    print("min distance", min_distance)
    phi_rotation = math.atan2(y_charge - start_y, x_charge - start_x)
    start_node = Node(start_x, start_y, start_z, x_diff=0, y_diff=0, z_diff=0, total_distance=0)
    Node_List = [start_node]
    nodelist_append = total_node_list.append
    total_distance_append=total_distance_list.append
    iterations=5
    start_time=time.time()
    for i in range(iterations):  # 100 iterations of rrt
        print("informed rrt iteration", i)
        print("node list length", len(Node_List))
        '''
        if i==0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y,start_z, marks_list)
            total_node_list.append(len(Node_List))
            total_distance_list.append(success_node.total_distance)
        '''
        if i == 0:
            success_node, Node_List, goal_node_list = main_rrt(Node_List, start_x, start_y, start_z, marks_list,free_list,total_distance_flown,
                                                               min_distance, phi_rotation)
            nodelist_append(len(Node_List))
            total_distance_append(success_node.total_distance)
        curr_time=time.time()
        tot_time=curr_time-start_time
        if success_node.total_distance<min_distance or tot_time>3 or abs(success_node.total_distance-min_distance)<2:
            i=iterations
        if i > 0 and i<iterations:
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
    global total_distance_flown
    global free_list
    # print ("state drone", state_drone)
    if not len(marks_list) == 0 and not len(free_list)==0 and total_distance_flown>0:
        rrt_poses = PoseArray()
        rrt_poses.header.stamp = rospy.Time.now()
        rrt_poses.header.frame_id = 'map'
        rrt_tree_poses = PoseArray()
        rrt_tree_poses.header.stamp = rospy.Time.now()
        rrt_tree_poses.header.frame_id = 'map'
        if state_drone == 1:
            print("length marks list", len(marks_list))
            print("length free list", len(free_list))

            start=time.time()
            success_node, Node_list, goal_node_list = informed_rrt(gps.pose.position.x, gps.pose.position.y,
                                                                   gps.pose.position.z, marks_list, free_list, total_distance_flown)
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
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # marker line points
        marker.points = []

        for i in range(len(goal_node_list)):
            curr_point_marker = Point()
            curr_point_marker.x = goal_node_list[i].x
            curr_point_marker.y = goal_node_list[i].y
            curr_point_marker.z = goal_node_list[i].z
            marker.points.append(curr_point_marker)

            curr_point_rrt = Pose()
            curr_point_rrt.position.x = goal_node_list[i].x
            curr_point_rrt.position.y =  goal_node_list[i].y
            curr_point_rrt.position.z =  goal_node_list[i].z
            curr_point_rrt.orientation.x =  goal_node_list[i].x_ori
            curr_point_rrt.orientation.y =  goal_node_list[i].y_ori
            curr_point_rrt.orientation.z =  goal_node_list[i].z_ori
            curr_point_rrt.orientation.w =  goal_node_list[i].w_ori
            rrt_poses.poses.append(curr_point_rrt)
        rrt_vis_pub.publish(rrt_poses)
        pub_line_min_dist.publish(marker)

        for i in range(len(Node_list)):
            curr_point_rrt_tree = Pose()
            curr_point_rrt_tree.position.x = Node_list[i].x
            curr_point_rrt_tree.position.y = Node_list[i].y
            curr_point_rrt_tree.position.z = Node_list[i].z
            curr_point_rrt_tree.orientation.x =  Node_list[i].x_ori
            curr_point_rrt_tree.orientation.y =  Node_list[i].y_ori
            curr_point_rrt_tree.orientation.z =  Node_list[i].z_ori
            curr_point_rrt_tree.orientation.w =  Node_list[i].w_ori
            rrt_tree_poses.poses.append(curr_point_rrt_tree)
        rrt_tree_vis_pub.publish(rrt_tree_poses)



        # curr_point_rrt.orientation.z = -3.14 / 2
        # curr_point_rrt.orientation.x = 2
        # print ("state drone2", state_drone)
        # print("success node", Node_list[3].z)


def callback_markers(marks):
    global marks_list
    global free_list
    global total_distance_flown
    if not len(free_list)==0 and total_distance_flown>0:
        #marks_unpacked = marks.markers[16].points
        marks_list = marks.markers[16].points
        # print("marks list", marks_list[2].x)

        # print("marks_unpacked first point", marks_unpacked[0].x)
        # print(marks_unpacked)

def callback_free(vis_free):
    global free_list
    global total_distance_flown
    if len(free_list)==0 and total_distance_flown>0:
        #print("len free list____________________________________________________________________________________________________", len(free_list))
        free_list = vis_free.markers[16].points
    #print(free_list)

'''
def callback_path(path):
    for i in range(len(path.poses)):
        print("path!!!!!!!!!!!!!!", )

'''

def callback_dist(dist_flown):
    global total_distance_flown
    total_distance_flown = dist_flown.pose.orientation.w

def main():
    gps_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_gps)
    vis_mark_sub = rospy.Subscriber('/stl_aeplanner/occupied_cells_vis_array', MarkerArray, callback_markers)
    free_vis_mark_sub = rospy.Subscriber('/stl_aeplanner/free_cells_vis_array', MarkerArray, callback_free)
    #explore_path_sub = rospy.Subscriber('/stl_aeplanner/ltl_path', Path, callback_path)
    dist_sub = rospy.Subscriber('distance_flown', PoseStamped, callback_dist)
    rospy.spin()


if __name__ == '__main__':
    main()
